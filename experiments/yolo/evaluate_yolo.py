#!/usr/bin/env python3
"""
YOLO Experiment Evaluation Script

This script parses LTTng traces from YOLO experiments and generates:
- Metrics: layer processing times, detection counts, total runtime, throughput
- Plots: runtime vs batch size, layer timing analysis, detection quality
- CSV/JSON exports of results
"""

import os
import sys
import json
import subprocess
from pathlib import Path
from collections import defaultdict
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor
import multiprocessing

# Configuration
EXPERIMENT_DIR = Path("/home/vscode/workspace/experiments/yolo")
TRACE_DIR = EXPERIMENT_DIR / "traces"
RESULTS_DIR = EXPERIMENT_DIR / "results"
PLOTS_DIR = RESULTS_DIR / "plots"

# Create output directories
RESULTS_DIR.mkdir(exist_ok=True)
PLOTS_DIR.mkdir(exist_ok=True)


class TraceEvent:
    """Represents a single trace event"""

    def __init__(self, timestamp, event_name, fields):
        self.timestamp = timestamp
        self.event_name = event_name
        self.fields = fields

    def __repr__(self):
        return f"TraceEvent({self.timestamp}, {self.event_name})"


def parse_trace_directory(trace_dir):
    """
    Parse a single trace directory using babeltrace
    """
    print(f"  Parsing trace: {trace_dir.name}")

    # Use babeltrace2 with filter if available (faster)
    try:
        # Try babeltrace2 first (newer, faster)
        result = subprocess.run(
            ['babeltrace2', '--names', 'none', str(trace_dir)],
            capture_output=True,
            text=True,
            check=True
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        # Fall back to babeltrace
        try:
            result = subprocess.run(
                ['babeltrace', str(trace_dir)],
                capture_output=True,
                text=True,
                check=True
            )
        except subprocess.CalledProcessError as e:
            print(f"    Error running babeltrace: {e}")
            return []
        except FileNotFoundError:
            print("    Error: babeltrace not found. Please install lttng-tools.")
            return []

    # Pre-filter lines to only process anytime events
    anytime_lines = [line for line in result.stdout.split(
        '\n') if 'anytime:' in line]

    events = []
    for line in anytime_lines:
        if not line.strip() or not 'anytime:' in line:
            continue

        # Parse the trace line
        # Format: [timestamp] hostname anytime:event_name: { fields }
        try:
            # Extract timestamp (in brackets)
            ts_start = line.find('[')
            ts_end = line.find(']')
            if ts_start == -1 or ts_end == -1:
                continue

            timestamp_str = line[ts_start+1:ts_end]
            # Convert timestamp to nanoseconds
            timestamp = float(timestamp_str.replace(':', '')) * 1e9

            # Extract event name (format: "anytime:event_name:")
            event_start = line.find('anytime:')
            if event_start == -1:
                continue

            event_name_part = line[event_start:]
            event_name_end = event_name_part.find(':')
            if event_name_end == -1:
                continue

            event_name = event_name_part[8:event_name_end]  # Skip "anytime:"

            # Extract fields (in braces)
            fields_start = line.find('{')
            fields_end = line.rfind('}')
            if fields_start == -1 or fields_end == -1:
                fields = {}
            else:
                fields_str = line[fields_start+1:fields_end]
                fields = parse_fields(fields_str)

            events.append(TraceEvent(timestamp, event_name, fields))

        except Exception as e:
            print(f"    Warning: Failed to parse line: {line[:100]}")
            print(f"    Error: {e}")
            continue

    print(f"    Parsed {len(events)} events")
    return events


def parse_fields(fields_str):
    """Parse the fields string into a dictionary"""
    fields = {}

    # Split by comma, but be careful with strings that might contain commas
    parts = []
    current = []
    in_string = False

    for char in fields_str + ',':
        if char == '"':
            in_string = not in_string
        elif char == ',' and not in_string:
            parts.append(''.join(current).strip())
            current = []
            continue
        current.append(char)

    for part in parts:
        if not part or '=' not in part:
            continue

        key, value = part.split('=', 1)
        key = key.strip()
        value = value.strip()

        # Remove quotes from strings
        if value.startswith('"') and value.endswith('"'):
            value = value[1:-1]
        # Try to convert to number
        else:
            try:
                if '.' in value:
                    value = float(value)
                else:
                    value = int(value)
            except ValueError:
                pass

        fields[key] = value

    return fields


def analyze_yolo_trace(trace_dir):
    """
    Analyze a single YOLO trace directory
    Returns a dictionary with metrics
    """
    events = parse_trace_directory(trace_dir)

    if not events:
        return None

    # Extract configuration from trace name
    trace_name = trace_dir.name
    config = parse_trace_name(trace_name)

    # Initialize metrics
    metrics = {
        'trace_name': trace_name,
        'config': config,
        'goals_processed': 0,
        'total_layers_processed': 0,
        'layer_times': defaultdict(list),  # layer_num -> [times]
        'detection_counts': [],  # per goal
        'total_runtime_ms': [],  # per goal
        'throughput_imgs_per_sec': 0,
    }

    # Track current goal processing
    current_goal = {
        'goal_start': None,
        'layer_starts': {},  # layer_num -> timestamp
        'layers_processed': 0,
        'detections': 0,
    }

    for event in events:
        if event.event_name == 'yolo_init':
            # Server initialization
            pass

        elif event.event_name == 'anytime_base_activate':
            # Goal started
            current_goal['goal_start'] = event.timestamp
            current_goal['layers_processed'] = 0
            current_goal['detections'] = 0
            current_goal['layer_starts'] = {}

        elif event.event_name == 'yolo_layer_start':
            layer_num = event.fields.get('layer_num', 0)
            current_goal['layer_starts'][layer_num] = event.timestamp

        elif event.event_name == 'yolo_layer_end':
            layer_num = event.fields.get('layer_num', 0)
            if layer_num in current_goal['layer_starts']:
                layer_time_ns = event.timestamp - \
                    current_goal['layer_starts'][layer_num]
                layer_time_ms = layer_time_ns / 1e6
                metrics['layer_times'][layer_num].append(layer_time_ms)
                current_goal['layers_processed'] = layer_num

        elif event.event_name == 'yolo_exit_calculation_end':
            # Detection count after exit calculation
            num_detections = event.fields.get('num_detections', 0)
            current_goal['detections'] = num_detections

        elif event.event_name == 'yolo_result':
            # Goal completed
            total_detections = event.fields.get('total_detections', 0)
            processed_layers = event.fields.get('processed_layers', 0)

            if current_goal['goal_start']:
                runtime_ns = event.timestamp - current_goal['goal_start']
                runtime_ms = runtime_ns / 1e6
                metrics['total_runtime_ms'].append(runtime_ms)
                metrics['detection_counts'].append(total_detections)
                metrics['total_layers_processed'] += processed_layers
                metrics['goals_processed'] += 1

    # Calculate summary statistics
    if metrics['total_runtime_ms']:
        total_time_sec = sum(metrics['total_runtime_ms']) / 1000
        metrics['throughput_imgs_per_sec'] = metrics['goals_processed'] / \
            total_time_sec
        metrics['avg_runtime_ms'] = np.mean(metrics['total_runtime_ms'])
        metrics['std_runtime_ms'] = np.std(metrics['total_runtime_ms'])
    else:
        metrics['avg_runtime_ms'] = 0
        metrics['std_runtime_ms'] = 0

    if metrics['detection_counts']:
        metrics['avg_detections'] = np.mean(metrics['detection_counts'])
        metrics['std_detections'] = np.std(metrics['detection_counts'])
    else:
        metrics['avg_detections'] = 0
        metrics['std_detections'] = 0

    # Calculate average layer times
    metrics['avg_layer_times'] = {}
    for layer_num, times in metrics['layer_times'].items():
        metrics['avg_layer_times'][layer_num] = {
            'mean': np.mean(times),
            'std': np.std(times),
            'min': np.min(times),
            'max': np.max(times),
        }

    return metrics


def parse_trace_name(trace_name):
    """
    Parse trace name to extract configuration
    Expected format: phase1_baseline, phase3_max_throughput, etc.
    """
    config = {
        'phase': None,
        'batch_size': None,
        'mode': None,
        'threading': None,
    }

    if 'phase1' in trace_name or 'baseline' in trace_name:
        config['phase'] = 1
        config['batch_size'] = 1
        config['mode'] = 'proactive'
    elif 'phase3' in trace_name or 'max_throughput' in trace_name:
        config['phase'] = 3
        config['batch_size'] = 25
        config['mode'] = 'proactive'

    # Extract more details if present
    parts = trace_name.split('_')
    for part in parts:
        if part.startswith('batch'):
            try:
                config['batch_size'] = int(part.replace('batch', ''))
            except:
                pass
        elif part in ['reactive', 'proactive']:
            config['mode'] = part
        elif part in ['single', 'multi']:
            config['threading'] = part

    return config


def plot_layer_times(all_metrics):
    """
    Plot average layer processing times
    """
    print("\n  Creating layer times plot...")

    fig, ax = plt.subplots(figsize=(12, 6))

    for metrics in all_metrics:
        if not metrics['avg_layer_times']:
            continue

        layers = sorted(metrics['avg_layer_times'].keys())
        means = [metrics['avg_layer_times'][l]['mean'] for l in layers]
        stds = [metrics['avg_layer_times'][l]['std'] for l in layers]

        label = f"{metrics['config']['phase']} (batch={metrics['config']['batch_size']})"
        ax.errorbar(layers, means, yerr=stds,
                    marker='o', label=label, capsize=5)

    ax.set_xlabel('Layer Number')
    ax.set_ylabel('Processing Time (ms)')
    ax.set_title('YOLO Layer Processing Times')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(PLOTS_DIR / 'layer_times.png', dpi=300)
    plt.close()
    print(f"    Saved: {PLOTS_DIR / 'layer_times.png'}")


def plot_throughput_comparison(all_metrics):
    """
    Plot throughput comparison across configurations
    """
    print("\n  Creating throughput comparison plot...")

    # Group by batch size
    batch_sizes = []
    throughputs = []
    labels = []

    for metrics in all_metrics:
        batch_size = metrics['config']['batch_size']
        if batch_size:
            batch_sizes.append(batch_size)
            throughputs.append(metrics['throughput_imgs_per_sec'])
            labels.append(metrics['trace_name'])

    if not batch_sizes:
        print("    No throughput data available")
        return

    fig, ax = plt.subplots(figsize=(10, 6))

    ax.bar(range(len(batch_sizes)), throughputs)
    ax.set_xlabel('Configuration')
    ax.set_ylabel('Throughput (images/sec)')
    ax.set_title('YOLO Throughput Comparison')
    ax.set_xticks(range(len(batch_sizes)))
    ax.set_xticklabels(
        [f"Batch={bs}" for bs in batch_sizes], rotation=45, ha='right')
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig(PLOTS_DIR / 'throughput_comparison.png', dpi=300)
    plt.close()
    print(f"    Saved: {PLOTS_DIR / 'throughput_comparison.png'}")


def plot_runtime_vs_batch_size(all_metrics):
    """
    Plot average runtime vs batch size
    """
    print("\n  Creating runtime vs batch size plot...")

    # Group by batch size
    data = defaultdict(list)

    for metrics in all_metrics:
        batch_size = metrics['config']['batch_size']
        if batch_size and metrics['avg_runtime_ms'] > 0:
            data[batch_size].append(metrics['avg_runtime_ms'])

    if not data:
        print("    No runtime data available")
        return

    batch_sizes = sorted(data.keys())
    means = [np.mean(data[bs]) for bs in batch_sizes]
    stds = [np.std(data[bs]) if len(data[bs]) > 1 else 0 for bs in batch_sizes]

    fig, ax = plt.subplots(figsize=(10, 6))

    ax.errorbar(batch_sizes, means, yerr=stds, marker='o',
                capsize=5, linewidth=2, markersize=8)
    ax.set_xlabel('Batch Size')
    ax.set_ylabel('Average Runtime (ms)')
    ax.set_title('YOLO Average Runtime vs Batch Size')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(PLOTS_DIR / 'runtime_vs_batch_size.png', dpi=300)
    plt.close()
    print(f"    Saved: {PLOTS_DIR / 'runtime_vs_batch_size.png'}")


def export_results(all_metrics):
    """
    Export results to CSV and JSON
    """
    print("\n  Exporting results...")

    # Create summary dataframe
    summary_data = []
    for metrics in all_metrics:
        row = {
            'trace_name': metrics['trace_name'],
            'phase': metrics['config']['phase'],
            'batch_size': metrics['config']['batch_size'],
            'mode': metrics['config']['mode'],
            'threading': metrics['config']['threading'],
            'goals_processed': metrics['goals_processed'],
            'avg_runtime_ms': metrics['avg_runtime_ms'],
            'std_runtime_ms': metrics['std_runtime_ms'],
            'avg_detections': metrics['avg_detections'],
            'std_detections': metrics['std_detections'],
            'throughput_imgs_per_sec': metrics['throughput_imgs_per_sec'],
        }
        summary_data.append(row)

    df = pd.DataFrame(summary_data)

    # Save CSV
    csv_path = RESULTS_DIR / 'yolo_summary.csv'
    df.to_csv(csv_path, index=False)
    print(f"    Saved: {csv_path}")

    # Save JSON with full details
    json_path = RESULTS_DIR / 'yolo_detailed.json'

    # Convert defaultdict and numpy types for JSON serialization
    export_data = []
    for metrics in all_metrics:
        clean_metrics = {
            'trace_name': metrics['trace_name'],
            'config': metrics['config'],
            'goals_processed': metrics['goals_processed'],
            'total_layers_processed': metrics['total_layers_processed'],
            'avg_runtime_ms': float(metrics['avg_runtime_ms']),
            'std_runtime_ms': float(metrics['std_runtime_ms']),
            'avg_detections': float(metrics['avg_detections']),
            'std_detections': float(metrics['std_detections']),
            'throughput_imgs_per_sec': float(metrics['throughput_imgs_per_sec']),
            'avg_layer_times': {
                int(k): {
                    'mean': float(v['mean']),
                    'std': float(v['std']),
                    'min': float(v['min']),
                    'max': float(v['max']),
                }
                for k, v in metrics['avg_layer_times'].items()
            },
            'detection_counts': [int(x) for x in metrics['detection_counts']],
            'total_runtime_ms': [float(x) for x in metrics['total_runtime_ms']],
        }
        export_data.append(clean_metrics)

    with open(json_path, 'w') as f:
        json.dump(export_data, f, indent=2)
    print(f"    Saved: {json_path}")

    # Print summary to console
    print("\n" + "="*80)
    print("YOLO EXPERIMENT SUMMARY")
    print("="*80)
    print(df.to_string(index=False))
    print("="*80 + "\n")


def main():
    """Main evaluation function"""
    print("="*80)
    print("YOLO EXPERIMENT EVALUATION")
    print("="*80)
    print(f"Trace directory: {TRACE_DIR}")
    print(f"Results directory: {RESULTS_DIR}")

    # Find all trace directories
    trace_dirs = [d for d in TRACE_DIR.iterdir() if d.is_dir()]

    if not trace_dirs:
        print("\nError: No trace directories found!")
        print(
            f"Please run experiments first to generate traces in: {TRACE_DIR}")
        return 1

    print(f"\nFound {len(trace_dirs)} trace directories")

    # Analyze traces in parallel
    print("\nAnalyzing traces...")
    all_metrics = []

    # Use single-threaded for debugging, parallel for production
    use_parallel = len(trace_dirs) > 2

    if use_parallel:
        with ProcessPoolExecutor(max_workers=multiprocessing.cpu_count()) as executor:
            results = executor.map(analyze_yolo_trace, trace_dirs)
            all_metrics = [r for r in results if r is not None]
    else:
        for trace_dir in trace_dirs:
            result = analyze_yolo_trace(trace_dir)
            if result:
                all_metrics.append(result)

    if not all_metrics:
        print("\nError: No valid metrics extracted from traces!")
        return 1

    print(f"\nSuccessfully analyzed {len(all_metrics)} traces")

    # Generate plots
    print("\nGenerating plots...")
    plot_layer_times(all_metrics)
    plot_throughput_comparison(all_metrics)
    plot_runtime_vs_batch_size(all_metrics)

    # Export results
    export_results(all_metrics)

    print("\n" + "="*80)
    print("EVALUATION COMPLETE")
    print("="*80)
    print(f"Results saved to: {RESULTS_DIR}")
    print(f"Plots saved to: {PLOTS_DIR}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
