#!/bin/bash
# Collect paper figures and tables into paper_figures/ for easy reviewer access
#
# Usage: ./scripts/collect_figures.sh
#
# Copies only the figures and tables that appear in the paper.
# Files are prefixed with their paper reference (e.g., figure_5a_, table_1_).
# Only copies files that exist — safe to run after partial experiments.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
OUTPUT_DIR="${WORKSPACE_DIR}/paper_figures"

MC_DIR="${WORKSPACE_DIR}/experiments/monte_carlo"
IF_DIR="${WORKSPACE_DIR}/experiments/interference"
YOLO_DIR="${WORKSPACE_DIR}/experiments/yolo"

mkdir -p "${OUTPUT_DIR}"

collected=0

copy_if_exists() {
    local src="$1"
    local dst="$2"
    if [ -f "${src}" ]; then
        cp "${src}" "${dst}"
        echo "  Collected: $(basename "${dst}")"
        collected=$((collected + 1))
    fi
}

echo "Collecting paper figures into ${OUTPUT_DIR}/ ..."
echo ""

# Figure 5a: Monte Carlo — segment count vs batch size
copy_if_exists "${MC_DIR}/results/plots/batch_size_vs_iterations.pdf" \
               "${OUTPUT_DIR}/figure_5a_batch_size_vs_iterations.pdf"

# Figure 5b: Monte Carlo — cancellation delay
copy_if_exists "${MC_DIR}/results/plots/cancellation_delay.pdf" \
               "${OUTPUT_DIR}/figure_5b_cancellation_delay.pdf"

# Figure 6a: Interference — jitter vs batch size
copy_if_exists "${IF_DIR}/results/plots/jitter_vs_batch_size.pdf" \
               "${OUTPUT_DIR}/figure_6a_jitter_vs_batch_size.pdf"

# Figure 6b: Interference — compute time vs batch size
copy_if_exists "${IF_DIR}/results/plots/compute_time_vs_batch_size.pdf" \
               "${OUTPUT_DIR}/figure_6b_compute_time_vs_batch_size.pdf"

# Table I: Interference — aggregated metrics
copy_if_exists "${IF_DIR}/results/aggregated_results.csv" \
               "${OUTPUT_DIR}/table_1_interference_metrics.csv"

# Figure 7a: YOLO — quality ratio progression
copy_if_exists "${YOLO_DIR}/results/quality_analysis/quality_ratio_progression.pdf" \
               "${OUTPUT_DIR}/figure_7a_quality_ratio_progression.pdf"

# Figure 7b: YOLO — throughput comparison
copy_if_exists "${YOLO_DIR}/results/runtime_analysis/throughput_comparison.png" \
               "${OUTPUT_DIR}/figure_7b_throughput_comparison.png"

echo ""
if [ ${collected} -eq 0 ]; then
    echo "No figures found. Run experiments first, then re-run this script."
else
    echo "Collected ${collected} file(s) in ${OUTPUT_DIR}/"
fi
