#!/bin/bash
# Monte Carlo Experiment Launcher
# Quick access script to run Monte Carlo experiments from workspace root

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
MONTE_CARLO_DIR="${SCRIPT_DIR}/experiments/monte_carlo"

# Check if experiments directory exists
if [ ! -d "${MONTE_CARLO_DIR}" ]; then
    echo "Error: Monte Carlo experiments directory not found"
    echo "Expected: ${MONTE_CARLO_DIR}"
    exit 1
fi

# Parse command line arguments
case "$1" in
    test)
        echo "Running single configuration test..."
        cd "${MONTE_CARLO_DIR}"
        ./test_single_config.sh
        ;;
    run)
        echo "Running full experiment suite..."
        cd "${MONTE_CARLO_DIR}"
        ./run_monte_carlo_experiments.sh
        ;;
    evaluate)
        echo "Running evaluation only..."
        cd "${MONTE_CARLO_DIR}"
        python3 evaluate_monte_carlo.py
        ;;
    clean)
        echo "Cleaning experiment data..."
        read -p "This will delete all traces and results. Are you sure? (y/N) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "${MONTE_CARLO_DIR}/traces/"*
            rm -rf "${MONTE_CARLO_DIR}/results/"*
            echo "Cleaned traces and results directories"
        else
            echo "Cancelled"
        fi
        ;;
    status)
        echo "========================================="
        echo "Monte Carlo Experiment Status"
        echo "========================================="
        echo ""
        
        # Count configs
        config_count=$(ls -1 "${MONTE_CARLO_DIR}/configs/"*_server.yaml 2>/dev/null | wc -l)
        echo "Configurations: ${config_count}"
        
        # Count traces
        trace_count=$(ls -1d "${MONTE_CARLO_DIR}/traces/"* 2>/dev/null | wc -l)
        echo "Trace directories: ${trace_count}"
        
        # Check for results
        if [ -f "${MONTE_CARLO_DIR}/results/aggregated_results.csv" ]; then
            result_count=$(tail -n +2 "${MONTE_CARLO_DIR}/results/aggregated_results.csv" | wc -l)
            echo "Results analyzed: ${result_count} configurations"
        else
            echo "Results analyzed: 0"
        fi
        
        # Check for plots
        plot_count=$(ls -1 "${MONTE_CARLO_DIR}/results/plots/"*.png 2>/dev/null | wc -l)
        echo "Plots generated: ${plot_count}"
        
        echo ""
        ;;
    help|*)
        echo "Monte Carlo Experiment Launcher"
        echo ""
        echo "Usage: $0 [COMMAND]"
        echo ""
        echo "Commands:"
        echo "  test      - Run quick test with single configuration (10s)"
        echo "  run       - Run full experiment suite (72 runs, ~40 min)"
        echo "  evaluate  - Run evaluation on existing traces"
        echo "  status    - Show experiment status"
        echo "  clean     - Delete all traces and results"
        echo "  help      - Show this help message"
        echo ""
        echo "Examples:"
        echo "  $0 test              # Quick test"
        echo "  $0 run               # Run all experiments"
        echo "  $0 status            # Check progress"
        echo ""
        exit 0
        ;;
esac
