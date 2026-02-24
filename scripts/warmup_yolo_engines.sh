#!/bin/bash
# Pre-build all TensorRT engines for YOLO experiments.
# Run once after downloading weights or changing GPU/TensorRT version.
#
# Usage: ./scripts/warmup_yolo_engines.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
WEIGHTS_DIR="${WORKSPACE_DIR}/packages/src/anytime_yolo/weights_32"

echo "Warming up YOLO TensorRT engines..."
ros2 run anytime_yolo warmup_engines "${WEIGHTS_DIR}"
echo "Engine warmup complete."
