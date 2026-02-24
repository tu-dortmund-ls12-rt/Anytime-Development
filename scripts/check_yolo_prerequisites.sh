#!/bin/bash
# Check YOLO prerequisites (weights + images) before running experiments.
# Automatically downloads and extracts if any files are missing or incomplete.
#
# Usage: ./scripts/check_yolo_prerequisites.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
WEIGHTS_DIR="${WORKSPACE_DIR}/packages/src/anytime_yolo/weights_32"
WEIGHTS_PARENT="${WORKSPACE_DIR}/packages/src/anytime_yolo"
IMAGES_DIR="${WORKSPACE_DIR}/packages/src/video_publisher/images"
IMAGES_PARENT="${WORKSPACE_DIR}/packages/src/video_publisher"

WEIGHTS_URL="https://tu-dortmund.sciebo.de/s/gmGSJEsFgwKb6MY/download"
IMAGES_URL="https://tu-dortmund.sciebo.de/s/BQRaiztJkmx33tt/download"

# Expected file counts
EXPECTED_LAYERS=22      # layer_0.onnx .. layer_21.onnx
EXPECTED_SUBEXITS=18    # subexit_0.onnx .. subexit_17.onnx
EXPECTED_COMBINERS=6    # combine_subheads_0.onnx .. combine_subheads_5.onnx
EXPECTED_IMAGES=191     # image_0.jpg .. image_190.jpg

errors=0

# ─────────────────────────────────────────────
# Helper: check if all weight files are present
# ─────────────────────────────────────────────
weights_complete() {
    [ -f "${WEIGHTS_DIR}/model.json" ] || return 1
    [ -f "${WEIGHTS_DIR}/nms.onnx" ] || return 1
    [ "$(ls "${WEIGHTS_DIR}"/layer_*.onnx 2>/dev/null | wc -l)" -eq "${EXPECTED_LAYERS}" ] || return 1
    [ "$(ls "${WEIGHTS_DIR}"/subexit_*.onnx 2>/dev/null | wc -l)" -eq "${EXPECTED_SUBEXITS}" ] || return 1
    [ "$(ls "${WEIGHTS_DIR}"/combine_subheads_*.onnx 2>/dev/null | wc -l)" -eq "${EXPECTED_COMBINERS}" ] || return 1
    return 0
}

# ─────────────────────────────────────────────
# Helper: check if all image files are present
# ─────────────────────────────────────────────
images_complete() {
    [ "$(ls "${IMAGES_DIR}"/image_*.jpg 2>/dev/null | wc -l)" -eq "${EXPECTED_IMAGES}" ] || return 1
    return 0
}

# ─────────────────────────────────────────────
# Weights: download if incomplete
# ─────────────────────────────────────────────
if ! weights_complete; then
    echo "YOLO weights incomplete or missing — downloading..."
    mkdir -p "${WEIGHTS_PARENT}"
    if wget -q --show-progress "${WEIGHTS_URL}" -O "${WEIGHTS_PARENT}/weights.zip"; then
        # Zip contains a weights_32/ folder — extract into parent
        unzip -o "${WEIGHTS_PARENT}/weights.zip" -d "${WEIGHTS_PARENT}"
        rm -f "${WEIGHTS_PARENT}/weights.zip"
    else
        echo "ERROR: Failed to download YOLO weights"
        rm -f "${WEIGHTS_PARENT}/weights.zip"
        errors=1
    fi

    # Verify after download
    if [ "${errors}" -eq 0 ] && ! weights_complete; then
        echo "ERROR: Downloaded weights archive is incomplete. Expected:"
        echo "  - model.json"
        echo "  - nms.onnx"
        echo "  - ${EXPECTED_LAYERS} layer ONNX files (layer_0.onnx .. layer_$((EXPECTED_LAYERS-1)).onnx)"
        echo "  - ${EXPECTED_SUBEXITS} subexit ONNX files (subexit_0.onnx .. subexit_$((EXPECTED_SUBEXITS-1)).onnx)"
        echo "  - ${EXPECTED_COMBINERS} combine_subheads ONNX files (combine_subheads_0.onnx .. combine_subheads_$((EXPECTED_COMBINERS-1)).onnx)"
        errors=1
    fi
fi

# ─────────────────────────────────────────────
# Images: download if incomplete
# ─────────────────────────────────────────────
if ! images_complete; then
    echo "Test images incomplete or missing — downloading..."
    mkdir -p "${IMAGES_PARENT}"
    if wget -q --show-progress "${IMAGES_URL}" -O "${IMAGES_PARENT}/images.zip"; then
        # Zip contains an images/ folder — extract into parent
        unzip -o "${IMAGES_PARENT}/images.zip" -d "${IMAGES_PARENT}"
        rm -f "${IMAGES_PARENT}/images.zip"
    else
        echo "ERROR: Failed to download test images"
        rm -f "${IMAGES_PARENT}/images.zip"
        errors=1
    fi

    # Verify after download
    if [ "${errors}" -eq 0 ] && ! images_complete; then
        image_count=$(ls "${IMAGES_DIR}"/image_*.jpg 2>/dev/null | wc -l)
        echo "ERROR: Downloaded images archive is incomplete. Expected ${EXPECTED_IMAGES} images, found ${image_count}"
        errors=1
    fi
fi

# ─────────────────────────────────────────────
# Summary
# ─────────────────────────────────────────────
if [ "${errors}" -ne 0 ]; then
    echo ""
    echo "See ARTIFACT_EVALUATION.md section 'YOLO Prerequisites' for details."
    exit 1
fi

image_count=$(ls "${IMAGES_DIR}"/image_*.jpg 2>/dev/null | wc -l)
echo "YOLO prerequisites OK (weights: ${EXPECTED_LAYERS} layers, ${EXPECTED_SUBEXITS} subexits, ${EXPECTED_COMBINERS} combiners, nms, model.json | images: ${image_count} files)"
