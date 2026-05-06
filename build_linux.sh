#!/usr/bin/env bash
set -euo pipefail

# Build script for Linux (stripped latency server)
# Output binary: ./build/testcpp_latency

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_FILE="${ROOT_DIR}/testcpp/main_linux.cpp"
OUT_DIR="${ROOT_DIR}/build"
OUT_BIN="${OUT_DIR}/testcpp_latency"

if [[ ! -f "${SRC_FILE}" ]]; then
  echo "Source file not found: ${SRC_FILE}"
  exit 1
fi

mkdir -p "${OUT_DIR}"

echo "Building ${SRC_FILE} -> ${OUT_BIN}"
g++ "${SRC_FILE}" \
  -std=c++20 \
  -O3 \
  -DNDEBUG \
  -Wall -Wextra -Wpedantic \
  -pthread \
  -o "${OUT_BIN}"

echo "Build complete: ${OUT_BIN}"
echo "Run example:"
echo "  ${OUT_BIN} 172.20.10.2 1 200 0.6 1.0"
