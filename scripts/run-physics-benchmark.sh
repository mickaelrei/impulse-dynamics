#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build}"
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"

mkdir -p "$BUILD_DIR"
cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
cmake --build "$BUILD_DIR" --target physics-scenarios-bench -j"$(nproc)"

cd "$ROOT"
exec "$BUILD_DIR/physics-scenarios-bench" "$@"
