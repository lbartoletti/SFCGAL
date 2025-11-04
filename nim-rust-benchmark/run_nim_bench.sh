#!/bin/bash
# Run Nim benchmarks

cd "$(dirname "$0")/nim"

echo "Compiling Nim benchmark..."
nim c -r -d:release --opt:speed benchmark.nim

echo ""
echo "Benchmark complete!"
