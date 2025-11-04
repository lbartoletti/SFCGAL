#!/bin/bash
# Run both Nim and Rust benchmarks and compare

echo "======================================"
echo "  Nim vs Rust Geometry Benchmarks"
echo "======================================"
echo ""

echo "Running Rust benchmarks..."
echo "--------------------------------------"
./run_rust_bench.sh

echo ""
echo "======================================"
echo ""

echo "Running Nim benchmarks..."
echo "--------------------------------------"
./run_nim_bench.sh

echo ""
echo "======================================"
echo "All benchmarks complete!"
echo "======================================"
