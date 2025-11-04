#!/bin/bash
# Run Rust benchmarks

cd "$(dirname "$0")/rust"

echo "Compiling Rust benchmark..."
cargo run --release

echo ""
echo "Benchmark complete!"
