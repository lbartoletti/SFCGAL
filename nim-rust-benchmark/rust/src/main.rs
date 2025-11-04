use geometry_bench::convex_hull::*;
use geometry_bench::delaunay::*;
use geometry_bench::geometry_types::*;
use geometry_bench::line_sweep::*;
use geometry_bench::mesh::*;
use geometry_bench::overlay::*;
use std::time::Instant;

fn random_points_2d(n: usize, seed: u64) -> Vec<Coord2D> {
    // Simple LCG random generator for reproducibility
    let mut rng = seed;
    let mut points = Vec::with_capacity(n);

    for _ in 0..n {
        rng = rng.wrapping_mul(1664525).wrapping_add(1013904223);
        let x = ((rng >> 16) & 0xFFFF) as f64 / 65535.0 * 100.0;

        rng = rng.wrapping_mul(1664525).wrapping_add(1013904223);
        let y = ((rng >> 16) & 0xFFFF) as f64 / 65535.0 * 100.0;

        points.push(new_coord2d(x, y));
    }

    points
}

fn benchmark_delaunay(points: &[Coord2D]) -> (usize, f64) {
    let start = Instant::now();
    let del = triangulate(points);
    let duration = start.elapsed();

    (del.triangles.len() / 3, duration.as_secs_f64() * 1000.0)
}

fn benchmark_convex_hull(points: &[Coord2D]) -> (usize, f64) {
    let start = Instant::now();
    let hull = convex_hull(points);
    let duration = start.elapsed();

    (hull.len(), duration.as_secs_f64() * 1000.0)
}

fn benchmark_convex_hull_graham(points: &[Coord2D]) -> (usize, f64) {
    let start = Instant::now();
    let hull = convex_hull_graham(points);
    let duration = start.elapsed();

    (hull.len(), duration.as_secs_f64() * 1000.0)
}

fn benchmark_line_sweep(segments: &[Segment]) -> (usize, f64) {
    let start = Instant::now();
    let intersections = find_intersections(segments);
    let duration = start.elapsed();

    (intersections.len(), duration.as_secs_f64() * 1000.0)
}

fn benchmark_line_sweep_brute(segments: &[Segment]) -> (usize, f64) {
    let start = Instant::now();
    let intersections = find_intersections_brute_force(segments);
    let duration = start.elapsed();

    (intersections.len(), duration.as_secs_f64() * 1000.0)
}

fn benchmark_mesh_generation(n_points: usize, seed: u64) -> (usize, f64) {
    let points = random_points_2d(n_points, seed);

    let start = Instant::now();
    let del = triangulate(&points);
    let mesh = create_mesh_from_delaunay(&del);
    let duration = start.elapsed();

    (mesh.triangles.len(), duration.as_secs_f64() * 1000.0)
}

fn benchmark_mesh_refinement(spacing: f64) -> (usize, f64) {
    let mut mesh = generate_uniform_mesh(0.0, 0.0, 10.0, 10.0, spacing);

    let start = Instant::now();
    refine_mesh(&mut mesh, 0.6);
    let duration = start.elapsed();

    (mesh.triangles.len(), duration.as_secs_f64() * 1000.0)
}

fn benchmark_polygon_intersection(n_sides: usize) -> (f64, f64) {
    // Create two regular polygons
    let mut p1_verts = Vec::new();
    let mut p2_verts = Vec::new();

    for i in 0..n_sides {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / (n_sides as f64);
        p1_verts.push(new_coord2d(angle.cos() * 5.0, angle.sin() * 5.0));
        p2_verts.push(new_coord2d(
            angle.cos() * 4.0 + 1.0,
            angle.sin() * 4.0 + 1.0,
        ));
    }

    let p1 = Polygon::new(p1_verts);
    let p2 = Polygon::new(p2_verts);

    let start = Instant::now();
    let result = polygon_intersection(&p1, &p2);
    let duration = start.elapsed();

    let area = if !result.is_empty() {
        polygon_area(&result[0])
    } else {
        0.0
    };

    (area, duration.as_secs_f64() * 1000.0)
}

fn benchmark_polygon_union(n_sides: usize) -> (f64, f64) {
    let mut p1_verts = Vec::new();
    let mut p2_verts = Vec::new();

    for i in 0..n_sides {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / (n_sides as f64);
        p1_verts.push(new_coord2d(angle.cos() * 5.0, angle.sin() * 5.0));
        p2_verts.push(new_coord2d(
            angle.cos() * 4.0 + 1.0,
            angle.sin() * 4.0 + 1.0,
        ));
    }

    let p1 = Polygon::new(p1_verts);
    let p2 = Polygon::new(p2_verts);

    let start = Instant::now();
    let result = polygon_union(&p1, &p2);
    let duration = start.elapsed();

    let area = if !result.is_empty() {
        polygon_area(&result[0])
    } else {
        0.0
    };

    (area, duration.as_secs_f64() * 1000.0)
}

fn main() {
    println!("=== Rust Geometry Benchmark ===\n");

    // Delaunay triangulation benchmark
    println!("--- Delaunay Triangulation ---");
    for &n in &[100, 500, 1000, 5000] {
        let points = random_points_2d(n, 12345);
        let (triangles, time) = benchmark_delaunay(&points);
        println!(
            "  {} points: {} triangles in {:.2} ms",
            n, triangles, time
        );
    }

    // Convex hull benchmark
    println!("\n--- Convex Hull (Andrew's Algorithm) ---");
    for &n in &[100, 500, 1000, 5000, 10000] {
        let points = random_points_2d(n, 12345);
        let (hull_size, time) = benchmark_convex_hull(&points);
        println!("  {} points: {} hull vertices in {:.2} ms", n, hull_size, time);
    }

    // Convex hull Graham scan benchmark
    println!("\n--- Convex Hull (Graham Scan) ---");
    for &n in &[100, 500, 1000, 5000, 10000] {
        let points = random_points_2d(n, 12345);
        let (hull_size, time) = benchmark_convex_hull_graham(&points);
        println!("  {} points: {} hull vertices in {:.2} ms", n, hull_size, time);
    }

    // Line sweep intersection benchmark
    println!("\n--- Line Sweep Intersection ---");
    for &n in &[50, 100, 200, 500] {
        let points = random_points_2d(n * 2, 12345);
        let segments: Vec<_> = (0..n)
            .map(|i| Segment::new(points[i * 2], points[i * 2 + 1], i))
            .collect();
        let (intersections, time) = benchmark_line_sweep(&segments);
        println!(
            "  {} segments: {} intersections in {:.2} ms",
            n, intersections, time
        );
    }

    // Line sweep brute force comparison
    println!("\n--- Line Sweep (Brute Force) ---");
    for &n in &[50, 100, 200] {
        let points = random_points_2d(n * 2, 12345);
        let segments: Vec<_> = (0..n)
            .map(|i| Segment::new(points[i * 2], points[i * 2 + 1], i))
            .collect();
        let (intersections, time) = benchmark_line_sweep_brute(&segments);
        println!(
            "  {} segments: {} intersections in {:.2} ms",
            n, intersections, time
        );
    }

    // Mesh generation benchmark
    println!("\n--- Mesh Generation ---");
    for &n in &[100, 500, 1000, 2000] {
        let (triangles, time) = benchmark_mesh_generation(n, 12345);
        println!("  {} points: {} triangles in {:.2} ms", n, triangles, time);
    }

    // Mesh refinement benchmark
    println!("\n--- Mesh Refinement ---");
    for &spacing in &[2.0, 1.0, 0.5] {
        let (triangles, time) = benchmark_mesh_refinement(spacing);
        println!(
            "  spacing {}: {} triangles after refinement in {:.2} ms",
            spacing, triangles, time
        );
    }

    // Polygon intersection benchmark
    println!("\n--- Polygon Intersection ---");
    for &n in &[4, 8, 16, 32, 64] {
        let (area, time) = benchmark_polygon_intersection(n);
        println!(
            "  {} sides: area {:.2} computed in {:.2} ms",
            n, area, time
        );
    }

    // Polygon union benchmark
    println!("\n--- Polygon Union ---");
    for &n in &[4, 8, 16, 32, 64] {
        let (area, time) = benchmark_polygon_union(n);
        println!(
            "  {} sides: area {:.2} computed in {:.2} ms",
            n, area, time
        );
    }

    println!("\n=== Benchmark Complete ===");
}
