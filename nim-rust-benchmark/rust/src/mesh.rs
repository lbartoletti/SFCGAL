/// Mesh generation and manipulation
/// Triangle mesh operations including refinement and quality improvement

use crate::delaunay::{triangulate, Delaunator};
use crate::geometry_types::*;
use std::collections::HashSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Triangle {
    pub v0: usize,
    pub v1: usize,
    pub v2: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Edge {
    pub v0: usize,
    pub v1: usize,
}

impl Edge {
    pub fn new(v0: usize, v1: usize) -> Self {
        if v0 < v1 {
            Edge { v0, v1 }
        } else {
            Edge { v0: v1, v1: v0 }
        }
    }
}

#[derive(Debug, Clone)]
pub struct Mesh {
    pub vertices: Vec<Coord2D>,
    pub triangles: Vec<Triangle>,
    pub edges: Vec<Edge>,
}

fn triangle_area(a: &Coord2D, b: &Coord2D, c: &Coord2D) -> f64 {
    Coord2D::orient2d(a, b, c).abs() * 0.5
}

fn triangle_quality(a: &Coord2D, b: &Coord2D, c: &Coord2D) -> f64 {
    let area = triangle_area(a, b, c);
    let a_len = a.distance(b);
    let b_len = a.distance(c);
    let c_len = b.distance(c);

    let s = (a_len + b_len + c_len) * 0.5;
    if s < 1e-10 {
        return 0.0;
    }

    let inradius = area / s;
    let circumradius = (a_len * b_len * c_len) / (4.0 * area + 1e-10);

    if circumradius < 1e-10 {
        return 0.0;
    }

    2.0 * inradius / circumradius
}

pub fn create_mesh_from_delaunay(del: &Delaunator) -> Mesh {
    let vertices = del.coords.clone();

    let mut triangles = Vec::with_capacity(del.triangles.len() / 3);
    for i in 0..(del.triangles.len() / 3) {
        triangles.push(Triangle {
            v0: del.triangles[i * 3],
            v1: del.triangles[i * 3 + 1],
            v2: del.triangles[i * 3 + 2],
        });
    }

    let mut edge_set = HashSet::new();
    for tri in &triangles {
        edge_set.insert(Edge::new(tri.v0, tri.v1));
        edge_set.insert(Edge::new(tri.v1, tri.v2));
        edge_set.insert(Edge::new(tri.v2, tri.v0));
    }

    let edges = edge_set.into_iter().collect();

    Mesh {
        vertices,
        triangles,
        edges,
    }
}

pub fn generate_uniform_mesh(
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
    spacing: f64,
) -> Mesh {
    let nx = ((max_x - min_x) / spacing) as usize + 1;
    let ny = ((max_y - min_y) / spacing) as usize + 1;

    let mut points = Vec::new();

    for j in 0..ny {
        for i in 0..nx {
            let x = min_x + i as f64 * spacing;
            let y = min_y + j as f64 * spacing;
            points.push(new_coord2d(x, y));
        }
    }

    let del = triangulate(&points);
    create_mesh_from_delaunay(&del)
}

pub fn mesh_quality_stats(mesh: &Mesh) -> (f64, f64, f64) {
    if mesh.triangles.is_empty() {
        return (0.0, 0.0, 0.0);
    }

    let mut min_q: f64 = 1.0;
    let mut max_q: f64 = 0.0;
    let mut sum_q: f64 = 0.0;

    for tri in &mesh.triangles {
        let a = mesh.vertices[tri.v0];
        let b = mesh.vertices[tri.v1];
        let c = mesh.vertices[tri.v2];
        let q = triangle_quality(&a, &b, &c);

        min_q = min_q.min(q);
        max_q = max_q.max(q);
        sum_q += q;
    }

    (min_q, max_q, sum_q / mesh.triangles.len() as f64)
}

pub fn refine_mesh(mesh: &mut Mesh, min_quality: f64) {
    let mut to_refine = Vec::new();

    for (i, tri) in mesh.triangles.iter().enumerate() {
        let a = mesh.vertices[tri.v0];
        let b = mesh.vertices[tri.v1];
        let c = mesh.vertices[tri.v2];
        let q = triangle_quality(&a, &b, &c);

        if q < min_quality {
            to_refine.push(i);
        }
    }

    if to_refine.is_empty() {
        return;
    }

    let mut new_vertices = mesh.vertices.clone();

    for &tri_idx in &to_refine {
        let tri = mesh.triangles[tri_idx];
        let a = mesh.vertices[tri.v0];
        let b = mesh.vertices[tri.v1];
        let c = mesh.vertices[tri.v2];

        let cx = (a.x() + b.x() + c.x()) / 3.0;
        let cy = (a.y() + b.y() + c.y()) / 3.0;
        new_vertices.push(new_coord2d(cx, cy));
    }

    let del = triangulate(&new_vertices);
    *mesh = create_mesh_from_delaunay(&del);
}

pub fn smooth_mesh(mesh: &mut Mesh, iterations: usize) {
    for _ in 0..iterations {
        let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); mesh.vertices.len()];

        for tri in &mesh.triangles {
            adjacency[tri.v0].push(tri.v1);
            adjacency[tri.v0].push(tri.v2);
            adjacency[tri.v1].push(tri.v0);
            adjacency[tri.v1].push(tri.v2);
            adjacency[tri.v2].push(tri.v0);
            adjacency[tri.v2].push(tri.v1);
        }

        let mut new_vertices = mesh.vertices.clone();

        for i in 0..mesh.vertices.len() {
            if !adjacency[i].is_empty() {
                let mut sum_x = 0.0;
                let mut sum_y = 0.0;

                for &j in &adjacency[i] {
                    sum_x += mesh.vertices[j].x();
                    sum_y += mesh.vertices[j].y();
                }

                new_vertices[i] = new_coord2d(
                    sum_x / adjacency[i].len() as f64,
                    sum_y / adjacency[i].len() as f64,
                );
            }
        }

        mesh.vertices = new_vertices;
    }
}

pub fn mesh_bounding_box(mesh: &Mesh) -> (f64, f64, f64, f64) {
    if mesh.vertices.is_empty() {
        return (0.0, 0.0, 0.0, 0.0);
    }

    let mut min_x = mesh.vertices[0].x();
    let mut min_y = mesh.vertices[0].y();
    let mut max_x = mesh.vertices[0].x();
    let mut max_y = mesh.vertices[0].y();

    for v in &mesh.vertices {
        min_x = min_x.min(v.x());
        min_y = min_y.min(v.y());
        max_x = max_x.max(v.x());
        max_y = max_y.max(v.y());
    }

    (min_x, min_y, max_x, max_y)
}

pub fn mesh_total_area(mesh: &Mesh) -> f64 {
    let mut area = 0.0;
    for tri in &mesh.triangles {
        let a = mesh.vertices[tri.v0];
        let b = mesh.vertices[tri.v1];
        let c = mesh.vertices[tri.v2];
        area += triangle_area(&a, &b, &c);
    }
    area
}
