/// Polygon overlay operations (intersection, union, difference)
/// Using the Weiler-Atherton clipping algorithm

use crate::geometry_types::*;

#[derive(Debug, Clone, PartialEq)]
pub struct Polygon {
    pub vertices: Vec<Coord2D>,
}

impl Polygon {
    pub fn new(vertices: Vec<Coord2D>) -> Self {
        Polygon { vertices }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OverlayOp {
    Intersection,
    Union,
    Difference,
    SymmetricDifference,
}

pub fn polygon_area(poly: &Polygon) -> f64 {
    if poly.vertices.len() < 3 {
        return 0.0;
    }

    let mut area = 0.0;
    for i in 0..poly.vertices.len() {
        let j = (i + 1) % poly.vertices.len();
        area += poly.vertices[i].x() * poly.vertices[j].y();
        area -= poly.vertices[j].x() * poly.vertices[i].y();
    }

    area.abs() * 0.5
}

pub fn is_point_in_polygon(poly: &Polygon, p: &Coord2D) -> bool {
    if poly.vertices.len() < 3 {
        return false;
    }

    let mut inside = false;
    let n = poly.vertices.len();

    let mut j = n - 1;
    for i in 0..n {
        let vi = poly.vertices[i];
        let vj = poly.vertices[j];

        if ((vi.y() > p.y()) != (vj.y() > p.y()))
            && (p.x() < (vj.x() - vi.x()) * (p.y() - vi.y()) / (vj.y() - vi.y() + 1e-10) + vi.x())
        {
            inside = !inside;
        }

        j = i;
    }

    inside
}

fn line_segment_intersection(
    p1: &Coord2D,
    p2: &Coord2D,
    p3: &Coord2D,
    p4: &Coord2D,
) -> Option<(Coord2D, f64)> {
    let x1 = p1.x();
    let y1 = p1.y();
    let x2 = p2.x();
    let y2 = p2.y();
    let x3 = p3.x();
    let y3 = p3.y();
    let x4 = p4.x();
    let y4 = p4.y();

    let denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    if denom.abs() < 1e-10 {
        return None;
    }

    let t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    let u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

    if t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0 {
        let x = x1 + t * (x2 - x1);
        let y = y1 + t * (y2 - y1);
        Some((new_coord2d(x, y), t))
    } else {
        None
    }
}

fn sutherland_hodgman_clip(
    subject: &Polygon,
    clip_edge_start: &Coord2D,
    clip_edge_end: &Coord2D,
) -> Polygon {
    if subject.vertices.is_empty() {
        return Polygon::new(Vec::new());
    }

    let mut output = Vec::new();

    for i in 0..subject.vertices.len() {
        let curr = subject.vertices[i];
        let next = subject.vertices[(i + 1) % subject.vertices.len()];

        let curr_inside = Coord2D::orient2d(clip_edge_start, clip_edge_end, &curr) >= 0.0;
        let next_inside = Coord2D::orient2d(clip_edge_start, clip_edge_end, &next) >= 0.0;

        if next_inside {
            if !curr_inside {
                if let Some((point, _)) =
                    line_segment_intersection(&curr, &next, clip_edge_start, clip_edge_end)
                {
                    output.push(point);
                }
            }
            output.push(next);
        } else if curr_inside {
            if let Some((point, _)) =
                line_segment_intersection(&curr, &next, clip_edge_start, clip_edge_end)
            {
                output.push(point);
            }
        }
    }

    Polygon::new(output)
}

pub fn polygon_intersection_sh(subject: &Polygon, clip: &Polygon) -> Polygon {
    let mut result = subject.clone();

    for i in 0..clip.vertices.len() {
        let edge_start = clip.vertices[i];
        let edge_end = clip.vertices[(i + 1) % clip.vertices.len()];
        result = sutherland_hodgman_clip(&result, &edge_start, &edge_end);

        if result.vertices.is_empty() {
            break;
        }
    }

    result
}

pub fn polygon_intersection(p1: &Polygon, p2: &Polygon) -> Vec<Polygon> {
    let clipped = polygon_intersection_sh(p1, p2);
    if clipped.vertices.len() >= 3 {
        vec![clipped]
    } else {
        Vec::new()
    }
}

pub fn polygon_union(p1: &Polygon, p2: &Polygon) -> Vec<Polygon> {
    // Check if one polygon contains the other
    let mut p1_contains_p2 = true;
    for v in &p2.vertices {
        if !is_point_in_polygon(p1, v) {
            p1_contains_p2 = false;
            break;
        }
    }

    if p1_contains_p2 {
        return vec![p1.clone()];
    }

    let mut p2_contains_p1 = true;
    for v in &p1.vertices {
        if !is_point_in_polygon(p2, v) {
            p2_contains_p1 = false;
            break;
        }
    }

    if p2_contains_p1 {
        return vec![p2.clone()];
    }

    let mut vertices = Vec::new();

    // Collect vertices from p1 that are outside p2
    for v in &p1.vertices {
        if !is_point_in_polygon(p2, v) {
            vertices.push(*v);
        }
    }

    // Collect vertices from p2 that are outside p1
    for v in &p2.vertices {
        if !is_point_in_polygon(p1, v) {
            vertices.push(*v);
        }
    }

    // Find intersection points
    for i in 0..p1.vertices.len() {
        let p1_curr = p1.vertices[i];
        let p1_next = p1.vertices[(i + 1) % p1.vertices.len()];

        for j in 0..p2.vertices.len() {
            let p2_curr = p2.vertices[j];
            let p2_next = p2.vertices[(j + 1) % p2.vertices.len()];

            if let Some((point, _)) =
                line_segment_intersection(&p1_curr, &p1_next, &p2_curr, &p2_next)
            {
                vertices.push(point);
            }
        }
    }

    if vertices.len() < 3 {
        return vec![p1.clone(), p2.clone()];
    }

    // Sort vertices by angle from centroid
    let mut cx = 0.0;
    let mut cy = 0.0;
    for v in &vertices {
        cx += v.x();
        cy += v.y();
    }
    cx /= vertices.len() as f64;
    cy /= vertices.len() as f64;
    let center = new_coord2d(cx, cy);

    vertices.sort_by(|a, b| {
        let angle_a = (a.y() - center.y()).atan2(a.x() - center.x());
        let angle_b = (b.y() - center.y()).atan2(b.x() - center.x());
        angle_a.partial_cmp(&angle_b).unwrap()
    });

    vec![Polygon::new(vertices)]
}

pub fn polygon_difference(p1: &Polygon, p2: &Polygon) -> Vec<Polygon> {
    let mut result_vertices = Vec::new();

    // Collect vertices from p1 that are outside p2
    for v in &p1.vertices {
        if !is_point_in_polygon(p2, v) {
            result_vertices.push(*v);
        }
    }

    // Find intersection points
    for i in 0..p1.vertices.len() {
        let p1_curr = p1.vertices[i];
        let p1_next = p1.vertices[(i + 1) % p1.vertices.len()];

        for j in 0..p2.vertices.len() {
            let p2_curr = p2.vertices[j];
            let p2_next = p2.vertices[(j + 1) % p2.vertices.len()];

            if let Some((point, _)) =
                line_segment_intersection(&p1_curr, &p1_next, &p2_curr, &p2_next)
            {
                result_vertices.push(point);
            }
        }
    }

    if result_vertices.len() < 3 {
        return Vec::new();
    }

    vec![Polygon::new(result_vertices)]
}

pub fn bounding_boxes_overlap(p1: &Polygon, p2: &Polygon) -> bool {
    if p1.vertices.is_empty() || p2.vertices.is_empty() {
        return false;
    }

    let mut min_x1 = f64::INFINITY;
    let mut min_y1 = f64::INFINITY;
    let mut max_x1 = f64::NEG_INFINITY;
    let mut max_y1 = f64::NEG_INFINITY;

    for v in &p1.vertices {
        min_x1 = min_x1.min(v.x());
        min_y1 = min_y1.min(v.y());
        max_x1 = max_x1.max(v.x());
        max_y1 = max_y1.max(v.y());
    }

    let mut min_x2 = f64::INFINITY;
    let mut min_y2 = f64::INFINITY;
    let mut max_x2 = f64::NEG_INFINITY;
    let mut max_y2 = f64::NEG_INFINITY;

    for v in &p2.vertices {
        min_x2 = min_x2.min(v.x());
        min_y2 = min_y2.min(v.y());
        max_x2 = max_x2.max(v.x());
        max_y2 = max_y2.max(v.y());
    }

    !(max_x1 < min_x2 || max_x2 < min_x1 || max_y1 < min_y2 || max_y2 < min_y1)
}
