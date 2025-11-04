/// Convex Hull 2D implementation
/// Using Andrew's monotone chain algorithm - O(n log n)

use crate::geometry_types::*;

pub fn convex_hull(points: &[Coord2D]) -> Vec<Coord2D> {
    if points.len() < 3 {
        return points.to_vec();
    }

    // Sort points lexicographically
    let mut sorted = points.to_vec();
    sorted.sort();

    // Build lower hull
    let mut lower = Vec::new();
    for p in &sorted {
        while lower.len() >= 2 {
            let len = lower.len();
            if Coord2D::orient2d(&lower[len - 2], &lower[len - 1], p) <= 0.0 {
                lower.pop();
            } else {
                break;
            }
        }
        lower.push(*p);
    }

    // Build upper hull
    let mut upper = Vec::new();
    for p in sorted.iter().rev() {
        while upper.len() >= 2 {
            let len = upper.len();
            if Coord2D::orient2d(&upper[len - 2], &upper[len - 1], p) <= 0.0 {
                upper.pop();
            } else {
                break;
            }
        }
        upper.push(*p);
    }

    // Remove last point of each half because it's repeated
    lower.pop();
    upper.pop();

    // Concatenate lower and upper hull
    lower.extend(upper);
    lower
}

pub fn convex_hull_graham(points: &[Coord2D]) -> Vec<Coord2D> {
    if points.len() < 3 {
        return points.to_vec();
    }

    // Find the bottom-most point (or left most in case of tie)
    let mut min_idx = 0;
    let mut min_y = points[0].y();
    let mut min_x = points[0].x();

    for i in 1..points.len() {
        let y = points[i].y();
        let x = points[i].x();
        if y < min_y || (y == min_y && x < min_x) {
            min_y = y;
            min_x = x;
            min_idx = i;
        }
    }

    let pivot = points[min_idx];

    // Sort points by polar angle with respect to pivot
    let mut sorted: Vec<Coord2D> = points
        .iter()
        .enumerate()
        .filter(|&(i, _)| i != min_idx)
        .map(|(_, &p)| p)
        .collect();

    sorted.sort_by(|a, b| {
        let orient = Coord2D::orient2d(&pivot, a, b);
        if orient == 0.0 {
            // Collinear - sort by distance
            let dist_a = pivot.squared_distance(a);
            let dist_b = pivot.squared_distance(b);
            dist_a.partial_cmp(&dist_b).unwrap()
        } else if orient > 0.0 {
            std::cmp::Ordering::Less
        } else {
            std::cmp::Ordering::Greater
        }
    });

    // Build convex hull
    let mut result = vec![pivot];

    for p in sorted {
        while result.len() >= 2 {
            let len = result.len();
            if Coord2D::orient2d(&result[len - 2], &result[len - 1], &p) <= 0.0 {
                result.pop();
            } else {
                break;
            }
        }
        result.push(p);
    }

    result
}

pub fn convex_hull_area(hull: &[Coord2D]) -> f64 {
    if hull.len() < 3 {
        return 0.0;
    }

    let mut area = 0.0;
    for i in 0..hull.len() {
        let j = (i + 1) % hull.len();
        area += hull[i].x() * hull[j].y();
        area -= hull[j].x() * hull[i].y();
    }

    area.abs() / 2.0
}

pub fn is_point_in_convex_hull(hull: &[Coord2D], p: &Coord2D) -> bool {
    if hull.len() < 3 {
        return false;
    }

    // Point must be on the left side of all edges
    for i in 0..hull.len() {
        let j = (i + 1) % hull.len();
        if Coord2D::orient2d(&hull[i], &hull[j], p) < 0.0 {
            return false;
        }
    }

    true
}
