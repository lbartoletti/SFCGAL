/// Delaunay triangulation implementation
/// Based on the Delaunator algorithm (https://github.com/mapbox/delaunator)
/// Fast 2D Delaunay triangulation using sweep-hull algorithm

use crate::geometry_types::*;
use std::f64::consts::PI;

const INVALID_INDEX: i32 = -1;

/// Delaunay triangulation result
#[derive(Debug, Clone)]
pub struct Delaunator {
    /// Input coordinates
    pub coords: Vec<Coord2D>,
    /// Triangle vertex indices (flat array, each 3 = 1 triangle)
    pub triangles: Vec<usize>,
    /// Halfedge adjacency (triangles[i] adjacent to halfedges[i])
    pub halfedges: Vec<i32>,
    /// Indices of hull points in counter-clockwise order
    pub hull: Vec<usize>,
}

fn circumradius(a: &Coord2D, b: &Coord2D, c: &Coord2D) -> f64 {
    let dx = b.x() - a.x();
    let dy = b.y() - a.y();
    let ex = c.x() - a.x();
    let ey = c.y() - a.y();

    let bl = dx * dx + dy * dy;
    let cl = ex * ex + ey * ey;
    let d = 0.5 / (dx * ey - dy * ex);

    let x = (ey * bl - dy * cl) * d;
    let y = (dx * cl - ex * bl) * d;

    (x * x + y * y).sqrt()
}

fn circumcenter(a: &Coord2D, b: &Coord2D, c: &Coord2D) -> Coord2D {
    let dx = b.x() - a.x();
    let dy = b.y() - a.y();
    let ex = c.x() - a.x();
    let ey = c.y() - a.y();

    let bl = dx * dx + dy * dy;
    let cl = ex * ex + ey * ey;
    let d = 0.5 / (dx * ey - dy * ex);

    let x = a.x() + (ey * bl - dy * cl) * d;
    let y = a.y() + (dx * cl - ex * bl) * d;

    new_coord2d(x, y)
}

fn in_circle(a: &Coord2D, b: &Coord2D, c: &Coord2D, p: &Coord2D) -> bool {
    let dx = a.x() - p.x();
    let dy = a.y() - p.y();
    let ex = b.x() - p.x();
    let ey = b.y() - p.y();
    let fx = c.x() - p.x();
    let fy = c.y() - p.y();

    let ap = dx * dx + dy * dy;
    let bp = ex * ex + ey * ey;
    let cp = fx * fx + fy * fy;

    dx * (ey * cp - bp * fy) - dy * (ex * cp - bp * fx) + ap * (ex * fy - ey * fx) < 0.0
}

fn hash_key(p: &Coord2D, center: &Coord2D, hash_size: usize) -> usize {
    let angle = (p.y() - center.y()).atan2(p.x() - center.x());
    let normalized_angle = (angle + PI) / (2.0 * PI);
    ((normalized_angle * hash_size as f64).floor() as usize) % hash_size
}

impl Delaunator {
    fn add_triangle(&mut self, i0: usize, i1: usize, i2: usize, a: i32, b: i32, c: i32) -> usize {
        let t = self.triangles.len();
        self.triangles.push(i0);
        self.triangles.push(i1);
        self.triangles.push(i2);
        self.halfedges.push(a);
        self.halfedges.push(b);
        self.halfedges.push(c);

        if a != INVALID_INDEX {
            self.halfedges[a as usize] = t as i32;
        }
        if b != INVALID_INDEX {
            self.halfedges[(b + 1) as usize] = (t + 1) as i32;
        }
        if c != INVALID_INDEX {
            self.halfedges[(c + 2) as usize] = (t + 2) as i32;
        }

        t
    }

    fn link(&mut self, a: usize, b: i32) {
        self.halfedges[a] = b;
        if b != INVALID_INDEX {
            self.halfedges[b as usize] = a as i32;
        }
    }

    fn legalize(&mut self, a: usize, hull_start: &mut usize, hull: &mut Vec<usize>) -> usize {
        let mut i = 0;
        let mut ar;
        let mut a = a;

        loop {
            let b = self.halfedges[a];

            let a0 = a - a % 3;
            ar = a0 + (a + 2) % 3;

            if b == INVALID_INDEX {
                if i == 0 {
                    break;
                }
                i -= 1;
                a = hull[i];
                continue;
            }

            let b0 = (b as usize) - (b as usize) % 3;
            let al = a0 + (a + 1) % 3;
            let bl = b0 + ((b as usize) + 2) % 3;

            let p0 = self.triangles[ar];
            let pr = self.triangles[a];
            let pl = self.triangles[al];
            let p1 = self.triangles[bl];

            let illegal = in_circle(&self.coords[p0], &self.coords[pr], &self.coords[pl], &self.coords[p1]);

            if illegal {
                self.triangles[a] = p1;
                self.triangles[b as usize] = p0;

                let hbl = self.halfedges[bl];

                if hbl == INVALID_INDEX {
                    let mut e = *hull_start;
                    loop {
                        if hull[e] == bl {
                            hull[e] = a;
                            break;
                        }
                        e = (e + 1) % hull.len();
                        if e == *hull_start {
                            break;
                        }
                    }
                }

                self.link(a, hbl);
                self.link(b as usize, self.halfedges[ar]);
                self.link(ar, bl as i32);

                let br = b0 + ((b as usize) + 1) % 3;

                if i < hull.len() {
                    hull[i] = br;
                } else {
                    hull.push(br);
                }
                i += 1;
            } else {
                if i == 0 {
                    break;
                }
                i -= 1;
                a = hull[i];
            }
        }

        ar
    }
}

pub fn triangulate(points: &[Coord2D]) -> Delaunator {
    let n = points.len();

    if n < 3 {
        return Delaunator {
            coords: points.to_vec(),
            triangles: Vec::new(),
            halfedges: Vec::new(),
            hull: Vec::new(),
        };
    }

    let coords = points.to_vec();

    // Find bounding box center
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;

    for p in &coords {
        min_x = min_x.min(p.x());
        min_y = min_y.min(p.y());
        max_x = max_x.max(p.x());
        max_y = max_y.max(p.y());
    }

    let cx = (min_x + max_x) / 2.0;
    let cy = (min_y + max_y) / 2.0;
    let center = new_coord2d(cx, cy);

    // Find seed triangle
    let mut i0 = 0;
    let mut min_dist = f64::INFINITY;

    for i in 0..n {
        let d = coords[i].squared_distance(&center);
        if d < min_dist {
            i0 = i;
            min_dist = d;
        }
    }

    let p0 = coords[i0];

    let mut i1 = 0;
    min_dist = f64::INFINITY;
    for i in 0..n {
        if i == i0 {
            continue;
        }
        let d = coords[i].squared_distance(&p0);
        if d < min_dist && d > 0.0 {
            i1 = i;
            min_dist = d;
        }
    }

    let p1 = coords[i1];

    let mut i2 = 0;
    let mut min_radius = f64::INFINITY;
    for i in 0..n {
        if i == i0 || i == i1 {
            continue;
        }
        let r = circumradius(&p0, &p1, &coords[i]);
        if r < min_radius {
            i2 = i;
            min_radius = r;
        }
    }

    let mut p2 = coords[i2];

    if min_radius == f64::INFINITY {
        return Delaunator {
            coords,
            triangles: Vec::new(),
            halfedges: Vec::new(),
            hull: Vec::new(),
        };
    }

    // Orient triangle counter-clockwise
    if Coord2D::orient2d(&p0, &p1, &p2) < 0.0 {
        std::mem::swap(&mut i1, &mut i2);
        p2 = coords[i2];
    }

    let circum_ctr = circumcenter(&p0, &p1, &p2);

    // Sort points by distance from circumcenter
    let dists: Vec<f64> = coords.iter().map(|p| p.squared_distance(&circum_ctr)).collect();

    let mut ids: Vec<usize> = (0..n).collect();
    ids.sort_by(|&i, &j| dists[i].partial_cmp(&dists[j]).unwrap());

    // Initialize hash
    let hash_size = (n as f64).sqrt().ceil() as usize;
    let mut hull_hash = vec![INVALID_INDEX; hash_size];

    // Initialize hull
    let hull_start = i0;
    let mut hull_next = vec![0; n];
    let mut hull_prev = vec![0; n];
    let mut hull_tri = vec![0; n];

    hull_next[i0] = i1;
    hull_prev[i2] = i1;
    hull_next[i1] = i2;
    hull_prev[i0] = i2;
    hull_next[i2] = i0;
    hull_prev[i1] = i0;

    hull_tri[i0] = 0;
    hull_tri[i1] = 1;
    hull_tri[i2] = 2;

    hull_hash[hash_key(&p0, &center, hash_size)] = i0 as i32;
    hull_hash[hash_key(&p1, &center, hash_size)] = i1 as i32;
    hull_hash[hash_key(&p2, &center, hash_size)] = i2 as i32;

    let max_triangles = 2 * n - 5;
    let mut result = Delaunator {
        coords,
        triangles: Vec::with_capacity(max_triangles * 3),
        halfedges: Vec::with_capacity(max_triangles * 3),
        hull: Vec::new(),
    };

    result.add_triangle(i0, i1, i2, INVALID_INDEX, INVALID_INDEX, INVALID_INDEX);

    let mut xp = f64::NAN;
    let mut yp = f64::NAN;
    let mut tmp_hull = vec![0; n];

    for k in 0..n {
        let i = ids[k];
        let p = result.coords[i];

        if i == i0 || i == i1 || i == i2 {
            continue;
        }

        if p.x() == xp && p.y() == yp {
            continue;
        }
        xp = p.x();
        yp = p.y();

        let mut start = 0;
        let key = hash_key(&p, &center, hash_size);
        for j in 0..hash_size {
            start = hull_hash[(key + j) % hash_size];
            if start != INVALID_INDEX && start != hull_next[start as usize] as i32 {
                break;
            }
        }

        start = hull_prev[start as usize] as i32;
        let mut e = start as usize;
        let mut q = hull_next[e];

        while Coord2D::orient2d(&p, &result.coords[e], &result.coords[q]) >= 0.0 {
            e = q;
            if e == start as usize {
                e = usize::MAX;
                break;
            }
            q = hull_next[e];
        }

        if e == usize::MAX {
            continue;
        }

        let t = result.add_triangle(e, i, hull_next[e], INVALID_INDEX, INVALID_INDEX, hull_tri[e] as i32);
        hull_tri[i] = result.legalize(t + 2, &mut (hull_start as usize), &mut tmp_hull);
        hull_tri[e] = t;

        let mut next = hull_next[e];
        q = hull_next[next];
        while Coord2D::orient2d(&p, &result.coords[next], &result.coords[q]) < 0.0 {
            let t = result.add_triangle(next, i, q, hull_tri[i] as i32, INVALID_INDEX, hull_tri[next] as i32);
            hull_tri[i] = result.legalize(t + 2, &mut (hull_start as usize), &mut tmp_hull);
            hull_next[next] = next;
            next = q;
            q = hull_next[next];
        }

        if e == start as usize {
            q = hull_prev[e];
            while Coord2D::orient2d(&p, &result.coords[q], &result.coords[e]) < 0.0 {
                let t = result.add_triangle(q, i, e, INVALID_INDEX, hull_tri[e] as i32, hull_tri[q] as i32);
                result.legalize(t + 2, &mut (hull_start as usize), &mut tmp_hull);
                hull_tri[q] = t;
                hull_next[q] = q;
                e = q;
                q = hull_prev[e];
            }
        }

        hull_prev[i] = e;
        hull_next[e] = i;
        hull_prev[next] = i;
        hull_next[i] = next;

        hull_hash[hash_key(&p, &center, hash_size)] = i as i32;
    }

    let mut e = hull_start;
    loop {
        result.hull.push(e);
        e = hull_next[e];
        if e == hull_start {
            break;
        }
    }

    result
}

pub fn get_triangles(del: &Delaunator) -> Vec<[Coord2D; 3]> {
    let mut result = Vec::with_capacity(del.triangles.len() / 3);
    for i in 0..(del.triangles.len() / 3) {
        result.push([
            del.coords[del.triangles[i * 3]],
            del.coords[del.triangles[i * 3 + 1]],
            del.coords[del.triangles[i * 3 + 2]],
        ]);
    }
    result
}
