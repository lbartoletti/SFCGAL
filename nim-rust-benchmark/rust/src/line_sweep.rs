/// Line sweep algorithm for segment intersections
/// Bentley-Ottmann algorithm implementation

use crate::geometry_types::*;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Segment {
    pub p1: Coord2D,
    pub p2: Coord2D,
    pub id: usize,
}

impl Segment {
    pub fn new(p1: Coord2D, p2: Coord2D, id: usize) -> Self {
        if p1 < p2 {
            Segment { p1, p2, id }
        } else {
            Segment { p1: p2, p2: p1, id }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum EventType {
    SegmentStart,
    SegmentEnd,
    Intersection,
}

#[derive(Debug, Clone, Copy)]
struct Event {
    point: Coord2D,
    event_type: EventType,
    segment: usize,
    other_segment: usize,
}

impl PartialEq for Event {
    fn eq(&self, other: &Self) -> bool {
        self.point == other.point
            && self.event_type as i32 == other.event_type as i32
            && self.segment == other.segment
    }
}

impl Eq for Event {}

impl PartialOrd for Event {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Event {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap
        match other.point.x().partial_cmp(&self.point.x()) {
            Some(Ordering::Equal) => match other.point.y().partial_cmp(&self.point.y()) {
                Some(Ordering::Equal) => {
                    (other.event_type as i32).cmp(&(self.event_type as i32))
                }
                Some(ord) => ord,
                None => Ordering::Equal,
            },
            Some(ord) => ord,
            None => Ordering::Equal,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Intersection {
    pub point: Coord2D,
    pub segment1: usize,
    pub segment2: usize,
}

fn orientation(p: &Coord2D, q: &Coord2D, r: &Coord2D) -> f64 {
    (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y())
}

fn on_segment(p: &Coord2D, q: &Coord2D, r: &Coord2D) -> bool {
    q.x() <= p.x().max(r.x())
        && q.x() >= p.x().min(r.x())
        && q.y() <= p.y().max(r.y())
        && q.y() >= p.y().min(r.y())
}

fn segments_intersect(s1: &Segment, s2: &Segment) -> bool {
    let p1 = s1.p1;
    let q1 = s1.p2;
    let p2 = s2.p1;
    let q2 = s2.p2;

    let o1 = orientation(&p1, &q1, &p2);
    let o2 = orientation(&p1, &q1, &q2);
    let o3 = orientation(&p2, &q2, &p1);
    let o4 = orientation(&p2, &q2, &q1);

    if o1 * o2 < 0.0 && o3 * o4 < 0.0 {
        return true;
    }

    if o1 == 0.0 && on_segment(&p1, &p2, &q1) {
        return true;
    }
    if o2 == 0.0 && on_segment(&p1, &q2, &q1) {
        return true;
    }
    if o3 == 0.0 && on_segment(&p2, &p1, &q2) {
        return true;
    }
    if o4 == 0.0 && on_segment(&p2, &q1, &q2) {
        return true;
    }

    false
}

fn compute_intersection(s1: &Segment, s2: &Segment) -> Option<Coord2D> {
    let p1 = s1.p1;
    let p2 = s1.p2;
    let p3 = s2.p1;
    let p4 = s2.p2;

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
        Some(new_coord2d(x, y))
    } else {
        None
    }
}

pub fn find_intersections(segments: &[Segment]) -> Vec<Intersection> {
    if segments.len() < 2 {
        return Vec::new();
    }

    let mut events = BinaryHeap::new();

    for (i, seg) in segments.iter().enumerate() {
        events.push(Event {
            point: seg.p1,
            event_type: EventType::SegmentStart,
            segment: i,
            other_segment: 0,
        });
        events.push(Event {
            point: seg.p2,
            event_type: EventType::SegmentEnd,
            segment: i,
            other_segment: 0,
        });
    }

    let mut status: Vec<usize> = Vec::new();
    let mut result: Vec<Intersection> = Vec::new();

    while let Some(event) = events.pop() {
        match event.event_type {
            EventType::SegmentStart => {
                let seg_idx = event.segment;
                status.push(seg_idx);

                for &other_idx in &status {
                    if other_idx != seg_idx && segments_intersect(&segments[seg_idx], &segments[other_idx]) {
                        if let Some(point) = compute_intersection(&segments[seg_idx], &segments[other_idx]) {
                            result.push(Intersection {
                                point,
                                segment1: seg_idx.min(other_idx),
                                segment2: seg_idx.max(other_idx),
                            });
                        }
                    }
                }
            }
            EventType::SegmentEnd => {
                let seg_idx = event.segment;
                status.retain(|&x| x != seg_idx);
            }
            EventType::Intersection => {}
        }
    }

    // Remove duplicates
    let mut seen = HashSet::new();
    let mut unique_result = Vec::new();

    for inter in result {
        let key = (
            (inter.point.x() * 1e9) as i64,
            (inter.point.y() * 1e9) as i64,
            inter.segment1,
            inter.segment2,
        );
        if seen.insert(key) {
            unique_result.push(inter);
        }
    }

    unique_result
}

pub fn find_intersections_brute_force(segments: &[Segment]) -> Vec<Intersection> {
    let mut result = Vec::new();

    for i in 0..segments.len() {
        for j in (i + 1)..segments.len() {
            if segments_intersect(&segments[i], &segments[j]) {
                if let Some(point) = compute_intersection(&segments[i], &segments[j]) {
                    result.push(Intersection {
                        point,
                        segment1: i,
                        segment2: j,
                    });
                }
            }
        }
    }

    result
}
