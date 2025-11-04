/// Base geometry types for nD geometry library
/// Provides Vector, Coordinate, and Point types with generic dimension and numeric types

use std::ops::{Add, Sub, Mul};

/// Generic nD vector with compile-time dimension D and numeric type T
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector<const D: usize, T> {
    pub data: [T; D],
}

/// Coordinate wrapper around Vector
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Coordinate<const D: usize, T>(pub Vector<D, T>);

/// Point type built on Coordinate
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point<const D: usize, T> {
    pub coord: Coordinate<D, T>,
}

// Vector implementations
impl<const D: usize, T> Vector<D, T> {
    pub fn new(data: [T; D]) -> Self {
        Vector { data }
    }
}

impl<const D: usize, T: Copy> Vector<D, T> {
    pub fn get(&self, i: usize) -> T {
        self.data[i]
    }

    pub fn set(&mut self, i: usize, val: T) {
        self.data[i] = val;
    }
}

// Coordinate implementations
impl<const D: usize, T> Coordinate<D, T> {
    pub fn new(data: [T; D]) -> Self {
        Coordinate(Vector::new(data))
    }

    pub fn from_slice(data: &[T]) -> Self
    where
        T: Copy + Default,
    {
        let mut arr = [T::default(); D];
        for i in 0..D.min(data.len()) {
            arr[i] = data[i];
        }
        Coordinate(Vector::new(arr))
    }
}

impl<const D: usize, T: Copy> Coordinate<D, T> {
    pub fn get(&self, i: usize) -> T {
        self.0.data[i]
    }

    pub fn set(&mut self, i: usize, val: T) {
        self.0.data[i] = val;
    }

    pub fn x(&self) -> T {
        self.0.data[0]
    }

    pub fn y(&self) -> T {
        self.0.data[1]
    }
}

impl<const D: usize, T: Copy + Default> Coordinate<D, T> {
    pub fn z(&self) -> T {
        if D >= 3 {
            self.0.data[2]
        } else {
            T::default()
        }
    }
}

// Point implementations
impl<const D: usize, T> Point<D, T> {
    pub fn new(data: [T; D]) -> Self {
        Point {
            coord: Coordinate::new(data),
        }
    }
}

impl<const D: usize, T: Copy> Point<D, T> {
    pub fn x(&self) -> T {
        self.coord.x()
    }

    pub fn y(&self) -> T {
        self.coord.y()
    }
}

impl<const D: usize, T: Copy + Default> Point<D, T> {
    pub fn z(&self) -> T {
        self.coord.z()
    }
}

// 2D specific types for convenience
pub type Coord2D = Coordinate<2, f64>;
pub type Point2D = Point<2, f64>;
pub type Coord2Di = Coordinate<2, i32>;
pub type Point2Di = Point<2, i32>;

pub fn new_coord2d(x: f64, y: f64) -> Coord2D {
    Coord2D::new([x, y])
}

pub fn new_point2d(x: f64, y: f64) -> Point2D {
    Point2D::new([x, y])
}

// Mathematical operations
impl<const D: usize, T: Copy + Sub<Output = T>> Sub for Coordinate<D, T> {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        let mut result = self.0.data;
        for i in 0..D {
            result[i] = result[i] - other.0.data[i];
        }
        Coordinate(Vector::new(result))
    }
}

impl<const D: usize, T: Copy + Add<Output = T>> Add for Coordinate<D, T> {
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        let mut result = self.0.data;
        for i in 0..D {
            result[i] = result[i] + other.0.data[i];
        }
        Coordinate(Vector::new(result))
    }
}

impl<const D: usize, T: Copy + Mul<Output = T>> Mul<T> for Coordinate<D, T> {
    type Output = Self;

    fn mul(self, scalar: T) -> Self::Output {
        let mut result = self.0.data;
        for i in 0..D {
            result[i] = result[i] * scalar;
        }
        Coordinate(Vector::new(result))
    }
}

impl<const D: usize, T: Copy + Mul<Output = T> + Add<Output = T> + Default> Coordinate<D, T> {
    pub fn dot(&self, other: &Self) -> T {
        let mut result = T::default();
        for i in 0..D {
            result = result + self.0.data[i] * other.0.data[i];
        }
        result
    }

    pub fn squared_length(&self) -> T {
        self.dot(self)
    }
}

impl Coord2D {
    pub fn length(&self) -> f64 {
        self.squared_length().sqrt()
    }

    /// 2D cross product (z-component of 3D cross product)
    pub fn cross2d(&self, other: &Self) -> f64 {
        self.x() * other.y() - self.y() * other.x()
    }

    /// Cross product of vectors (a-o) and (b-o)
    pub fn cross2d_with_origin(&self, a: &Coord2D, b: &Coord2D) -> f64 {
        (a.x() - self.x()) * (b.y() - self.y()) - (a.y() - self.y()) * (b.x() - self.x())
    }

    /// Orientation test: positive if ccw, negative if cw, zero if collinear
    pub fn orient2d(a: &Coord2D, b: &Coord2D, c: &Coord2D) -> f64 {
        a.cross2d_with_origin(b, c)
    }

    pub fn squared_distance(&self, other: &Self) -> f64 {
        (*self - *other).squared_length()
    }

    pub fn distance(&self, other: &Self) -> f64 {
        self.squared_distance(other).sqrt()
    }
}

// Comparison operators for Coord2D
impl PartialOrd for Coord2D {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for Coord2D {}

impl Ord for Coord2D {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Lexicographic ordering
        match self.x().partial_cmp(&other.x()) {
            Some(std::cmp::Ordering::Equal) => {
                self.y().partial_cmp(&other.y()).unwrap_or(std::cmp::Ordering::Equal)
            }
            Some(ord) => ord,
            None => std::cmp::Ordering::Equal,
        }
    }
}
