## Base geometry types for nD geometry library
## Provides Vector, Coordinate, and Point types with generic dimension and numeric types

import std/[math, sequtils, algorithm]

type
  Vector*[D: static[int], T] = object
    ## Generic nD vector with dimension D and numeric type T
    data*: array[D, T]

  Coordinate*[D: static[int], T] = distinct Vector[D, T]
    ## Coordinate wrapper around Vector

  Point*[D: static[int], T] = object
    ## Point type built on Coordinate
    coord*: Coordinate[D, T]

# Vector operations
proc `[]`*[D: static[int], T](v: Vector[D, T], i: int): T {.inline.} =
  v.data[i]

proc `[]=`*[D: static[int], T](v: var Vector[D, T], i: int, val: T) {.inline.} =
  v.data[i] = val

proc newVector*[D: static[int], T](data: array[D, T]): Vector[D, T] =
  result.data = data

proc newVector*[D: static[int], T](data: openArray[T]): Vector[D, T] =
  for i in 0..<D:
    result.data[i] = data[i]

# Coordinate operations
proc `[]`*[D: static[int], T](c: Coordinate[D, T], i: int): T {.inline.} =
  Vector[D, T](c).data[i]

proc `[]=`*[D: static[int], T](c: var Coordinate[D, T], i: int, val: T) {.inline.} =
  Vector[D, T](c).data[i] = val

proc newCoordinate*[D: static[int], T](data: array[D, T]): Coordinate[D, T] =
  Coordinate[D, T](newVector[D, T](data))

proc newCoordinate*[D: static[int], T](data: openArray[T]): Coordinate[D, T] =
  Coordinate[D, T](newVector[D, T](data))

proc x*[D: static[int], T](c: Coordinate[D, T]): T {.inline.} = c[0]
proc y*[D: static[int], T](c: Coordinate[D, T]): T {.inline.} = c[1]
proc z*[D: static[int], T](c: Coordinate[D, T]): T {.inline.} =
  when D >= 3: c[2] else: T(0)

# Point operations
proc newPoint*[D: static[int], T](data: array[D, T]): Point[D, T] =
  result.coord = newCoordinate[D, T](data)

proc newPoint*[D: static[int], T](data: openArray[T]): Point[D, T] =
  result.coord = newCoordinate[D, T](data)

proc x*[D: static[int], T](p: Point[D, T]): T {.inline.} = p.coord.x
proc y*[D: static[int], T](p: Point[D, T]): T {.inline.} = p.coord.y
proc z*[D: static[int], T](p: Point[D, T]): T {.inline.} = p.coord.z

# 2D specific types for convenience
type
  Coord2D* = Coordinate[2, float64]
  Point2D* = Point[2, float64]
  Coord2Di* = Coordinate[2, int]
  Point2Di* = Point[2, int]

proc newCoord2D*(x, y: float64): Coord2D =
  newCoordinate([x, y])

proc newPoint2D*(x, y: float64): Point2D =
  newPoint([x, y])

# Mathematical operations
proc `-`*[D: static[int], T](a, b: Coordinate[D, T]): Coordinate[D, T] =
  var v = Vector[D, T](a)
  let vb = Vector[D, T](b)
  for i in 0..<D:
    v.data[i] = v.data[i] - vb.data[i]
  Coordinate[D, T](v)

proc `+`*[D: static[int], T](a, b: Coordinate[D, T]): Coordinate[D, T] =
  var v = Vector[D, T](a)
  let vb = Vector[D, T](b)
  for i in 0..<D:
    v.data[i] = v.data[i] + vb.data[i]
  Coordinate[D, T](v)

proc `*`*[D: static[int], T](a: Coordinate[D, T], s: T): Coordinate[D, T] =
  var v = Vector[D, T](a)
  for i in 0..<D:
    v.data[i] = v.data[i] * s
  Coordinate[D, T](v)

proc dot*[D: static[int], T](a, b: Coordinate[D, T]): T =
  let va = Vector[D, T](a)
  let vb = Vector[D, T](b)
  for i in 0..<D:
    result += va.data[i] * vb.data[i]

proc squaredLength*[D: static[int], T](c: Coordinate[D, T]): T =
  let v = Vector[D, T](c)
  for i in 0..<D:
    result += v.data[i] * v.data[i]

proc length*[D: static[int]](c: Coordinate[D, float64]): float64 =
  sqrt(squaredLength(c))

# 2D specific geometric operations
proc cross2D*(a, b: Coord2D): float64 {.inline.} =
  ## 2D cross product (z-component of 3D cross product)
  a.x * b.y - a.y * b.x

proc cross2D*(o, a, b: Coord2D): float64 {.inline.} =
  ## Cross product of vectors (a-o) and (b-o)
  (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)

proc orient2D*(a, b, c: Coord2D): float64 {.inline.} =
  ## Orientation test: positive if ccw, negative if cw, zero if collinear
  cross2D(a, b, c)

proc squaredDistance*[D: static[int], T](a, b: Coordinate[D, T]): T =
  squaredLength(a - b)

proc distance*(a, b: Coord2D): float64 =
  sqrt(squaredDistance(a, b))

# Comparison operators
proc `==`*[D: static[int], T](a, b: Coordinate[D, T]): bool =
  let va = Vector[D, T](a)
  let vb = Vector[D, T](b)
  for i in 0..<D:
    if va.data[i] != vb.data[i]:
      return false
  true

proc `<`*(a, b: Coord2D): bool =
  ## Lexicographic ordering
  if a.x != b.x:
    a.x < b.x
  else:
    a.y < b.y
