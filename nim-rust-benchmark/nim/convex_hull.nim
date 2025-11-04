## Convex Hull 2D implementation
## Using Andrew's monotone chain algorithm - O(n log n)

import geometry_types
import std/algorithm

proc convexHull*(points: seq[Coord2D]): seq[Coord2D] =
  ## Compute 2D convex hull using Andrew's monotone chain algorithm
  ## Returns vertices in counter-clockwise order

  if points.len < 3:
    return points

  # Sort points lexicographically (first by x, then by y)
  var sorted = points
  sorted.sort(proc(a, b: Coord2D): int =
    if a.x != b.x:
      if a.x < b.x: -1 else: 1
    else:
      if a.y < b.y: -1 else: 1
  )

  # Build lower hull
  var lower = newSeq[Coord2D]()
  for p in sorted:
    while lower.len >= 2 and orient2D(lower[^2], lower[^1], p) <= 0.0:
      discard lower.pop()
    lower.add(p)

  # Build upper hull
  var upper = newSeq[Coord2D]()
  for i in countdown(sorted.len - 1, 0):
    let p = sorted[i]
    while upper.len >= 2 and orient2D(upper[^2], upper[^1], p) <= 0.0:
      discard upper.pop()
    upper.add(p)

  # Remove last point of each half because it's repeated
  discard lower.pop()
  discard upper.pop()

  # Concatenate lower and upper hull
  result = lower & upper

proc convexHullGraham*(points: seq[Coord2D]): seq[Coord2D] =
  ## Compute 2D convex hull using Graham scan algorithm
  ## Returns vertices in counter-clockwise order

  if points.len < 3:
    return points

  # Find the bottom-most point (or left most in case of tie)
  var minIdx = 0
  var minY = points[0].y
  var minX = points[0].x

  for i in 1..<points.len:
    let y = points[i].y
    let x = points[i].x
    if y < minY or (y == minY and x < minX):
      minY = y
      minX = x
      minIdx = i

  let pivot = points[minIdx]

  # Sort points by polar angle with respect to pivot
  var sorted = newSeq[Coord2D]()
  for i, p in points:
    if i != minIdx:
      sorted.add(p)

  sorted.sort(proc(a, b: Coord2D): int =
    let orient = orient2D(pivot, a, b)
    if orient == 0.0:
      # Collinear - sort by distance
      let distA = squaredDistance(pivot, a)
      let distB = squaredDistance(pivot, b)
      if distA < distB: -1 elif distA > distB: 1 else: 0
    elif orient > 0.0:
      -1
    else:
      1
  )

  # Build convex hull
  result = @[pivot]

  for p in sorted:
    # Remove points that make clockwise turn
    while result.len >= 2 and orient2D(result[^2], result[^1], p) <= 0.0:
      discard result.pop()
    result.add(p)

proc convexHullArea*(hull: seq[Coord2D]): float64 =
  ## Calculate area of convex hull using shoelace formula
  if hull.len < 3:
    return 0.0

  result = 0.0
  for i in 0..<hull.len:
    let j = (i + 1) mod hull.len
    result += hull[i].x * hull[j].y
    result -= hull[j].x * hull[i].y

  result = abs(result) / 2.0

proc isPointInConvexHull*(hull: seq[Coord2D], p: Coord2D): bool =
  ## Test if point is inside convex hull
  ## Hull must be in counter-clockwise order
  if hull.len < 3:
    return false

  # Point must be on the left side of all edges
  for i in 0..<hull.len:
    let j = (i + 1) mod hull.len
    if orient2D(hull[i], hull[j], p) < 0.0:
      return false

  return true
