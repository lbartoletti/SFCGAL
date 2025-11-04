## Polygon overlay operations (intersection, union, difference)
## Using the Weiler-Atherton clipping algorithm

import geometry_types
import std/[algorithm, sequtils, tables, sets]

type
  Polygon* = object
    ## Simple polygon defined by vertices in counter-clockwise order
    vertices*: seq[Coord2D]

  OverlayOp* = enum
    ## Overlay operation type
    OpIntersection
    OpUnion
    OpDifference
    OpSymmetricDifference

proc newPolygon*(vertices: seq[Coord2D]): Polygon =
  Polygon(vertices: vertices)

proc polygonArea*(poly: Polygon): float64 =
  ## Calculate polygon area using shoelace formula
  if poly.vertices.len < 3:
    return 0.0

  result = 0.0
  for i in 0..<poly.vertices.len:
    let j = (i + 1) mod poly.vertices.len
    result += poly.vertices[i].x * poly.vertices[j].y
    result -= poly.vertices[j].x * poly.vertices[i].y

  result = abs(result) * 0.5

proc isPointInPolygon*(poly: Polygon, p: Coord2D): bool =
  ## Test if point is inside polygon using ray casting
  if poly.vertices.len < 3:
    return false

  var inside = false
  let n = poly.vertices.len

  var j = n - 1
  for i in 0..<n:
    let vi = poly.vertices[i]
    let vj = poly.vertices[j]

    if ((vi.y > p.y) != (vj.y > p.y)) and
       (p.x < (vj.x - vi.x) * (p.y - vi.y) / (vj.y - vi.y + 1e-10) + vi.x):
      inside = not inside

    j = i

  return inside

proc lineSegmentIntersection(p1, p2, p3, p4: Coord2D): tuple[exists: bool, point: Coord2D, t: float64] =
  ## Find intersection of line segments p1-p2 and p3-p4
  let x1 = p1.x
  let y1 = p1.y
  let x2 = p2.x
  let y2 = p2.y
  let x3 = p3.x
  let y3 = p3.y
  let x4 = p4.x
  let y4 = p4.y

  let denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

  if abs(denom) < 1e-10:
    return (false, newCoord2D(0, 0), 0.0)

  let t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
  let u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

  if t >= 0.0 and t <= 1.0 and u >= 0.0 and u <= 1.0:
    let x = x1 + t * (x2 - x1)
    let y = y1 + t * (y2 - y1)
    return (true, newCoord2D(x, y), t)

  return (false, newCoord2D(0, 0), 0.0)

proc sutherlandHodgmanClip(subject: Polygon, clipEdgeStart, clipEdgeEnd: Coord2D): Polygon =
  ## Clip polygon against a single edge using Sutherland-Hodgman algorithm
  if subject.vertices.len == 0:
    return Polygon(vertices: @[])

  var output = newSeq[Coord2D]()

  for i in 0..<subject.vertices.len:
    let curr = subject.vertices[i]
    let next = subject.vertices[(i + 1) mod subject.vertices.len]

    # Determine if points are inside (left of) the clip edge
    let currInside = orient2D(clipEdgeStart, clipEdgeEnd, curr) >= 0.0
    let nextInside = orient2D(clipEdgeStart, clipEdgeEnd, next) >= 0.0

    if nextInside:
      if not currInside:
        # Entering - add intersection
        let inter = lineSegmentIntersection(curr, next, clipEdgeStart, clipEdgeEnd)
        if inter.exists:
          output.add(inter.point)
      output.add(next)
    elif currInside:
      # Leaving - add intersection
      let inter = lineSegmentIntersection(curr, next, clipEdgeStart, clipEdgeEnd)
      if inter.exists:
        output.add(inter.point)

  Polygon(vertices: output)

proc polygonIntersectionSH*(subject, clip: Polygon): Polygon =
  ## Compute intersection of two polygons using Sutherland-Hodgman algorithm
  ## Works for convex clip polygon
  result = subject

  for i in 0..<clip.vertices.len:
    let edgeStart = clip.vertices[i]
    let edgeEnd = clip.vertices[(i + 1) mod clip.vertices.len]
    result = sutherlandHodgmanClip(result, edgeStart, edgeEnd)

    if result.vertices.len == 0:
      break

proc polygonIntersection*(p1, p2: Polygon): seq[Polygon] =
  ## Compute intersection of two polygons (general case)
  ## Returns list of polygons (may be multiple for non-convex)

  # For simple implementation, use Sutherland-Hodgman
  # This works correctly only if clip polygon (p2) is convex
  let clipped = polygonIntersectionSH(p1, p2)
  if clipped.vertices.len >= 3:
    result = @[clipped]
  else:
    result = @[]

proc polygonUnion*(p1, p2: Polygon): seq[Polygon] =
  ## Compute union of two polygons
  ## Simplified implementation - works for simple cases

  # Check if one polygon contains the other
  var p1ContainsP2 = true
  for v in p2.vertices:
    if not isPointInPolygon(p1, v):
      p1ContainsP2 = false
      break

  if p1ContainsP2:
    return @[p1]

  var p2ContainsP1 = true
  for v in p1.vertices:
    if not isPointInPolygon(p2, v):
      p2ContainsP1 = false
      break

  if p2ContainsP1:
    return @[p2]

  # Find all intersection points
  var intersections = newSeq[Coord2D]()
  var vertices = newSeq[Coord2D]()

  # Collect vertices from p1 that are outside p2
  for v in p1.vertices:
    if not isPointInPolygon(p2, v):
      vertices.add(v)

  # Collect vertices from p2 that are outside p1
  for v in p2.vertices:
    if not isPointInPolygon(p1, v):
      vertices.add(v)

  # Find intersection points
  for i in 0..<p1.vertices.len:
    let p1_curr = p1.vertices[i]
    let p1_next = p1.vertices[(i + 1) mod p1.vertices.len]

    for j in 0..<p2.vertices.len:
      let p2_curr = p2.vertices[j]
      let p2_next = p2.vertices[(j + 1) mod p2.vertices.len]

      let inter = lineSegmentIntersection(p1_curr, p1_next, p2_curr, p2_next)
      if inter.exists:
        vertices.add(inter.point)

  if vertices.len < 3:
    # No valid union - return both polygons
    return @[p1, p2]

  # Sort vertices by angle from centroid to create convex hull
  var cx = 0.0
  var cy = 0.0
  for v in vertices:
    cx += v.x
    cy += v.y
  cx /= float64(vertices.len)
  cy /= float64(vertices.len)
  let center = newCoord2D(cx, cy)

  vertices.sort(proc(a, b: Coord2D): int =
    let angleA = arctan2(a.y - center.y, a.x - center.x)
    let angleB = arctan2(b.y - center.y, b.x - center.x)
    if angleA < angleB: -1 elif angleA > angleB: 1 else: 0
  )

  result = @[Polygon(vertices: vertices)]

proc polygonDifference*(p1, p2: Polygon): seq[Polygon] =
  ## Compute difference of two polygons (p1 - p2)
  ## Returns parts of p1 that are not in p2

  # Simplified implementation
  var resultVertices = newSeq[Coord2D]()

  # Collect vertices from p1 that are outside p2
  for v in p1.vertices:
    if not isPointInPolygon(p2, v):
      resultVertices.add(v)

  # Find intersection points and add them
  for i in 0..<p1.vertices.len:
    let p1_curr = p1.vertices[i]
    let p1_next = p1.vertices[(i + 1) mod p1.vertices.len]

    for j in 0..<p2.vertices.len:
      let p2_curr = p2.vertices[j]
      let p2_next = p2.vertices[(j + 1) mod p2.vertices.len]

      let inter = lineSegmentIntersection(p1_curr, p1_next, p2_curr, p2_next)
      if inter.exists:
        resultVertices.add(inter.point)

  if resultVertices.len < 3:
    # p1 is completely inside p2
    return @[]

  result = @[Polygon(vertices: resultVertices)]

proc boundingBoxesOverlap*(p1, p2: Polygon): bool =
  ## Quick check if bounding boxes of two polygons overlap
  if p1.vertices.len == 0 or p2.vertices.len == 0:
    return false

  var minX1 = Inf
  var minY1 = Inf
  var maxX1 = -Inf
  var maxY1 = -Inf

  for v in p1.vertices:
    minX1 = min(minX1, v.x)
    minY1 = min(minY1, v.y)
    maxX1 = max(maxX1, v.x)
    maxY1 = max(maxY1, v.y)

  var minX2 = Inf
  var minY2 = Inf
  var maxX2 = -Inf
  var maxY2 = -Inf

  for v in p2.vertices:
    minX2 = min(minX2, v.x)
    minY2 = min(minY2, v.y)
    maxX2 = max(maxX2, v.x)
    maxY2 = max(maxY2, v.y)

  not (maxX1 < minX2 or maxX2 < minX1 or maxY1 < minY2 or maxY2 < minY1)
