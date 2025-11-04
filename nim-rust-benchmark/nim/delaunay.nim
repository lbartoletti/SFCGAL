## Delaunay triangulation implementation
## Based on the Delaunator algorithm (https://github.com/mapbox/delaunator)
## Fast 2D Delaunay triangulation using sweep-hull algorithm

import geometry_types
import std/[math, algorithm, sequtils]

type
  Delaunator* = object
    ## Delaunay triangulation result
    coords*: seq[Coord2D]          ## Input coordinates
    triangles*: seq[int]           ## Triangle vertex indices (flat array, each 3 = 1 triangle)
    halfedges*: seq[int]           ## Halfedge adjacency (triangles[i] adjacent to halfedges[i])
    hull*: seq[int]                ## Indices of hull points in counter-clockwise order

const INVALID_INDEX = -1

proc circumradius(a, b, c: Coord2D): float64 =
  ## Calculate circumradius of triangle abc
  let dx = b.x - a.x
  let dy = b.y - a.y
  let ex = c.x - a.x
  let ey = c.y - a.y

  let bl = dx * dx + dy * dy
  let cl = ex * ex + ey * ey
  let d = 0.5 / (dx * ey - dy * ex)

  let x = (ey * bl - dy * cl) * d
  let y = (dx * cl - ex * bl) * d

  sqrt(x * x + y * y)

proc circumcenter(a, b, c: Coord2D): Coord2D =
  ## Calculate circumcenter of triangle abc
  let dx = b.x - a.x
  let dy = b.y - a.y
  let ex = c.x - a.x
  let ey = c.y - a.y

  let bl = dx * dx + dy * dy
  let cl = ex * ex + ey * ey
  let d = 0.5 / (dx * ey - dy * ex)

  let x = a.x + (ey * bl - dy * cl) * d
  let y = a.y + (dx * cl - ex * bl) * d

  newCoord2D(x, y)

proc inCircle(a, b, c, p: Coord2D): bool =
  ## Check if point p is inside circumcircle of triangle abc
  let dx = a.x - p.x
  let dy = a.y - p.y
  let ex = b.x - p.x
  let ey = b.y - p.y
  let fx = c.x - p.x
  let fy = c.y - p.y

  let ap = dx * dx + dy * dy
  let bp = ex * ex + ey * ey
  let cp = fx * fx + fy * fy

  return dx * (ey * cp - bp * fy) -
         dy * (ex * cp - bp * fx) +
         ap * (ex * fy - ey * fx) < 0.0

proc hashKey(p: Coord2D, center: Coord2D, hashSize: int): int =
  let angle = arctan2(p.y - center.y, p.x - center.x)
  let normalizedAngle = (angle + PI) / (2.0 * PI)
  int(floor(normalizedAngle * float64(hashSize))) mod hashSize

proc addTriangle(del: var Delaunator, i0, i1, i2: int, a, b, c: int): int =
  ## Add a triangle and return its index
  let t = del.triangles.len
  del.triangles.add(i0)
  del.triangles.add(i1)
  del.triangles.add(i2)
  del.halfedges.add(a)
  del.halfedges.add(b)
  del.halfedges.add(c)

  if a != INVALID_INDEX: del.halfedges[a] = t
  if b != INVALID_INDEX: del.halfedges[b] = t + 1
  if c != INVALID_INDEX: del.halfedges[c] = t + 2

  return t

proc link(del: var Delaunator, a, b: int) =
  del.halfedges[a] = b
  if b != INVALID_INDEX:
    del.halfedges[b] = a

proc legalize(del: var Delaunator, a: int, hullStart: var int, hull: var seq[int]): int =
  ## Legalize edge by flipping if needed
  var i = 0
  var ar = 0
  var a = a

  while true:
    let b = del.halfedges[a]

    let a0 = a - a mod 3
    ar = a0 + (a + 2) mod 3

    if b == INVALID_INDEX:
      if i == 0: break
      i -= 1
      a = hull[i]
      continue

    let b0 = b - b mod 3
    let al = a0 + (a + 1) mod 3
    let bl = b0 + (b + 2) mod 3

    let p0 = del.triangles[ar]
    let pr = del.triangles[a]
    let pl = del.triangles[al]
    let p1 = del.triangles[bl]

    let illegal = inCircle(
      del.coords[p0], del.coords[pr], del.coords[pl], del.coords[p1]
    )

    if illegal:
      del.triangles[a] = p1
      del.triangles[b] = p0

      let hbl = del.halfedges[bl]

      if hbl == INVALID_INDEX:
        var e = hullStart
        while true:
          if hull[e] == bl:
            hull[e] = a
            break
          e = (e + 1) mod hull.len
          if e == hullStart: break

      del.link(a, hbl)
      del.link(b, del.halfedges[ar])
      del.link(ar, bl)

      let br = b0 + (b + 1) mod 3

      if i < hull.len:
        hull[i] = br
        i += 1
      else:
        hull.add(br)
        i += 1
    else:
      if i == 0: break
      i -= 1
      a = hull[i]

  return ar

proc triangulate*(points: seq[Coord2D]): Delaunator =
  ## Compute Delaunay triangulation of input points
  let n = points.len

  if n < 3:
    return Delaunator()

  result.coords = points

  # Find bounding box center
  var minX = Inf
  var minY = Inf
  var maxX = -Inf
  var maxY = -Inf

  for p in points:
    minX = min(minX, p.x)
    minY = min(minY, p.y)
    maxX = max(maxX, p.x)
    maxY = max(maxY, p.y)

  let cx = (minX + maxX) / 2.0
  let cy = (minY + maxY) / 2.0
  let center = newCoord2D(cx, cy)

  # Find seed triangle (closest point to center + 2 more points)
  var i0, i1, i2 = 0
  var minDist = Inf

  for i in 0..<n:
    let d = squaredDistance(points[i], center)
    if d < minDist:
      i0 = i
      minDist = d

  let p0 = points[i0]

  minDist = Inf
  for i in 0..<n:
    if i == i0: continue
    let d = squaredDistance(points[i], p0)
    if d < minDist and d > 0.0:
      i1 = i
      minDist = d

  let p1 = points[i1]

  var minRadius = Inf
  for i in 0..<n:
    if i == i0 or i == i1: continue
    let r = circumradius(p0, p1, points[i])
    if r < minRadius:
      i2 = i
      minRadius = r

  var p2 = points[i2]

  if minRadius == Inf:
    # Collinear points - return empty triangulation
    return result

  # Orient triangle counter-clockwise
  if orient2D(p0, p1, p2) < 0.0:
    swap(i1, i2)
    swap(p1, p2)

  # Calculate circumcenter of seed triangle
  let circumCtr = circumcenter(p0, p1, p2)

  # Sort points by distance from circumcenter
  var dists = newSeq[float64](n)
  for i in 0..<n:
    dists[i] = squaredDistance(points[i], circumCtr)

  # Create index array and sort by distance
  var ids = newSeq[int](n)
  for i in 0..<n:
    ids[i] = i

  ids.sort(proc(i, j: int): int = cmp(dists[i], dists[j]))

  # Initialize hash for spatial indexing
  let hashSize = int(ceil(sqrt(float64(n))))
  var hullHash = newSeq[int](hashSize)
  for i in 0..<hashSize:
    hullHash[i] = INVALID_INDEX

  # Initialize hull
  var hullStart = i0
  var hullSize = 3
  var hullNext = newSeq[int](n)
  var hullPrev = newSeq[int](n)
  var hullTri = newSeq[int](n)

  hullNext[i0] = i1
  hullPrev[i2] = i1
  hullNext[i1] = i2
  hullPrev[i0] = i2
  hullNext[i2] = i0
  hullPrev[i1] = i0

  hullTri[i0] = 0
  hullTri[i1] = 1
  hullTri[i2] = 2

  hullHash[hashKey(p0, center, hashSize)] = i0
  hullHash[hashKey(p1, center, hashSize)] = i1
  hullHash[hashKey(p2, center, hashSize)] = i2

  # Allocate space for triangles
  let maxTriangles = max(2 * n - 5, 0)
  result.triangles = newSeqOfCap[int](maxTriangles * 3)
  result.halfedges = newSeqOfCap[int](maxTriangles * 3)

  discard result.addTriangle(i0, i1, i2, INVALID_INDEX, INVALID_INDEX, INVALID_INDEX)

  var xp, yp = NaN
  var tmpHull = newSeq[int](n)

  # Add remaining points
  for k in 0..<n:
    let i = ids[k]
    let p = points[i]

    if i == i0 or i == i1 or i == i2:
      continue

    # Skip duplicate points
    if p.x == xp and p.y == yp:
      continue
    xp = p.x
    yp = p.y

    # Find visible edge on convex hull
    var start = 0
    let key = hashKey(p, center, hashSize)
    for j in 0..<hashSize:
      start = hullHash[(key + j) mod hashSize]
      if start != INVALID_INDEX and start != hullNext[start]:
        break

    start = hullPrev[start]
    var e = start
    var q = hullNext[e]

    while orient2D(p, points[e], points[q]) >= 0.0:
      e = q
      if e == start:
        e = INVALID_INDEX
        break
      q = hullNext[e]

    if e == INVALID_INDEX:
      continue  # Point is inside the hull

    # Add triangle
    var t = result.addTriangle(e, i, hullNext[e], INVALID_INDEX, INVALID_INDEX, hullTri[e])
    hullTri[i] = result.legalize(t + 2, hullStart, tmpHull)
    hullTri[e] = t
    hullSize += 1

    # Walk forward to remove visible triangles
    var next = hullNext[e]
    q = hullNext[next]
    while orient2D(p, points[next], points[q]) < 0.0:
      t = result.addTriangle(next, i, q, hullTri[i], INVALID_INDEX, hullTri[next])
      hullTri[i] = result.legalize(t + 2, hullStart, tmpHull)
      hullNext[next] = next  # Mark as removed
      hullSize -= 1
      next = q
      q = hullNext[next]

    # Walk backward
    if e == start:
      q = hullPrev[e]
      while orient2D(p, points[q], points[e]) < 0.0:
        t = result.addTriangle(q, i, e, INVALID_INDEX, hullTri[e], hullTri[q])
        discard result.legalize(t + 2, hullStart, tmpHull)
        hullTri[q] = t
        hullNext[q] = q  # Mark as removed
        hullSize -= 1
        e = q
        q = hullPrev[e]

    # Update hull
    hullStart = e
    hullPrev[i] = e
    hullNext[e] = i
    hullPrev[next] = i
    hullNext[i] = next

    # Update hash
    hullHash[hashKey(p, center, hashSize)] = i

  # Extract hull
  result.hull = newSeq[int]()
  var e = hullStart
  while true:
    result.hull.add(e)
    e = hullNext[e]
    if e == hullStart:
      break

proc getTriangles*(del: Delaunator): seq[array[3, Coord2D]] =
  ## Extract triangles as coordinate arrays
  result = newSeq[array[3, Coord2D]](del.triangles.len div 3)
  for i in 0..<result.len:
    result[i] = [
      del.coords[del.triangles[i * 3]],
      del.coords[del.triangles[i * 3 + 1]],
      del.coords[del.triangles[i * 3 + 2]]
    ]
