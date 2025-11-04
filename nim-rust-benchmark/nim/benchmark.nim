## Nim Geometry Benchmark Suite
## Benchmarks for all geometry algorithms

import geometry_types, delaunay, convex_hull, line_sweep, mesh, overlay
import std/[times, math, strformat]

type
  LCG = object
    state: uint64

proc initLCG(seed: uint64): LCG =
  LCG(state: seed)

proc next(rng: var LCG): uint64 =
  rng.state = rng.state * 1664525'u64 + 1013904223'u64
  result = rng.state

proc randomPoints2D(n: int, seed: uint64): seq[Coord2D] =
  var rng = initLCG(seed)
  result = newSeq[Coord2D](n)

  for i in 0..<n:
    let x = float64((rng.next() shr 16) and 0xFFFF) / 65535.0 * 100.0
    let y = float64((rng.next() shr 16) and 0xFFFF) / 65535.0 * 100.0
    result[i] = newCoord2D(x, y)

proc benchmarkDelaunay(points: seq[Coord2D]): tuple[triangles: int, time: float] =
  let start = cpuTime()
  let del = triangulate(points)
  let duration = cpuTime() - start

  result = (del.triangles.len div 3, duration * 1000.0)

proc benchmarkConvexHull(points: seq[Coord2D]): tuple[hullSize: int, time: float] =
  let start = cpuTime()
  let hull = convexHull(points)
  let duration = cpuTime() - start

  result = (hull.len, duration * 1000.0)

proc benchmarkConvexHullGraham(points: seq[Coord2D]): tuple[hullSize: int, time: float] =
  let start = cpuTime()
  let hull = convexHullGraham(points)
  let duration = cpuTime() - start

  result = (hull.len, duration * 1000.0)

proc benchmarkLineSweep(segments: seq[Segment]): tuple[intersections: int, time: float] =
  let start = cpuTime()
  let intersections = findIntersections(segments)
  let duration = cpuTime() - start

  result = (intersections.len, duration * 1000.0)

proc benchmarkLineSweepBrute(segments: seq[Segment]): tuple[intersections: int, time: float] =
  let start = cpuTime()
  let intersections = findIntersectionsBruteForce(segments)
  let duration = cpuTime() - start

  result = (intersections.len, duration * 1000.0)

proc benchmarkMeshGeneration(nPoints: int, seed: uint64): tuple[triangles: int, time: float] =
  let points = randomPoints2D(nPoints, seed)

  let start = cpuTime()
  let del = triangulate(points)
  let m = createMeshFromDelaunay(del)
  let duration = cpuTime() - start

  result = (m.triangles.len, duration * 1000.0)

proc benchmarkMeshRefinement(spacing: float): tuple[triangles: int, time: float] =
  var m = generateUniformMesh((0.0, 0.0, 10.0, 10.0), spacing)

  let start = cpuTime()
  m.refineMesh(0.6)
  let duration = cpuTime() - start

  result = (m.triangles.len, duration * 1000.0)

proc benchmarkPolygonIntersection(nSides: int): tuple[area: float, time: float] =
  # Create two regular polygons
  var p1Verts = newSeq[Coord2D](nSides)
  var p2Verts = newSeq[Coord2D](nSides)

  for i in 0..<nSides:
    let angle = 2.0 * PI * float(i) / float(nSides)
    p1Verts[i] = newCoord2D(cos(angle) * 5.0, sin(angle) * 5.0)
    p2Verts[i] = newCoord2D(cos(angle) * 4.0 + 1.0, sin(angle) * 4.0 + 1.0)

  let p1 = newPolygon(p1Verts)
  let p2 = newPolygon(p2Verts)

  let start = cpuTime()
  let results = polygonIntersection(p1, p2)
  let duration = cpuTime() - start

  let area = if results.len > 0: polygonArea(results[0]) else: 0.0

  result = (area, duration * 1000.0)

proc benchmarkPolygonUnion(nSides: int): tuple[area: float, time: float] =
  var p1Verts = newSeq[Coord2D](nSides)
  var p2Verts = newSeq[Coord2D](nSides)

  for i in 0..<nSides:
    let angle = 2.0 * PI * float(i) / float(nSides)
    p1Verts[i] = newCoord2D(cos(angle) * 5.0, sin(angle) * 5.0)
    p2Verts[i] = newCoord2D(cos(angle) * 4.0 + 1.0, sin(angle) * 4.0 + 1.0)

  let p1 = newPolygon(p1Verts)
  let p2 = newPolygon(p2Verts)

  let start = cpuTime()
  let results = polygonUnion(p1, p2)
  let duration = cpuTime() - start

  let area = if results.len > 0: polygonArea(results[0]) else: 0.0

  result = (area, duration * 1000.0)

proc main() =
  echo "=== Nim Geometry Benchmark ===\n"

  # Delaunay triangulation benchmark
  echo "--- Delaunay Triangulation ---"
  for n in [100, 500, 1000, 5000]:
    let points = randomPoints2D(n, 12345)
    let (triangles, time) = benchmarkDelaunay(points)
    echo fmt"  {n} points: {triangles} triangles in {time:.2f} ms"

  # Convex hull benchmark
  echo "\n--- Convex Hull (Andrew's Algorithm) ---"
  for n in [100, 500, 1000, 5000, 10000]:
    let points = randomPoints2D(n, 12345)
    let (hullSize, time) = benchmarkConvexHull(points)
    echo fmt"  {n} points: {hullSize} hull vertices in {time:.2f} ms"

  # Convex hull Graham scan benchmark
  echo "\n--- Convex Hull (Graham Scan) ---"
  for n in [100, 500, 1000, 5000, 10000]:
    let points = randomPoints2D(n, 12345)
    let (hullSize, time) = benchmarkConvexHullGraham(points)
    echo fmt"  {n} points: {hullSize} hull vertices in {time:.2f} ms"

  # Line sweep intersection benchmark
  echo "\n--- Line Sweep Intersection ---"
  for n in [50, 100, 200, 500]:
    let points = randomPoints2D(n * 2, 12345)
    var segments = newSeq[Segment](n)
    for i in 0..<n:
      segments[i] = newSegment(points[i * 2], points[i * 2 + 1], i)
    let (intersections, time) = benchmarkLineSweep(segments)
    echo fmt"  {n} segments: {intersections} intersections in {time:.2f} ms"

  # Line sweep brute force comparison
  echo "\n--- Line Sweep (Brute Force) ---"
  for n in [50, 100, 200]:
    let points = randomPoints2D(n * 2, 12345)
    var segments = newSeq[Segment](n)
    for i in 0..<n:
      segments[i] = newSegment(points[i * 2], points[i * 2 + 1], i)
    let (intersections, time) = benchmarkLineSweepBrute(segments)
    echo fmt"  {n} segments: {intersections} intersections in {time:.2f} ms"

  # Mesh generation benchmark
  echo "\n--- Mesh Generation ---"
  for n in [100, 500, 1000, 2000]:
    let (triangles, time) = benchmarkMeshGeneration(n, 12345)
    echo fmt"  {n} points: {triangles} triangles in {time:.2f} ms"

  # Mesh refinement benchmark
  echo "\n--- Mesh Refinement ---"
  for spacing in [2.0, 1.0, 0.5]:
    let (triangles, time) = benchmarkMeshRefinement(spacing)
    echo fmt"  spacing {spacing}: {triangles} triangles after refinement in {time:.2f} ms"

  # Polygon intersection benchmark
  echo "\n--- Polygon Intersection ---"
  for n in [4, 8, 16, 32, 64]:
    let (area, time) = benchmarkPolygonIntersection(n)
    echo fmt"  {n} sides: area {area:.2f} computed in {time:.2f} ms"

  # Polygon union benchmark
  echo "\n--- Polygon Union ---"
  for n in [4, 8, 16, 32, 64]:
    let (area, time) = benchmarkPolygonUnion(n)
    echo fmt"  {n} sides: area {area:.2f} computed in {time:.2f} ms"

  echo "\n=== Benchmark Complete ==="

when isMainModule:
  main()
