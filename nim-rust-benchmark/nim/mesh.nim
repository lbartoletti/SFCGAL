## Mesh generation and manipulation
## Triangle mesh operations including refinement and quality improvement

import geometry_types
import delaunay
import std/[math, algorithm, sets, tables]

type
  Triangle* = object
    ## Triangle defined by three vertex indices
    v0*, v1*, v2*: int

  Edge* = object
    ## Edge defined by two vertex indices (always sorted)
    v0*, v1*: int

  Mesh* = object
    ## Triangle mesh structure
    vertices*: seq[Coord2D]
    triangles*: seq[Triangle]
    edges*: seq[Edge]

proc newEdge*(v0, v1: int): Edge =
  ## Create edge with sorted vertices
  if v0 < v1:
    Edge(v0: v0, v1: v1)
  else:
    Edge(v0: v1, v1: v0)

proc `==`*(a, b: Edge): bool =
  a.v0 == b.v0 and a.v1 == b.v1

proc hash*(e: Edge): int =
  result = e.v0
  result = result xor (e.v1 shl 1)

proc triangleArea*(a, b, c: Coord2D): float64 =
  ## Calculate area of triangle using cross product
  abs(cross2D(a, b, c)) * 0.5

proc triangleQuality*(a, b, c: Coord2D): float64 =
  ## Calculate triangle quality metric (0 = degenerate, 1 = equilateral)
  ## Using ratio of inscribed circle radius to circumradius
  let area = triangleArea(a, b, c)
  let a_len = distance(b, c)
  let b_len = distance(a, c)
  let c_len = distance(a, b)

  let s = (a_len + b_len + c_len) * 0.5  # Semi-perimeter
  if s < 1e-10:
    return 0.0

  let inradius = area / s
  let circumradius = (a_len * b_len * c_len) / (4.0 * area + 1e-10)

  if circumradius < 1e-10:
    return 0.0

  # Quality ratio (normalized to 0-1, where 1 is equilateral)
  return 2.0 * inradius / circumradius

proc createMeshFromDelaunay*(del: Delaunator): Mesh =
  ## Create mesh from Delaunay triangulation
  result.vertices = del.coords

  # Build triangles
  result.triangles = newSeq[Triangle](del.triangles.len div 3)
  for i in 0..<result.triangles.len:
    result.triangles[i] = Triangle(
      v0: del.triangles[i * 3],
      v1: del.triangles[i * 3 + 1],
      v2: del.triangles[i * 3 + 2]
    )

  # Build edge list
  var edgeSet = initHashSet[Edge]()
  for tri in result.triangles:
    edgeSet.incl(newEdge(tri.v0, tri.v1))
    edgeSet.incl(newEdge(tri.v1, tri.v2))
    edgeSet.incl(newEdge(tri.v2, tri.v0))

  result.edges = toSeq(edgeSet.items)

proc generateUniformMesh*(bounds: tuple[minX, minY, maxX, maxY: float64], spacing: float64): Mesh =
  ## Generate a uniform triangular mesh within given bounds
  let nx = int((bounds.maxX - bounds.minX) / spacing) + 1
  let ny = int((bounds.maxY - bounds.minY) / spacing) + 1

  var points = newSeq[Coord2D]()

  for j in 0..<ny:
    for i in 0..<nx:
      let x = bounds.minX + float64(i) * spacing
      let y = bounds.minY + float64(j) * spacing
      points.add(newCoord2D(x, y))

  let del = triangulate(points)
  result = createMeshFromDelaunay(del)

proc meshQualityStats*(mesh: Mesh): tuple[minQ, maxQ, avgQ: float64] =
  ## Calculate quality statistics for mesh
  if mesh.triangles.len == 0:
    return (0.0, 0.0, 0.0)

  var minQ = 1.0
  var maxQ = 0.0
  var sumQ = 0.0

  for tri in mesh.triangles:
    let a = mesh.vertices[tri.v0]
    let b = mesh.vertices[tri.v1]
    let c = mesh.vertices[tri.v2]
    let q = triangleQuality(a, b, c)

    minQ = min(minQ, q)
    maxQ = max(maxQ, q)
    sumQ += q

  (minQ, maxQ, sumQ / float64(mesh.triangles.len))

proc refineMesh*(mesh: var Mesh, minQuality: float64) =
  ## Refine mesh by subdividing poor quality triangles
  ## Adds vertices at centroids of low-quality triangles
  var toRefine: seq[int] = @[]

  for i, tri in mesh.triangles:
    let a = mesh.vertices[tri.v0]
    let b = mesh.vertices[tri.v1]
    let c = mesh.vertices[tri.v2]
    let q = triangleQuality(a, b, c)

    if q < minQuality:
      toRefine.add(i)

  if toRefine.len == 0:
    return

  # Add centroids as new vertices
  var newVertices = mesh.vertices
  var vertexMap = initTable[int, int]()  # triangle index -> new vertex index

  for triIdx in toRefine:
    let tri = mesh.triangles[triIdx]
    let a = mesh.vertices[tri.v0]
    let b = mesh.vertices[tri.v1]
    let c = mesh.vertices[tri.v2]

    let cx = (a.x + b.x + c.x) / 3.0
    let cy = (a.y + b.y + c.y) / 3.0
    let centroid = newCoord2D(cx, cy)

    vertexMap[triIdx] = newVertices.len
    newVertices.add(centroid)

  # Retriangulate with new vertices
  let del = triangulate(newVertices)
  mesh = createMeshFromDelaunay(del)

proc smoothMesh*(mesh: var Mesh, iterations: int = 1) =
  ## Smooth mesh using Laplacian smoothing
  for iter in 0..<iterations:
    # Build adjacency information
    var adjacency = newSeq[seq[int]](mesh.vertices.len)

    for tri in mesh.triangles:
      adjacency[tri.v0].add(tri.v1)
      adjacency[tri.v0].add(tri.v2)
      adjacency[tri.v1].add(tri.v0)
      adjacency[tri.v1].add(tri.v2)
      adjacency[tri.v2].add(tri.v0)
      adjacency[tri.v2].add(tri.v1)

    # Compute new positions as average of neighbors
    var newVertices = mesh.vertices

    for i in 0..<mesh.vertices.len:
      if adjacency[i].len > 0:
        var sumX = 0.0
        var sumY = 0.0

        for j in adjacency[i]:
          sumX += mesh.vertices[j].x
          sumY += mesh.vertices[j].y

        newVertices[i] = newCoord2D(
          sumX / float64(adjacency[i].len),
          sumY / float64(adjacency[i].len)
        )

    mesh.vertices = newVertices

proc meshBoundingBox*(mesh: Mesh): tuple[minX, minY, maxX, maxY: float64] =
  ## Calculate bounding box of mesh
  if mesh.vertices.len == 0:
    return (0.0, 0.0, 0.0, 0.0)

  var minX = mesh.vertices[0].x
  var minY = mesh.vertices[0].y
  var maxX = mesh.vertices[0].x
  var maxY = mesh.vertices[0].y

  for v in mesh.vertices:
    minX = min(minX, v.x)
    minY = min(minY, v.y)
    maxX = max(maxX, v.x)
    maxY = max(maxY, v.y)

  (minX, minY, maxX, maxY)

proc meshTotalArea*(mesh: Mesh): float64 =
  ## Calculate total area of mesh
  result = 0.0
  for tri in mesh.triangles:
    let a = mesh.vertices[tri.v0]
    let b = mesh.vertices[tri.v1]
    let c = mesh.vertices[tri.v2]
    result += triangleArea(a, b, c)
