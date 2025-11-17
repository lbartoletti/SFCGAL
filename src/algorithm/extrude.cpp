// Copyright (c) 2012-2013, IGN France.
// Copyright (c) 2012-2024, Oslandia.
// Copyright (c) 2024-2025, SFCGAL team.
// SPDX-License-Identifier: LGPL-2.0-or-later

#include "SFCGAL/algorithm/extrude.h"

#include "SFCGAL/GeometryCollection.h"
#include "SFCGAL/LineString.h"
#include "SFCGAL/MultiLineString.h"
#include "SFCGAL/MultiPoint.h"
#include "SFCGAL/MultiPolygon.h"
#include "SFCGAL/MultiSolid.h"
#include "SFCGAL/NURBSCurve.h"
#include "SFCGAL/Point.h"
#include "SFCGAL/Polygon.h"
#include "SFCGAL/PolyhedralSurface.h"
#include "SFCGAL/Solid.h"
#include "SFCGAL/Triangle.h"
#include "SFCGAL/TriangulatedSurface.h"

#include "SFCGAL/Exception.h"

#include "SFCGAL/algorithm/force3D.h"
#include "SFCGAL/algorithm/isValid.h"
#include "SFCGAL/algorithm/normal.h"
#include "SFCGAL/algorithm/translate.h"
#include "SFCGAL/numeric.h"
#include "SFCGAL/triangulate/triangulatePolygon.h"

#include "SFCGAL/detail/tools/Log.h"

#include <CGAL/Line_3.h>
#include <CGAL/Ray_3.h>

#include <utility>

namespace SFCGAL::algorithm {

// ----------------------------------------------------------------------------------
// -- private interface
// ----------------------------------------------------------------------------------
/// @{
/// @privatesection

auto
extrude(const Point &g, const Kernel::Vector_3 &v) -> LineString *;
auto
extrude(const LineString &g, const Kernel::Vector_3 &v) -> PolyhedralSurface *;
auto
extrude(const Polygon &g, const Kernel::Vector_3 &v, bool addTop = true)
    -> Solid *;
auto
extrude(const Triangle &g, const Kernel::Vector_3 &v) -> Solid *;

auto
extrude(const MultiPoint &g, const Kernel::Vector_3 &v) -> MultiLineString *;
auto
extrude(const MultiLineString &g, const Kernel::Vector_3 &v)
    -> PolyhedralSurface *;
auto
extrude(const MultiPolygon &g, const Kernel::Vector_3 &v) -> MultiSolid *;

auto
extrude(const TriangulatedSurface &g, const Kernel::Vector_3 &v) -> Solid *;
auto
extrude(const PolyhedralSurface &g, const Kernel::Vector_3 &v) -> Solid *;

auto
extrude(const GeometryCollection &g, const Kernel::Vector_3 &v)
    -> GeometryCollection *;

auto
extrude(const Point &g, const Kernel::Vector_3 &v) -> LineString *
{
  if (g.isEmpty()) {
    return new LineString();
  }

  Kernel::Point_3 const a = g.toPoint_3();
  Kernel::Point_3 const b = a + v;

  return new LineString(Point(a), Point(b));
}

auto
extrude(const LineString &g, const Kernel::Vector_3 &v) -> PolyhedralSurface *
{

  std::unique_ptr<PolyhedralSurface> polyhedralSurface(new PolyhedralSurface());

  if (g.isEmpty()) {
    return polyhedralSurface.release();
  }

  for (size_t i = 0; i < g.numPoints() - 1; i++) {
    std::unique_ptr<LineString> ring(new LineString);

    Kernel::Point_3 const a = g.pointN(i).toPoint_3();
    Kernel::Point_3 const b = g.pointN(i + 1).toPoint_3();
    ring->addPoint(new Point(a));
    ring->addPoint(new Point(b));
    ring->addPoint(new Point(b + v));
    ring->addPoint(new Point(a + v));
    ring->addPoint(new Point(a));

    polyhedralSurface->addPatch(new Polygon(ring.release()));
  }

  return polyhedralSurface.release();
}

auto
extrude(const Polygon &g, const Kernel::Vector_3 &v, bool addTop) -> Solid *
{
  if (g.isEmpty()) {
    return new Solid();
  }

  bool const reverseOrientation = (v * normal3D<Kernel>(g)) > 0;

  // resulting shell
  PolyhedralSurface polyhedralSurface;

  // "bottom"
  Polygon bottom(g);
  force3D(bottom);

  if (reverseOrientation) {
    bottom.reverse();
  }

  polyhedralSurface.addPatch(bottom);

  // "top"
  if (addTop) {
    Polygon top(bottom);
    top.reverse();
    translate(top, v);
    polyhedralSurface.addPatch(top);
  }
  // exterior ring and interior rings extruded
  for (size_t i = 0; i < bottom.numRings(); i++) {
    std::unique_ptr<PolyhedralSurface> boundaryExtruded(
        extrude(bottom.ringN(i), v));

    for (size_t j = 0; j < boundaryExtruded->numPatches(); j++) {
      boundaryExtruded->patchN(j).reverse();
      polyhedralSurface.addPatch(boundaryExtruded->patchN(j));
    }
  }

  return new Solid(polyhedralSurface);
}

auto
extrude(const Triangle &g, const Kernel::Vector_3 &v) -> Solid *
{
  return extrude(g.toPolygon(), v);
}

auto
extrude(const MultiPoint &g, const Kernel::Vector_3 &v) -> MultiLineString *
{
  std::unique_ptr<MultiLineString> result(new MultiLineString());

  if (g.isEmpty()) {
    return result.release();
  }

  for (size_t i = 0; i < g.numGeometries(); i++) {
    result->addGeometry(extrude(g.pointN(i), v));
  }

  return result.release();
}

auto
extrude(const MultiLineString &g, const Kernel::Vector_3 &v)
    -> PolyhedralSurface *
{
  std::unique_ptr<PolyhedralSurface> result(new PolyhedralSurface());

  if (g.isEmpty()) {
    return result.release();
  }

  for (size_t i = 0; i < g.numGeometries(); i++) {
    std::unique_ptr<PolyhedralSurface> extruded(extrude(g.lineStringN(i), v));

    for (size_t j = 0; j < extruded->numPatches(); j++) {
      result->addPatch(extruded->patchN(j));
    }
  }

  return result.release();
}

auto
extrude(const MultiPolygon &g, const Kernel::Vector_3 &v) -> MultiSolid *
{
  std::unique_ptr<MultiSolid> result(new MultiSolid());

  if (g.isEmpty()) {
    return result.release();
  }

  for (size_t i = 0; i < g.numGeometries(); i++) {
    result->addGeometry(extrude(g.polygonN(i), v));
  }

  return result.release();
}

auto
extrude(const TriangulatedSurface &g, const Kernel::Vector_3 &v) -> Solid *
{
  std::unique_ptr<Solid> result(new Solid());

  if (g.isEmpty()) {
    return result.release();
  }

  // bottom and top
  for (size_t i = 0; i < g.numPatches(); i++) {
    Triangle bottomPart(g.patchN(i));
    force3D(bottomPart);
    bottomPart.reverse();
    result->exteriorShell().addPatch(bottomPart);

    Triangle topPart(g.patchN(i));
    force3D(topPart);
    translate(topPart, v);
    result->exteriorShell().addPatch(topPart);
  }

  // boundary
  std::unique_ptr<Geometry> boundary(g.boundary());
  BOOST_ASSERT(boundary.get() != NULL);

  // closed surface extruded
  if (!boundary->isEmpty()) {
    std::unique_ptr<Geometry> extrudedBoundary(extrude(*boundary, v));
    BOOST_ASSERT(extrudedBoundary->is<PolyhedralSurface>());
    result->exteriorShell().addPolygons(
        extrudedBoundary->as<PolyhedralSurface>());
  }

  return result.release();
}

auto
extrude(const PolyhedralSurface &g, const Kernel::Vector_3 &v) -> Solid *
{
  if (g.isEmpty()) {
    return new Solid();
  }

  TriangulatedSurface triangulatedSurface;
  triangulate::triangulatePolygon3D(g, triangulatedSurface);
  return extrude(triangulatedSurface, v);
}

auto
extrude(const GeometryCollection &g, const Kernel::Vector_3 &v)
    -> GeometryCollection *
{
  std::unique_ptr<GeometryCollection> result(new GeometryCollection());

  if (g.isEmpty()) {
    return result.release();
  }

  for (size_t i = 0; i < g.numGeometries(); i++) {
    result->addGeometry(extrude(g.geometryN(i), v).release());
  }

  return result.release();
}

/// @private
auto
extrude(const Geometry &inputGeometry, const Kernel::Vector_3 &vector)
    -> std::unique_ptr<Geometry>
{
  switch (inputGeometry.geometryTypeId()) {
  case TYPE_POINT:
    return std::unique_ptr<Geometry>(
        extrude(inputGeometry.as<Point>(), vector));

  case TYPE_LINESTRING:
    return std::unique_ptr<Geometry>(
        extrude(inputGeometry.as<LineString>(), vector));

  case TYPE_NURBSCURVE: {
    auto lineString =
        inputGeometry.as<NURBSCurve>().toLineString(); // default parameters
    if (!lineString || lineString->isEmpty()) {
      return std::make_unique<PolyhedralSurface>(); // empty result
    }
    return std::unique_ptr<Geometry>(extrude(*lineString, vector));
  }

  case TYPE_POLYGON:
    return std::unique_ptr<Geometry>(
        extrude(inputGeometry.as<Polygon>(), vector));

  case TYPE_TRIANGLE:
    return std::unique_ptr<Geometry>(
        extrude(inputGeometry.as<Triangle>(), vector));

  case TYPE_GEOMETRYCOLLECTION:
    return std::unique_ptr<Geometry>(
        extrude(inputGeometry.as<GeometryCollection>(), vector));

  case TYPE_MULTIPOINT:
    return std::unique_ptr<Geometry>(
        extrude(inputGeometry.as<MultiPoint>(), vector));

  case TYPE_MULTILINESTRING:
    return std::unique_ptr<Geometry>(
        extrude(inputGeometry.as<MultiLineString>(), vector));

  case TYPE_MULTIPOLYGON:
    return std::unique_ptr<Geometry>(
        extrude(inputGeometry.as<MultiPolygon>(), vector));

  case TYPE_TRIANGULATEDSURFACE:
    return std::unique_ptr<Geometry>(
        extrude(inputGeometry.as<TriangulatedSurface>(), vector));

  case TYPE_POLYHEDRALSURFACE:
    return std::unique_ptr<Geometry>(
        extrude(inputGeometry.as<PolyhedralSurface>(), vector));

  case TYPE_SOLID:
  case TYPE_MULTISOLID:
    // extrusion not available
    break;
  }

  BOOST_THROW_EXCEPTION(InappropriateGeometryException(
      (boost::format("Extrusion of %s is not supported") %
       inputGeometry.geometryType())
          .str()));
}

/// @private
auto
extrude(const Geometry &inputGeom, const Kernel::FT &displacementX,
        const Kernel::FT &displacementY, const Kernel::FT &displacementZ,
        NoValidityCheck /*unused*/) -> std::unique_ptr<Geometry>
{
  return extrude(inputGeom,
                 Kernel::Vector_3(displacementX, displacementY, displacementZ));
}

/// @private
auto
extrude(const Geometry &geometry, const Kernel::FT &deltaX,
        const Kernel::FT &deltaY, const Kernel::FT &deltaZ)
    -> std::unique_ptr<Geometry>
{
  SFCGAL_ASSERT_GEOMETRY_VALIDITY(geometry);
  std::unique_ptr<Geometry> result(
      extrude(geometry, deltaX, deltaY, deltaZ, NoValidityCheck()));
  propagateValidityFlag(*result, true);
  return result;
}

/// @private
SFCGAL_API auto
extrude(const Geometry &geom, const double &displacementX,
        const double &displacementY, const double &displacementZ)
    -> std::unique_ptr<Geometry>
{
  if (!std::isfinite(displacementX) || !std::isfinite(displacementY) ||
      !std::isfinite(displacementZ)) {
    BOOST_THROW_EXCEPTION(NonFiniteValueException(
        "trying to extrude with non finite value in direction"));
  }

  return extrude(geom, Kernel::FT(displacementX), Kernel::FT(displacementY),
                 Kernel::FT(displacementZ));
}

/// @private
SFCGAL_API auto
extrude(const Polygon &polygon, const double &height)
    -> std::unique_ptr<Geometry>
{

  if (!std::isfinite(height)) {
    BOOST_THROW_EXCEPTION(NonFiniteValueException(
        "trying to extrude with non finite value in direction"));
  }

  return std::unique_ptr<Geometry>(
      extrude(polygon, Kernel::Vector_3(0.0, 0.0, height), false));
}


/// @private
SFCGAL_API auto
extrudeUntil(const Polygon &footprint, const PolyhedralSurface &roof)
    -> std::unique_ptr<Solid>
{
  // Handle empty inputs
  if (footprint.isEmpty() || roof.isEmpty()) {
    return std::make_unique<Solid>();  // Return empty solid, not exception
  }

  // Build the Solid shell
  std::unique_ptr<PolyhedralSurface> shell = std::make_unique<PolyhedralSurface>();

  // 1. Add bottom face (footprint at z=0)
  Polygon bottomFace = footprint;
  force3D(bottomFace);
  bottomFace.reverse(); // Normal pointing down (into solid)
  shell->addPolygon(bottomFace);

  // 2. Add roof as top surface
  for (size_t i = 0; i < roof.numPolygons(); ++i) {
    shell->addPolygon(roof.polygonN(i));
  }

  // 3. Detect and create lateral faces from roof boundary to footprint
  // Extract boundary edges from roof (edges that appear only once = boundary)
  struct Edge {
    Point p1, p2;
    bool operator<(const Edge &other) const {
      if (p1.x() != other.p1.x()) return p1.x() < other.p1.x();
      if (p1.y() != other.p1.y()) return p1.y() < other.p1.y();
      if (p1.z() != other.p1.z()) return p1.z() < other.p1.z();
      if (p2.x() != other.p2.x()) return p2.x() < other.p2.x();
      if (p2.y() != other.p2.y()) return p2.y() < other.p2.y();
      return p2.z() < other.p2.z();
    }
  };

  std::map<Edge, int> edgeCount;

  // Count edge occurrences in roof (all rings: exterior + interior)
  for (size_t polyIdx = 0; polyIdx < roof.numPolygons(); ++polyIdx) {
    const Polygon &poly = roof.polygonN(polyIdx);

    // Process all rings (exterior + interior)
    for (size_t ringIdx = 0; ringIdx < poly.numRings(); ++ringIdx) {
      const LineString &ring = poly.ringN(ringIdx);

      for (size_t i = 0; i < ring.numPoints() - 1; ++i) {
        Point p1 = ring.pointN(i);
        Point p2 = ring.pointN(i + 1);

        // Normalize edge direction (smaller point first)
        Edge edge;
        if (p1.x() < p2.x() || (p1.x() == p2.x() && p1.y() < p2.y()) ||
            (p1.x() == p2.x() && p1.y() == p2.y() && p1.z() < p2.z())) {
          edge.p1 = p1;
          edge.p2 = p2;
        } else {
          edge.p1 = p2;
          edge.p2 = p1;
        }

        edgeCount[edge]++;
      }
    }
  }

  // Boundary edges are those with count == 1
  std::vector<std::pair<Point, Point>> boundaryEdges;
  for (const auto &[edge, count] : edgeCount) {
    if (count == 1) {
      boundaryEdges.push_back({edge.p1, edge.p2});
    }
  }

  const Kernel::FT TOLERANCE = Kernel::FT(EPSILON);

  // 4. Create lateral faces for each roof boundary edge
  for (const auto &[roofP1, roofP2] : boundaryEdges) {
    // Project roof edge endpoints down to z=0
    Point base1(roofP1.x(), roofP1.y(), Kernel::FT(0));
    Point base2(roofP2.x(), roofP2.y(), Kernel::FT(0));

    // Check if projected points lie on footprint boundary
    bool p1OnBoundary = false;
    bool p2OnBoundary = false;

    for (size_t ringIdx = 0; ringIdx < footprint.numRings(); ++ringIdx) {
      const LineString &ring = footprint.ringN(ringIdx);

      for (size_t i = 0; i < ring.numPoints() - 1; ++i) {
        Point v1 = ring.pointN(i);
        Point v2 = ring.pointN(i + 1);

        // Check if base1 is on edge [v1, v2]
        Kernel::Segment_2 seg(Point_2(v1.x(), v1.y()), Point_2(v2.x(), v2.y()));
        Point_2 pt1(base1.x(), base1.y());

        if (CGAL::squared_distance(pt1, seg) < TOLERANCE * TOLERANCE) {
          p1OnBoundary = true;
        }

        // Check if base2 is on edge [v1, v2]
        Point_2 pt2(base2.x(), base2.y());
        if (CGAL::squared_distance(pt2, seg) < TOLERANCE * TOLERANCE) {
          p2OnBoundary = true;
        }
      }
    }

    // If both projected points are on footprint boundary, create lateral face
    if (p1OnBoundary && p2OnBoundary) {
      std::unique_ptr<LineString> lateralRing = std::make_unique<LineString>();
      lateralRing->addPoint(base1);
      lateralRing->addPoint(base2);
      lateralRing->addPoint(roofP2);
      lateralRing->addPoint(roofP1);
      lateralRing->addPoint(base1);

      shell->addPolygon(Polygon(lateralRing.release()));
    }
  }

  // 5. Create lateral faces for interior rings (holes) in footprint
  //    Phase 1 always handles the exterior ring via roof boundary edges
  //    Phase 2 handles interior rings that may not have corresponding roof geometry
  for (size_t ringIdx = 1; ringIdx < footprint.numRings(); ++ringIdx) {
    const LineString &ring = footprint.ringN(ringIdx);

    for (size_t i = 0; i < ring.numPoints() - 1; ++i) {
      Point v1 = ring.pointN(i);
      Point v2 = ring.pointN(i + 1);

      // Ray-cast to find roof height at these points
      Point base1(v1.x(), v1.y(), Kernel::FT(0));
      Point base2(v2.x(), v2.y(), Kernel::FT(0));

      // Find z on roof by ray-casting from each point
      auto findRoofZ = [&](const Point &basePoint) -> Kernel::FT {
        Kernel::Point_3 rayOrigin(basePoint.x(), basePoint.y(), Kernel::FT(0));
        Kernel::Vector_3 rayDirection(Kernel::FT(0), Kernel::FT(0), Kernel::FT(1));
        CGAL::Ray_3<Kernel> ray(rayOrigin, rayDirection);

        Kernel::FT minZ = Kernel::FT(std::numeric_limits<double>::max());

        for (size_t polyIdx = 0; polyIdx < roof.numPolygons(); ++polyIdx) {
          const Polygon &poly = roof.polygonN(polyIdx);
          const LineString &roofRing = poly.exteriorRing();

          if (roofRing.numPoints() < 4) continue;

          // Triangulate face
          const Point &p0 = roofRing.pointN(0);
          Kernel::Point_3 v0(p0.x(), p0.y(), p0.z());

          for (size_t j = 1; j < roofRing.numPoints() - 2; ++j) {
            const Point &p1 = roofRing.pointN(j);
            const Point &p2 = roofRing.pointN(j + 1);

            Kernel::Point_3 v1(p1.x(), p1.y(), p1.z());
            Kernel::Point_3 v2(p2.x(), p2.y(), p2.z());

            CGAL::Triangle_3<Kernel> triangle(v0, v1, v2);

            auto result = CGAL::intersection(ray, triangle);
            if (result) {
              if (const Kernel::Point_3 *point = std::get_if<Kernel::Point_3>(&(*result))) {
                Kernel::FT z = point->z();
                if (z < minZ && z > Kernel::FT(-EPSILON)) {
                  minZ = z;
                }
              }
            }
          }
        }
        return minZ;
      };

      Kernel::FT z1 = findRoofZ(base1);
      Kernel::FT z2 = findRoofZ(base2);

      // Create vertical face if valid z found
      if (z1 < Kernel::FT(std::numeric_limits<double>::max()) &&
          z2 < Kernel::FT(std::numeric_limits<double>::max())) {

        Point top1(base1.x(), base1.y(), z1);
        Point top2(base2.x(), base2.y(), z2);

        std::unique_ptr<LineString> lateralRing = std::make_unique<LineString>();

        // Interior ring: inward normal (reversed)
        lateralRing->addPoint(base1);
        lateralRing->addPoint(top1);
        lateralRing->addPoint(top2);
        lateralRing->addPoint(base2);
        lateralRing->addPoint(base1);

        shell->addPolygon(Polygon(lateralRing.release()));
      }
    }
  }

  return std::make_unique<Solid>(shell.release());
}
} // namespace SFCGAL::algorithm
