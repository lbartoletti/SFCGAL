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
    return std::make_unique<Solid>();
  }

  // Prepare to store projected vertices for all rings
  std::vector<std::vector<Point>> projectedRings;

  // Process all rings (exterior + interior)
  for (size_t ringIdx = 0; ringIdx < footprint.numRings(); ++ringIdx) {
    const LineString  &ring = footprint.ringN(ringIdx);
    std::vector<Point> projectedVertices;

    // For each vertex in the ring
    for (size_t i = 0; i < ring.numPoints(); ++i) {
      const Point &vertex = ring.pointN(i);

      // Create vertical ray from (x, y, 0) upward
      Kernel::Point_3     rayOrigin(vertex.x(), vertex.y(), Kernel::FT(0));
      Kernel::Vector_3    rayDirection(Kernel::FT(0), Kernel::FT(0),
                                       Kernel::FT(1));
      CGAL::Ray_3<Kernel> ray(rayOrigin, rayDirection);

      // Find intersection with roof
      bool            foundIntersection = false;
      Kernel::Point_3 closestIntersection;
      Kernel::FT      minZ = Kernel::FT(std::numeric_limits<double>::max());

      // Check intersection with each polygon face in the roof
      for (size_t faceIdx = 0; faceIdx < roof.numPolygons(); ++faceIdx) {
        const Polygon &face = roof.polygonN(faceIdx);

        // Triangulate the face if it has more than 3 vertices
        if (face.exteriorRing().numPoints() < 4) {
          continue; // Degenerate face
        }

        // Simple triangulation: fan from first vertex
        const Point    &p0 = face.exteriorRing().pointN(0);
        Kernel::Point_3 v0(p0.x(), p0.y(), p0.z());

        for (size_t j = 1; j < face.exteriorRing().numPoints() - 2; ++j) {
          const Point &p1 = face.exteriorRing().pointN(j);
          const Point &p2 = face.exteriorRing().pointN(j + 1);

          Kernel::Point_3 v1(p1.x(), p1.y(), p1.z());
          Kernel::Point_3 v2(p2.x(), p2.y(), p2.z());

          CGAL::Triangle_3<Kernel> triangle(v0, v1, v2);

          // Check intersection between ray and triangle
          auto result = CGAL::intersection(ray, triangle);

          if (result) {
            // Extract intersection point
            if (const Kernel::Point_3 *point =
                    std::get_if<Kernel::Point_3>(&(*result))) {
              Kernel::FT z = point->z();

              // Keep the closest intersection (first hit from below)
              if (z < minZ && z > Kernel::FT(-EPSILON)) {
                minZ                = z;
                closestIntersection = *point;
                foundIntersection   = true;
              }
            }
          }
        }
      }

      if (!foundIntersection) {
        return std::make_unique<Solid>();
      }

      // Store the projected vertex
      projectedVertices.push_back(Point(closestIntersection));
    }

    projectedRings.push_back(projectedVertices);
  }

  // Check if the top surface is nearly planar
  // Calculate z variance
  Kernel::FT sumZ        = Kernel::FT(0);
  size_t     totalPoints = 0;
  for (const auto &ring : projectedRings) {
    for (const auto &pt : ring) {
      sumZ += pt.z();
      totalPoints++;
    }
  }
  Kernel::FT meanZ = sumZ / Kernel::FT(totalPoints);

  Kernel::FT variance = Kernel::FT(0);
  for (const auto &ring : projectedRings) {
    for (const auto &pt : ring) {
      Kernel::FT diff = pt.z() - meanZ;
      variance += diff * diff;
    }
  }
  variance = variance / Kernel::FT(totalPoints);

  const Kernel::FT PLANARITY_THRESHOLD =
      Kernel::FT(1) / Kernel::FT(100); // 0.01
  bool isNearlyPlanar = (variance < PLANARITY_THRESHOLD * PLANARITY_THRESHOLD);

  // Build the Solid
  std::unique_ptr<PolyhedralSurface> shell =
      std::make_unique<PolyhedralSurface>();

  // 1. Add bottom face (footprint at z=0)
  Polygon bottomFace = footprint;
  force3D(bottomFace);
  // Orient with normal pointing down (into solid)
  bottomFace.reverse();
  shell->addPolygon(bottomFace);

  // 2. Add lateral faces for each ring
  for (size_t ringIdx = 0; ringIdx < footprint.numRings(); ++ringIdx) {
    const LineString         &bottomRing = footprint.ringN(ringIdx);
    const std::vector<Point> &topRing    = projectedRings[ringIdx];

    // For exterior ring, faces should have outward normals
    // For interior rings (holes), faces should have inward normals
    bool isExteriorRing = (ringIdx == 0);

    for (size_t i = 0; i < bottomRing.numPoints() - 1; ++i) {
      // Create quad face
      std::unique_ptr<LineString> faceRing = std::make_unique<LineString>();

      Point bottom1 = bottomRing.pointN(i);
      force3D(bottom1);
      Point bottom2 = bottomRing.pointN(i + 1);
      force3D(bottom2);
      Point top1 = topRing[i];
      Point top2 = topRing[i + 1];

      if (isExteriorRing) {
        // Exterior: counter-clockwise from outside
        faceRing->addPoint(bottom1);
        faceRing->addPoint(bottom2);
        faceRing->addPoint(top2);
        faceRing->addPoint(top1);
        faceRing->addPoint(bottom1);
      } else {
        // Interior: clockwise from outside (reversed)
        faceRing->addPoint(bottom1);
        faceRing->addPoint(top1);
        faceRing->addPoint(top2);
        faceRing->addPoint(bottom2);
        faceRing->addPoint(bottom1);
      }

      shell->addPolygon(Polygon(faceRing.release()));
    }
  }

  // 3. Add top face
  if (isNearlyPlanar) {
    // Create a single polygon for the top face
    std::unique_ptr<Polygon> topFace = std::make_unique<Polygon>();

    // Add exterior ring
    std::unique_ptr<LineString> exteriorRing = std::make_unique<LineString>();
    for (const auto &pt : projectedRings[0]) {
      exteriorRing->addPoint(pt);
    }
    topFace->exteriorRing() = *exteriorRing;

    // Add interior rings if any
    for (size_t ringIdx = 1; ringIdx < projectedRings.size(); ++ringIdx) {
      std::unique_ptr<LineString> interiorRing = std::make_unique<LineString>();
      for (const auto &pt : projectedRings[ringIdx]) {
        interiorRing->addPoint(pt);
      }
      topFace->addInteriorRing(interiorRing.release());
    }

    // Orient with normal pointing up (out of solid)
    // Exterior ring should be counter-clockwise when viewed from above
    shell->addPolygon(*topFace);
  } else {
    // Non-planar: triangulate the top surface
    // Create a polygon from projected vertices
    std::unique_ptr<Polygon> topPolygon = std::make_unique<Polygon>();

    std::unique_ptr<LineString> exteriorRing = std::make_unique<LineString>();
    for (const auto &pt : projectedRings[0]) {
      exteriorRing->addPoint(pt);
    }
    topPolygon->exteriorRing() = *exteriorRing;

    for (size_t ringIdx = 1; ringIdx < projectedRings.size(); ++ringIdx) {
      std::unique_ptr<LineString> interiorRing = std::make_unique<LineString>();
      for (const auto &pt : projectedRings[ringIdx]) {
        interiorRing->addPoint(pt);
      }
      topPolygon->addInteriorRing(interiorRing.release());
    }

    // Triangulate the polygon
    TriangulatedSurface triangulatedTop;
    triangulate::triangulatePolygon3D(*topPolygon, triangulatedTop);

    // Add triangles to shell
    for (size_t i = 0; i < triangulatedTop.numTriangles(); ++i) {
      shell->addPolygon(triangulatedTop.triangleN(i).toPolygon());
    }
  }

  return std::make_unique<Solid>(shell.release());
}
} // namespace SFCGAL::algorithm
