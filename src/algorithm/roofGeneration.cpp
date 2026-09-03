// Copyright (c) 2025-2026, SFCGAL team.
// SPDX-License-Identifier: LGPL-2.0-or-later

#include "SFCGAL/algorithm/roofGeneration.h"
#include "SFCGAL/MultiLineString.h"
#include "SFCGAL/algorithm/distance.h"
#include "SFCGAL/algorithm/extrude.h"
#include "SFCGAL/algorithm/straightSkeleton.h"

#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/number_utils.h>
#include <map>

namespace SFCGAL::algorithm {

/// Distance to the footprint below which a medial axis endpoint is considered
/// to sit on the boundary rather than on the ridge.
constexpr double RIDGE_MAPPING_TOLERANCE = 1e-3;

auto
extrudeGableRoof(const Polygon &polygon, double clippingHeight,
                 double slopeAngle) -> std::unique_ptr<PolyhedralSurface>
{
  // 1. Generate standard hipped roof
  // Pass 0.0 for height to get the natural maximum height of the roof
  std::vector<Kernel::FT> angles(polygon.exteriorRing().numSegments(),
                                 slopeAngle);
  auto roof = extrudeStraightSkeleton(polygon, 0.0, {{}}, {angles});

  // 2. Compute projection mapping from medial axis
  auto medialAxisProjection = approximateMedialAxis(polygon, true);

  std::map<std::pair<double, double>, Point> ridgeToEdgeMapping;

  for (size_t segmentIdx = 0;
       segmentIdx < medialAxisProjection->numGeometries(); ++segmentIdx) {
    const auto &medialLine =
        medialAxisProjection->geometryN(segmentIdx).as<LineString>();
    if (medialLine.numPoints() < 2) {
      continue;
    }

    // Check start of medial line
    Point        firstPoint  = medialLine.pointN(0);
    Point        secondPoint = medialLine.pointN(1);
    const double firstPointDistance =
        distance(firstPoint, polygon.exteriorRing());
    const double secondPointDistance =
        distance(secondPoint, polygon.exteriorRing());

    if (firstPointDistance < RIDGE_MAPPING_TOLERANCE &&
        secondPointDistance > RIDGE_MAPPING_TOLERANCE) {
      ridgeToEdgeMapping[{CGAL::to_double(secondPoint.x()),
                          CGAL::to_double(secondPoint.y())}] = firstPoint;
    }

    // Check end of medial line
    size_t       pointCount       = medialLine.numPoints();
    Point        lastPoint        = medialLine.pointN(pointCount - 1);
    Point        penultimatePoint = medialLine.pointN(pointCount - 2);
    const double lastPointDistance =
        distance(lastPoint, polygon.exteriorRing());
    const double penultimatePointDistance =
        distance(penultimatePoint, polygon.exteriorRing());

    if (lastPointDistance < RIDGE_MAPPING_TOLERANCE &&
        penultimatePointDistance > RIDGE_MAPPING_TOLERANCE) {
      ridgeToEdgeMapping[{CGAL::to_double(penultimatePoint.x()),
                          CGAL::to_double(penultimatePoint.y())}] = lastPoint;
    }
  }

  // 3. Deform the roof: move ridge endpoints to the boundary
  for (size_t patchIdx = 0; patchIdx < roof->numPatches(); ++patchIdx) {
    Polygon    &patch = roof->patchN(patchIdx);
    LineString &ring  = patch.exteriorRing();
    for (size_t vertexIdx = 0; vertexIdx < ring.numPoints(); ++vertexIdx) {
      Point &vertex = ring.pointN(vertexIdx);
      if (vertex.z() > 0) {
        const double vertexX = CGAL::to_double(vertex.x());
        const double vertexY = CGAL::to_double(vertex.y());

        for (const auto &[ridgeCoord, edgePoint] : ridgeToEdgeMapping) {
          const double deltaX = vertexX - ridgeCoord.first;
          const double deltaY = vertexY - ridgeCoord.second;
          if (((deltaX * deltaX) + (deltaY * deltaY)) < 1e-4) {
            vertex = Point(edgePoint.x(), edgePoint.y(), vertex.z());
            break;
          }
        }
      }
    }
  }

  // 4. Clip the roof to the specified height if it's strictly positive
  if (clippingHeight > 0.0) {
    const Kernel::Plane_3 clippingPlane(0, 0, 1, -Kernel::FT(clippingHeight));
    auto                  surfaceMesh = roof->toSurfaceMesh();
    CGAL::Polygon_mesh_processing::clip(surfaceMesh, clippingPlane,
                                        CGAL::parameters::clip_volume(true));
    roof = std::make_unique<PolyhedralSurface>(surfaceMesh);
  }

  return roof;
}

auto
extrudeSkillionRoof(const Polygon &polygon, double clippingHeight,
                    size_t primaryEdgeIndex, double primaryAngle,
                    double secondaryAngle) -> std::unique_ptr<PolyhedralSurface>
{
  // If no clipping height, compute from geometry and slope
  if (clippingHeight <= 0.0) {
    const auto          &ring   = polygon.exteriorRing();
    const auto          &point0 = ring.pointN(primaryEdgeIndex);
    const auto          &point1 = ring.pointN(primaryEdgeIndex + 1);
    const Kernel::Line_2 edgeLine(point0.toPoint_2(), point1.toPoint_2());
    double               maxDist = 0.0;
    for (size_t i = 0; i < ring.numPoints(); ++i) {
      const double distance = std::sqrt(CGAL::to_double(
          CGAL::squared_distance(ring.pointN(i).toPoint_2(), edgeLine)));
      maxDist               = std::max(maxDist, distance);
    }
    const double rad = primaryAngle * CGAL_PI / 180.0;
    clippingHeight   = maxDist * std::tan(rad);
  }

  std::vector<Kernel::FT> angles;
  for (size_t edgeIdx = 0; edgeIdx < polygon.exteriorRing().numSegments();
       ++edgeIdx) {
    if (edgeIdx == primaryEdgeIndex) {
      angles.emplace_back(primaryAngle);
    } else {
      angles.emplace_back(secondaryAngle);
    }
  }

  std::vector<std::vector<Kernel::FT>> anglePattern = {angles};
  return extrudeStraightSkeleton(polygon, clippingHeight, {{}}, anglePattern);
}

auto
generateRoof(const Polygon &footprint, const RoofParameters &params)
    -> std::unique_ptr<Geometry>
{
  switch (params.type) {
  case RoofType::FLAT:
    return extrude(footprint, 0.0, 0.0, params.roofHeight);
  case RoofType::HIPPED:
    return extrudeStraightSkeleton(footprint, params.roofHeight);
  case RoofType::GABLE:
    return extrudeGableRoof(footprint, params.roofHeight, params.slopeAngle);
  case RoofType::SKILLION:
    return extrudeSkillionRoof(footprint, params.roofHeight,
                               params.primaryEdgeIndex, params.slopeAngle,
                               90.0);
  }
  return nullptr;
}

auto
generateRoof(const Polygon &footprint, const RoofParameters &params,
             NoValidityCheck & /*nvc*/) -> std::unique_ptr<Geometry>
{
  return generateRoof(footprint, params);
}

} // namespace SFCGAL::algorithm
