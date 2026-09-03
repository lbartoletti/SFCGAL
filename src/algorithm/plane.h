// Copyright (c) 2012-2013, IGN France.
// Copyright (c) 2012-2024, Oslandia.
// Copyright (c) 2024-2026, SFCGAL team.
// SPDX-License-Identifier: LGPL-2.0-or-later

#ifndef SFCGAL_ALGORITHM_PLANE_H_
#define SFCGAL_ALGORITHM_PLANE_H_

#include "SFCGAL/Exception.h"
#include "SFCGAL/Polygon.h"
#include "SFCGAL/algorithm/normal.h"
#include "SFCGAL/detail/GetPointsVisitor.h"

namespace SFCGAL::algorithm {

/**
 * @brief Test if a 3D plane can be extracted from a Polygon
 * @param polygon The input polygon
 * @param pointA Output parameter for first point
 * @param pointB Output parameter for second point
 * @param pointC Output parameter for third point
 * @return true if a plane can be extracted, false otherwise
 */
template <typename Kernel>
auto
hasPlane3D(const Polygon &polygon, CGAL::Point_3<Kernel> &pointA,
           CGAL::Point_3<Kernel> &pointB, CGAL::Point_3<Kernel> &pointC) -> bool
{
  if (polygon.isEmpty()) {
    return false;
  }

  const LineString &exteriorRing = polygon.exteriorRing();

  /*
   * look for 3 non collinear points
   */
  size_t nbCollinear = 0;

  for (size_t i = 0; i < exteriorRing.numPoints(); i++) {
    Point_3 point = exteriorRing.pointN(i).toPoint_3();

    if (nbCollinear == 0) {
      pointA = point;
      nbCollinear++;
    } else if (nbCollinear == 1 && pointA != point) {
      pointB = point;
      nbCollinear++;
    } else if (nbCollinear == 2 && !CGAL::collinear(pointA, pointB, point)) {
      pointC = point;
      nbCollinear++;
      return true;
    }
  }

  BOOST_ASSERT(nbCollinear < 3);
  return false;
}

/**
 * @brief Test if a 3D plane can be extracted from a Polygon
 * @param polygon The input polygon
 * @return true if a plane can be extracted, false otherwise
 */
template <typename Kernel>
auto
hasPlane3D(const Polygon &polygon) -> bool
{
  // temporary arguments
  CGAL::Point_3<Kernel> pointA;
  CGAL::Point_3<Kernel> pointB;
  CGAL::Point_3<Kernel> pointC;
  return hasPlane3D(polygon, pointA, pointB, pointC);
}

/**
 * @brief Get 3 non collinear points from a Polygon
 * @param polygon The input polygon
 * @param pointA Output parameter for first point
 * @param pointB Output parameter for second point
 * @param pointC Output parameter for third point
 */
template <typename Kernel>
auto
plane3D(const Polygon &polygon, CGAL::Point_3<Kernel> &pointA,
        CGAL::Point_3<Kernel> &pointB, CGAL::Point_3<Kernel> &pointC) -> void
{
  if (!hasPlane3D(polygon, pointA, pointB, pointC)) {
    throw Exception(
        std::format("can't find plane for Polygon '{}'", polygon.asText(3)));
  }
}

/**
 * @brief Returns the oriented 3D plane of a polygon (supposed to be planar).
 * May return degenerate plane.
 * @param polygon The input polygon
 * @return The 3D plane of the polygon
 */
template <typename Kernel>
auto
plane3D(const Polygon &polygon) -> CGAL::Plane_3<Kernel>
{
  if (polygon.isEmpty()) {
    throw Exception("Cannot compute plane for empty polygon");
  }

  CGAL::Vector_3<Kernel> nrml = normal3D<Kernel>(polygon, true);

  return CGAL::Plane_3<Kernel>(polygon.exteriorRing().pointN(0).toPoint_3(),
                               nrml);
}

struct Plane3DInexactUnsafe {};

/**
 * @brief Returns the oriented 3D plane of a polygon (supposed to be planar) -
 * inexact version.
 * @param polygon The input polygon
 * @return The 3D plane of the polygon
 * @warning Will divide by zero if polygon is degenerate.
 * @warning result is rounded to double (avoid huge expression tree).
 */
template <typename Kernel>
auto
plane3D(const Polygon &polygon, const Plane3DInexactUnsafe & /* unused */)
    -> CGAL::Plane_3<Kernel>
{
  if (polygon.isEmpty()) {
    throw Exception("Cannot compute plane for empty polygon");
  }

  CGAL::Vector_3<Kernel> nrml = normal3D<Kernel>(polygon, false);

  const double nrm = std::sqrt(CGAL::to_double(nrml.squared_length()));
  nrml = CGAL::Vector_3<Kernel>(nrml.x() / nrm, nrml.y() / nrm, nrml.z() / nrm);

  return CGAL::Plane_3<Kernel>(polygon.exteriorRing().pointN(0).toPoint_3(),
                               nrml);
}

/**
 * @brief Returns the oriented 3D plane of a polygon (supposed to be planar).
 * This is legacy code for SFCGAL users and should be deprecated.
 * @param polygon The input polygon
 * @param exact Whether to use exact computation
 * @return The 3D plane of the polygon
 * @warning result is rounded to double if exact is false (avoid huge expression
 * tree).
 * @warning Will divide by zero if polygon is degenerate. This maintains the
 * previous behaviour.
 */
template <typename Kernel>
auto
plane3D(const Polygon &polygon, bool exact) -> CGAL::Plane_3<Kernel>
{
  if (exact) {
    return plane3D<Kernel>(polygon);
  }

  return plane3D<Kernel>(polygon, Plane3DInexactUnsafe());
}

/**
 * @brief Test whether all points of a geometry lie in the same plane.
 *
 * The test is performed with an absolute distance tolerance.
 *
 * The algorithm is:
 * - compute the centroid C of all points;
 * - find the farthest point F from C;
 * - find the point G farthest from the line (CF);
 * - return true for coincident or collinear point sets;
 * - compute a normal with Newell's formula;
 * - fall back to (CF) x (CG) if Newell's normal vanishes;
 * - check the signed distance of every point from the plane (C, N).
 *
 * Newell's formula is appropriate for an ordered polygonal contour. It is
 * not a general least-squares fit for an arbitrary unordered point cloud.
 *
 * @param geom The input geometry.
 * @param toleranceAbs The absolute distance tolerance. Must be non-negative.
 *
 * @return true if all points lie within toleranceAbs of the same plane,
 *         false otherwise.
 */
template <typename Kernel>
auto
isPlane3D(const Geometry &geom, const double &toleranceAbs) -> bool
{
  BOOST_ASSERT(toleranceAbs >= 0.0);

  if (geom.isEmpty()) {
    return true;
  }

  using namespace SFCGAL::detail;
  using Vector_3 = CGAL::Vector_3<Kernel>;

  GetPointsVisitor visitor;
  const_cast<Geometry &>(geom).accept(visitor);

  if (visitor.points.empty()) {
    return true;
  }

  const auto end       = visitor.points.end();
  const auto numPoints = visitor.points.size();

  BOOST_ASSERT(numPoints > 0);

  /*
   * Compute the centroid of the point set.
   *
   * The conversion of numPoints to Kernel::FT avoids performing the
   * division through an intermediate integer type.
   */
  Vector_3 centroid(0, 0, 0);

  for (auto point = visitor.points.begin(); point != end; ++point) {
    centroid = centroid + (*point)->toVector_3();
  }

  centroid = centroid / typename Kernel::FT(numPoints);

  /*
   * Find the point F farthest from the centroid.
   *
   * If the maximum distance is smaller than the tolerance, all points are
   * considered coincident for the purpose of this test.
   */
  Vector_3            farthest      = centroid;
  typename Kernel::FT maxDistanceSq = 0;

  for (auto point = visitor.points.begin(); point != end; ++point) {
    const Vector_3 pointVector = (*point)->toVector_3();
    const Vector_3 centroidToPoint = pointVector - centroid;
    const typename Kernel::FT distanceSq =
        centroidToPoint.squared_length();

    if (distanceSq > maxDistanceSq) {
      farthest      = pointVector;
      maxDistanceSq = distanceSq;
    }
  }

  if (std::sqrt(CGAL::to_double(maxDistanceSq)) <= toleranceAbs) {
    // All points are coincident, hence they are coplanar.
    return true;
  }

  /*
   * Find the point G farthest from the line (CF).
   *
   * centroidFarthest is non-zero here because the coincident case was
   * handled above.
   */
  const Vector_3 centroidFarthest = farthest - centroid;
  const typename Kernel::FT centroidFarthestLengthSq =
      centroidFarthest.squared_length();

  Vector_3            farthestFromLine = centroid;
  maxDistanceSq = 0;

  for (auto point = visitor.points.begin(); point != end; ++point) {
    const Vector_3 pointVector = (*point)->toVector_3();
    const Vector_3 centroidToPoint = pointVector - centroid;

    /*
     * Projection of (C -> X) onto the direction (C -> F).
     */
    const Vector_3 projection =
        (centroidToPoint * centroidFarthest) *
        centroidFarthest / centroidFarthestLengthSq;

    const Vector_3 perpendicularComponent =
        centroidToPoint - projection;

    const typename Kernel::FT distanceSq =
        perpendicularComponent.squared_length();

    if (distanceSq > maxDistanceSq) {
      farthestFromLine = pointVector;
      maxDistanceSq    = distanceSq;
    }
  }

  if (std::sqrt(CGAL::to_double(maxDistanceSq)) <= toleranceAbs) {
    // All points are coincident or collinear, hence they are coplanar.
    return true;
  }

  /*
   * Compute a normal using Newell's formula.
   *
   * This is meaningful when visitor.points contains one ordered, closed
   * polygonal contour. Every consecutive pair contributes to the normal,
   * including the closing edge from the last point to the first point.
   *
   * For an arbitrary point ordering or for several unrelated contours
   * concatenated into one sequence, Newell's formula does not represent a
   * general least-squares plane fit.
   */
  Vector_3 normal(0, 0, 0);
  Vector_3 previous =
      visitor.points[numPoints - 1]->toVector_3();

  for (std::size_t i = 0; i < numPoints; ++i) {
    const Vector_3 current = visitor.points[i]->toVector_3();

    normal = normal +
             Vector_3(
                 (previous.y() - current.y()) *
                     (previous.z() + current.z()),
                 (previous.z() - current.z()) *
                     (previous.x() + current.x()),
                 (previous.x() - current.x()) *
                     (previous.y() + current.y()));

    previous = current;
  }

  /*
   * Newell's sum can vanish for a non-collinear sequence, for example when
   * the contour is traversed back and forth or when contributions cancel.
   *
   * In that case, use the normal obtained from the two independent
   * directions (C -> F) and (C -> G).
   */
  if (normal == CGAL::NULL_VECTOR) {
    normal = CGAL::cross_product(
        centroidFarthest,
        farthestFromLine - centroid);
  }

  /*
   * This should only occur for a collinear point set, already handled above.
   * Keep the check for robustness against degenerate or cancelled input.
   */
  if (normal == CGAL::NULL_VECTOR) {
    return true;
  }

  /*
   * Normalize the normal before computing point-to-plane distances.
   *
   * toleranceAbs is an absolute distance tolerance, so the normal must be
   * unit length.
   */
  const double normalLength =
      std::sqrt(CGAL::to_double(normal.squared_length()));

  if (normalLength == 0.0) {
    return true;
  }

  const Vector_3 normalizedNormal =
      normal / typename Kernel::FT(normalLength);

  /*
   * The plane is defined by the centroid and the computed normal.
   *
   * For a unit normal N, |(X - C) . N| is the perpendicular distance from X
   * to the plane. The conversion to double is intentional because the
   * public tolerance is a double.
   */
  for (auto point = visitor.points.begin(); point != end; ++point) {
    const Vector_3 centroidToPoint =
        (*point)->toVector_3() - centroid;

    const double distance =
        std::abs(CGAL::to_double(centroidToPoint * normalizedNormal));

    if (distance > toleranceAbs) {
      return false;
    }
  }

  return true;
}

} // namespace SFCGAL::algorithm

#endif
