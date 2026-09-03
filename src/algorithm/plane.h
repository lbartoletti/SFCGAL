// Copyright (c) 2012-2013, IGN France.
// Copyright (c) 2012-2024, Oslandia.
// Copyright (c) 2024-2026, SFCGAL team.
// SPDX-License-Identifier: LGPL-2.0-or-later

#ifndef SFCGAL_ALGORITHM_PLANE_H_
#define SFCGAL_ALGORITHM_PLANE_H_

#include <boost/format.hpp>

// #include "SFCGAL/detail/ublas.h"

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
    BOOST_THROW_EXCEPTION(
        Exception((boost::format("can't find plane for Polygon '%1%'") %
                   polygon.asText(3))
                      .str()));
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
    BOOST_THROW_EXCEPTION(Exception("Cannot compute plane for empty polygon"));
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
    BOOST_THROW_EXCEPTION(Exception("Cannot compute plane for empty polygon"));
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
 * @brief Test if all points of a geometry lie in the same plane
 * @param geom The input geometry
 * @param toleranceAbs The absolute tolerance for planarity test
 * @return true if all points lie in the same plane, false otherwise
 */
template <typename Kernel>
auto
isPlane3D(const Geometry &geom, const double &toleranceAbs) -> bool
{
  if (geom.isEmpty()) {
    return true;
  }

  using namespace SFCGAL::detail;
  GetPointsVisitor visitor;
  const_cast<Geometry &>(geom).accept(visitor);

  if (visitor.points.empty()) {
    return true;
  }

  // the present approach is to find a good plane by:
  // - computing the centroid C of the point set
  // - computing the unit normal N of the point sequence by Newell's formula
  // - we check that points Xi are in the plane CXi.N < tolerance
  //
  // the farthest point F from C and the farthest point G from (CF) are still
  // computed: they rule out the degenerate cases (all points at the same
  // location, all points collinear) and provide the CFxCG fallback normal when
  // Newell's sum vanishes
  //
  // note that we could compute the covarence matrix of the points and use SVD
  // but we would need a lib for that, and it may be overkill

  using Vector_3 = CGAL::Vector_3<Kernel>;

  const auto end = visitor.points.end();

  // centroid
  Vector_3          centroid(0, 0, 0);
  const std::size_t numPoints = visitor.points.size();

  for (auto x = visitor.points.begin(); x != end; ++x) {
    centroid = centroid + (*x)->toVector_3();
  }

  BOOST_ASSERT(numPoints);
  centroid = centroid / typename Kernel::FT(numPoints);

  // farthest point from centroid
  Vector_3            farthest      = centroid;
  typename Kernel::FT maxDistanceSq = 0;

  for (auto x = visitor.points.begin(); x != end; ++x) {
    const Vector_3            cx  = (*x)->toVector_3() - centroid;
    const typename Kernel::FT dSq = cx * cx;

    if (dSq > maxDistanceSq) {
      farthest      = (*x)->toVector_3();
      maxDistanceSq = dSq;
    }
  }

  if (std::sqrt(CGAL::to_double(maxDistanceSq)) < toleranceAbs) {
    // std::cout << "all points in the same location\n";
    return true;
  }

  // farthest point from line
  Vector_3       g                = centroid;
  const Vector_3 centroidFarthest = farthest - centroid; // direction of (CF)
  maxDistanceSq                   = 0; // watch out, we reuse the variable

  for (auto x = visitor.points.begin(); x != end; ++x) {
    const Vector_3 cx = (*x)->toVector_3() - centroid;
    const Vector_3 centroidProjected =
        (cx * centroidFarthest) * centroidFarthest /
        centroidFarthest.squared_length(); // projection of x on line (CF)
    const typename Kernel::FT dSq = (cx - centroidProjected).squared_length();

    if (dSq > maxDistanceSq) {
      g             = (*x)->toVector_3();
      maxDistanceSq = dSq;
    }
  }

  if (std::sqrt(CGAL::to_double(maxDistanceSq)) < toleranceAbs) {
    // std::cout << "all points aligned\n";
    return true;
  }

  // Normal of the point sequence, by Newell's formula: every edge of the
  // outline contributes, which is both exact for a planar outline and
  // equivalent to a least squares fit for a noisy one. A cross product would
  // only use three points and gets ill-conditioned as soon as they are close
  // to collinear.
  Vector_3 normal(0, 0, 0);
  Vector_3 previous = visitor.points[numPoints - 1]->toVector_3();

  for (std::size_t i = 0; i < numPoints; ++i) {
    const Vector_3 current = visitor.points[i]->toVector_3();
    normal =
        normal +
        Vector_3((previous.y() - current.y()) * (previous.z() + current.z()),
                 (previous.z() - current.z()) * (previous.x() + current.x()),
                 (previous.x() - current.x()) * (previous.y() + current.y()));
    previous = current;
  }

  // Newell's sum vanishes when the outline encloses no area, for instance when
  // it is traversed back and forth. Fall back on the three point estimate.
  if (normal == CGAL::NULL_VECTOR) {
    normal = CGAL::cross_product(centroidFarthest, g - centroid);
  }

  if (normal == CGAL::NULL_VECTOR) {
    // points are collinear, hence trivially coplanar
    return true;
  }

  const Vector_3 nNormed =
      normal / std::sqrt(CGAL::to_double(normal.squared_length()));

  for (auto x = visitor.points.begin(); x != end; ++x) {
    const Vector_3 cx = (*x)->toVector_3() - centroid;

    if (std::abs(CGAL::to_double(cx * nNormed)) > toleranceAbs) {
      // std::cout << "point out of plane\n";
      return false;
    }
  }

  // std::cout << "plane general case\n";
  return true;
}

} // namespace SFCGAL::algorithm

#endif
