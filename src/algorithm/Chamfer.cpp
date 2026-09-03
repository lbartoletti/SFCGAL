// Copyright (c) 2025-2026, SFCGAL Team.
// SPDX-License-Identifier: LGPL-2.0-or-later

#include "SFCGAL/algorithm/Chamfer.h"

#include "SFCGAL/algorithm/Sweep.h"
#include "SFCGAL/algorithm/collectionHomogenize.h"
#include "SFCGAL/algorithm/difference.h"
#include "SFCGAL/algorithm/isClosed.h"
#include "SFCGAL/algorithm/union.h"

#include "SFCGAL/Geometry.h"
#include "SFCGAL/Kernel.h"
#include "SFCGAL/LineString.h"
#include "SFCGAL/MultiLineString.h"
#include "SFCGAL/Polygon.h"
#include "SFCGAL/PolyhedralSurface.h"
#include "SFCGAL/Solid.h"
#include "SFCGAL/numeric.h"

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/squared_distance_3.h>

#include "SFCGAL/detail/tools/Log.h"

#include <cmath>
#include <stdexcept>

namespace SFCGAL::algorithm {

// ----------------------------------------------------------------------------------
// -- private interface
// ----------------------------------------------------------------------------------
/// @{
/// @privatesection

using Point_3        = Kernel::Point_3;
using Vector_3       = Kernel::Vector_3;
using Surface_mesh_3 = SFCGAL::Surface_mesh_3;

namespace PMP = CGAL::Polygon_mesh_processing;

namespace {

// Thresholds for numerical comparisons
constexpr double TOLERANCE_NEAR_ZERO_LENGTH =
    1e-12; // Length below which a projection is degenerate
constexpr double TOLERANCE_EPS_SCALE =
    1e-3; // Profile origin shift to avoid coplanar faces
constexpr double TOLERANCE_DEFAULT_EPSILON =
    EPSILON; // Default tolerance for halfedge matching
constexpr double MIN_OPENING_DEG =
    5.0; // Minimum supported opening angle (degrees)
constexpr double MAX_OPENING_DEG =
    175.0; // Maximum supported opening angle (degrees)

// ============================================================================
// Algorithm overview:
// 1. Convert solid to CGAL Surface_mesh (once for all edges)
// 2. For each input LineString: find the halfedge matching its first segment,
//    check convexity, compute dihedral angle, and build a cutting profile.
// 3. Sweep the profile along the LineString path to create a cutter solid.
//    Multi-segment paths use miter joins at corners.
// 4. Union all cutters
// 5. Boolean difference: solid - combined_cutter
// Edges that fail validation (not found, concave, etc.) are skipped with a
// warning.
// ============================================================================

// Find the halfedge in mesh that contains the segment (start_pt -> end_pt)
auto
find_halfedge(const Surface_mesh_3 &mesh, const Point_3 &start_pt,
              const Point_3 &end_pt, double epsilon = TOLERANCE_DEFAULT_EPSILON)
    -> Surface_mesh_3::Halfedge_index
{
  const double eps_sq = epsilon * epsilon;

  for (auto halfedge : mesh.halfedges()) {
    const Point_3 &src = mesh.point(mesh.source(halfedge));
    const Point_3 &tgt = mesh.point(mesh.target(halfedge));

    // Check if both start_pt and end_pt lie on the segment [src, tgt]
    // 1. Collinearity check using squared distance to the line containing the
    // segment
    const Kernel::Line_3 mesh_line(src, tgt);
    double dis1 = CGAL::to_double(CGAL::squared_distance(mesh_line, start_pt));
    double dis2 = CGAL::to_double(CGAL::squared_distance(mesh_line, end_pt));
    if (dis1 > eps_sq || dis2 > eps_sq) {
      continue;
    }

    // 2. Bounded side check using dot products: point P is between A and B if
    //    (P-A).(B-A) >= 0 AND (P-B).(A-B) >= 0
    const Vector_3 edge_dir     = tgt - src;
    const Vector_3 edge_dir_rev = src - tgt;

    auto is_between = [&](const Point_3 &pt) -> bool {
      double dot1 = CGAL::to_double((pt - src) * edge_dir);
      double dot2 = CGAL::to_double((pt - tgt) * edge_dir_rev);
      return dot1 >= -epsilon && dot2 >= -epsilon;
    };

    if (!is_between(start_pt) || !is_between(end_pt)) {
      continue;
    }

    // 3. Direction check: (end - start) must point in same direction as (tgt -
    // src)
    const Vector_3 edge_vec = end_pt - start_pt;
    double         dot_dir  = CGAL::to_double(edge_dir * edge_vec);
    if (dot_dir <= 0.0) {
      continue;
    }

    return halfedge;
  }

  return Surface_mesh_3::null_halfedge();
}

// Compute face surface directions (tangent to face, pointing toward solid
// interior) in the local (N, B) frame.
//
// In this frame, n1_perp is at angle 0 and n2_perp at angle theta_2.
// The tangent to each face is perpendicular to its normal. The correct
// tangent (pointing inward) is chosen by comparing with the inward bisector.
//
// Returns {f1_angle, f2_angle} in radians.
auto
compute_face_surface_angles(double theta_2) -> std::pair<double, double>
{
  const double sign = (theta_2 > 0) ? 1.0 : -1.0;

  // Face 1 surface direction: ⊥ to n1_perp (angle 0), pointing inward
  const double f1_angle = -(sign * M_PI) / 2.0;

  // Face 2 surface direction: ⊥ to n2_perp (angle θ₂), pointing inward
  const double f2_angle = theta_2 + ((sign * M_PI) / 2.0);

  return {f1_angle, f2_angle};
}

// Create chamfer triangle profile parameterized by the angle of n2
// in the local frame (N, B).
//
// Legs follow the face surfaces (tangent to faces, not along normals).
// This is correct for any dihedral angle, not just 90°.
auto
create_chamfer_profile_for_angle(double radius_x_val, double radius_y_val,
                                 double theta_2) -> std::unique_ptr<Polygon>
{
  auto [f1_angle, f2_angle] = compute_face_surface_angles(theta_2);

  // Leg endpoints at distance r along face surface directions
  const double leg1_x = radius_x_val * std::cos(f1_angle);
  const double leg1_y = radius_x_val * std::sin(f1_angle);

  const double leg2_x = radius_y_val * std::cos(f2_angle);
  const double leg2_y = radius_y_val * std::sin(f2_angle);

  // Small outward extension along the outward bisector of the two normals
  // to avoid coplanar faces in boolean ops.
  const double eps_shift =
      std::max(std::min(radius_x_val, radius_y_val) * TOLERANCE_EPS_SCALE,
               TOLERANCE_NEAR_ZERO_LENGTH);
  const double bis_angle = theta_2 / 2.0;
  const double center_x  = eps_shift * std::cos(bis_angle);
  const double center_y  = eps_shift * std::sin(bis_angle);

  std::vector<Point> points;
  points.reserve(4);
  points.emplace_back(center_x, center_y, 0);
  points.emplace_back(leg1_x, leg1_y, 0);
  points.emplace_back(leg2_x, leg2_y, 0);
  points.emplace_back(center_x, center_y, 0);

  return std::make_unique<Polygon>(LineString(points));
}

// Create fillet arc profile parameterized by the angle of n2.
//
// The fillet arc of radius R is tangent to both face surfaces.
// The tangent points (legs) are at distance R/tan(γ/2) from the edge,
// where γ is the opening angle. The arc center lies at the intersection
// of lines parallel to each face, offset inward by R.
auto
create_fillet_profile_for_angle(double radius, int segments, double theta_2)
    -> std::unique_ptr<Polygon>
{
  if (radius <= 0.0) {
    throw std::invalid_argument("Radius must be positive");
  }
  if (segments < 1) {
    throw std::invalid_argument("Segments must be at least 1");
  }

  auto [f1_angle, f2_angle] = compute_face_surface_angles(theta_2);

  // Opening angle γ = π - |θ₂|
  const double gamma       = M_PI - std::abs(theta_2);
  const double half_gamma  = gamma / 2.0;
  const double sin_theta_2 = std::sin(theta_2);
  const double tan_half    = std::tan(half_gamma);

  // Guard against degenerate angles (coplanar or knife-edge faces)
  if (std::abs(sin_theta_2) < TOLERANCE_NEAR_ZERO_LENGTH ||
      std::abs(tan_half) < TOLERANCE_NEAR_ZERO_LENGTH) {
    throw std::invalid_argument(
        "Degenerate dihedral angle for fillet: faces are nearly "
        "coplanar or form a knife-edge");
  }

  // Leg distance from edge = R / tan(γ/2)
  const double leg_dist = radius / tan_half;

  const double leg1_x = leg_dist * std::cos(f1_angle);
  const double leg1_y = leg_dist * std::sin(f1_angle);

  const double leg2_x = leg_dist * std::cos(f2_angle);
  const double leg2_y = leg_dist * std::sin(f2_angle);

  // Arc center: intersection of face lines offset inward by R.
  // In (N, B) frame:
  //   Face 1 equation: x = 0 → offset: x = -R
  //   Face 2 equation: x·cos(θ₂) + y·sin(θ₂) = 0 → offset: = -R
  // Solving: cx = -R, cy = R·(cos(θ₂) - 1) / sin(θ₂)
  const double center_x = -radius;
  const double center_y = radius * (std::cos(theta_2) - 1.0) / sin_theta_2;

  // Arc from leg1 to leg2 around center (center_x, center_y)
  double start_angle = std::atan2(leg1_y - center_y, leg1_x - center_x);
  double end_angle   = std::atan2(leg2_y - center_y, leg2_x - center_x);

  // Take the shorter arc through the exterior
  double delta = end_angle - start_angle;
  while (delta > M_PI) {
    delta -= 2.0 * M_PI;
  }
  while (delta < -M_PI) {
    delta += 2.0 * M_PI;
  }

  // Origin: small outward shift along the outward bisector
  const double eps_shift =
      std::max(radius * TOLERANCE_EPS_SCALE, TOLERANCE_NEAR_ZERO_LENGTH);
  const double bis_angle = theta_2 / 2.0;
  const double origin_x  = eps_shift * std::cos(bis_angle);
  const double origin_y  = eps_shift * std::sin(bis_angle);

  std::vector<Point> points;
  points.reserve(segments + 3);

  points.emplace_back(origin_x, origin_y, 0);

  for (int i = 0; i <= segments; ++i) {
    const double param = double(i) / segments;
    const double angle = start_angle + (param * delta);
    const double x_val = center_x + (radius * std::cos(angle));
    const double y_val = center_y + (radius * std::sin(angle));
    points.emplace_back(x_val, y_val, 0);
  }

  points.emplace_back(origin_x, origin_y, 0);

  return std::make_unique<Polygon>(LineString(points));
}

// Compute the signed angle of n2 relative to n1 in the plane perpendicular
// to the edge. Looking along the edge: n1's projection defines angle 0,
// and this returns the angle to n2's projection (counter-clockwise positive).
//   |theta_2| ≈ pi/2  → 90° corner (cube)
//   |theta_2| ≈ pi     → nearly flat (coplanar faces)
//   |theta_2| ≈ 0      → knife-edge
auto
compute_n2_angle(const Vector_3 &edge_dir, const Vector_3 &normal_1,
                 const Vector_3 &normal_2) -> double
{
  const Vector_3 tangent_dir = SFCGAL::normalizeVector(edge_dir);

  // N = projection of normal_1 perpendicular to edge
  Vector_3 normal_dir = normal_1 - (normal_1 * tangent_dir) * tangent_dir;
  double   normal_dir_len =
      std::sqrt(CGAL::to_double(normal_dir.squared_length()));

  if (normal_dir_len < TOLERANCE_NEAR_ZERO_LENGTH) {
    throw std::invalid_argument(
        "Face normal normal_1 is parallel to edge direction");
  }
  normal_dir = normal_dir / normal_dir_len;

  // B completes the right-handed frame
  const Vector_3 binormal_dir =
      SFCGAL::normalizeVector(CGAL::cross_product(tangent_dir, normal_dir));

  // Project normal_2 perpendicular to edge
  Vector_3 normal_2_perp = normal_2 - (normal_2 * tangent_dir) * tangent_dir;
  double   normal_2_perp_len =
      std::sqrt(CGAL::to_double(normal_2_perp.squared_length()));

  if (normal_2_perp_len < TOLERANCE_NEAR_ZERO_LENGTH) {
    throw std::invalid_argument(
        "Face normal normal_2 is parallel to edge direction");
  }
  normal_2_perp = normal_2_perp / normal_2_perp_len;

  // Angle of normal_2_perp in (normal_dir, binormal_dir) plane
  const double proj_N = CGAL::to_double(normal_2_perp * normal_dir);
  const double proj_B = CGAL::to_double(normal_2_perp * binormal_dir);

  return std::atan2(proj_B, proj_N);
}

// Create the cutter solid for a single LineString edge path.
// Finds the halfedge for the first segment, checks convexity, computes
// dihedral angle, builds the cutting profile, and sweeps it along the path.
// Throws std::invalid_argument on validation failure.
// NOLINTBEGIN(readability-function-cognitive-complexity)
auto
create_cutter_for_edge(const Surface_mesh_3 &mesh, const LineString &edge,
                       const ChamferOptions &options)
    -> std::unique_ptr<Geometry>
{
  if (edge.numPoints() < 2) {
    throw std::invalid_argument("Edge must have at least 2 points");
  }

  // Get first segment of edge for orientation and dihedral properties.
  // We assume the dihedral angle and incident faces are compatible along
  // the entire path.
  const Point_3 start_pt = edge.pointN(0).toPoint_3();
  const Point_3 end_pt   = edge.pointN(1).toPoint_3();

  // Find corresponding halfedge in mesh
  const auto halfedge_desc =
      find_halfedge(mesh, start_pt, end_pt, options.epsilon);
  if (halfedge_desc == Surface_mesh_3::null_halfedge()) {
    throw std::invalid_argument("Edge not found in solid mesh. "
                                "Ensure the edge coincides with a mesh edge.");
  }

  // Get the two incident faces
  const auto face_1 = mesh.face(halfedge_desc);
  const auto face_2 = mesh.face(mesh.opposite(halfedge_desc));

  if (face_1 == Surface_mesh_3::null_face() ||
      face_2 == Surface_mesh_3::null_face()) {
    throw std::invalid_argument(
        "Edge is not shared by two faces (boundary edge?)");
  }

  // Compute face normals using CGAL PMP
  const Vector_3 normal_1 = PMP::compute_face_normal(face_1, mesh);
  const Vector_3 normal_2 = PMP::compute_face_normal(face_2, mesh);

  // Reject concave (reflex) edges: the opposite vertex of face_1 must lie
  // on the interior side of face_2's plane (dot with normal_2 < 0).
  {
    const auto     vertex_opp = mesh.target(mesh.next(halfedge_desc));
    const Vector_3 to_opp     = mesh.point(vertex_opp) - start_pt;
    if (CGAL::to_double(to_opp * normal_2) >= 0.0) {
      throw std::invalid_argument("Edge is concave (reflex). "
                                  "Chamfer is only supported on convex edges.");
    }
  }

  // Compute dihedral angle between faces (angle between outward normals)
  const double dot_val       = CGAL::to_double(normal_1 * normal_2);
  const double alpha         = std::acos(std::clamp(dot_val, -1.0, 1.0));
  const double opening_angle = M_PI - alpha;

  // Validate opening angle range
  constexpr double MIN_OPENING = MIN_OPENING_DEG * M_PI / 180.0;
  constexpr double MAX_OPENING = MAX_OPENING_DEG * M_PI / 180.0;

  if (opening_angle < MIN_OPENING) {
    throw std::invalid_argument("Edge opening angle too small (" +
                                std::to_string(opening_angle * 180.0 / M_PI) +
                                "°). Minimum supported: 10°.");
  }
  if (opening_angle > MAX_OPENING) {
    throw std::invalid_argument("Edge opening angle too large (" +
                                std::to_string(opening_angle * 180.0 / M_PI) +
                                "°). Maximum supported: 170°.");
  }

  // Compute continuous angle of normal_2 in the local frame
  const Vector_3 edge_dir = SFCGAL::normalizeVector(end_pt - start_pt);
  const double   theta_2  = compute_n2_angle(edge_dir, normal_1, normal_2);

  // Create profile based on actual dihedral angle
  std::unique_ptr<Polygon> profile;
  if (options.type == ChamferType::ROUND) {
    profile = create_fillet_profile_for_angle(options.radius, options.segments,
                                              theta_2);
  } else {
    const double radius_y_val =
        (options.radius_y < 0) ? options.radius : options.radius_y;
    profile =
        create_chamfer_profile_for_angle(options.radius, radius_y_val, theta_2);
  }

  // Sweep profile along edge to create cutter
  SweepOptions sweep_opts;
  sweep_opts.frame_method = SweepOptions::FrameMethod::SEGMENT_ALIGNED;
  sweep_opts.closed_path  = isClosed(edge);

  // // For single-segment edges, use normal_1 as the sweep frame reference so
  // the
  // // chamfer profile legs align with the solid's face surfaces.
  // // For multi-segment paths, skip: normal_1 may be parallel to some segment,
  // // causing a degenerate reference and frame flips at corners.
  // if (edge.numPoints() == 2) {
  //   sweep_opts.reference_normal = normal_1;
  // }

  // Check whether normal_1 is safe to use as reference for all segments
  bool can_use_reference_normal = true;
  for (size_t i = 0; i + 1 < edge.numPoints(); ++i) {
    const Vector_3 seg_dir = SFCGAL::normalizeVector(
        edge.pointN(i + 1).toPoint_3() - edge.pointN(i).toPoint_3());
    const double parallel = std::abs(CGAL::to_double(seg_dir * normal_1));
    if (parallel > 1.0 - 1e-6) {
      can_use_reference_normal = false;
      SFCGAL_WARNING("Chamfer: reference normal is parallel to segment " +
                     std::to_string(i) + ", disabling reference normal");
      break;
    }
  }

  if (can_use_reference_normal) {
    sweep_opts.reference_normal = normal_1;
  }

  // TODO: works on slanted geometry, but produce invalid geometry on straight
  // one, like L-shape
  // *******************************************************************************************
  // For open paths, extend the edge slightly at both ends to ensure the cutter
  // fully covers the edge even when slanted (due to perpendicular end caps).
  // if (!sweep_opts.closed_path) {
  //   const double radius_y_val =
  //       (options.radius_y < 0) ? options.radius : options.radius_y;
  //   const double extension_length =
  //       std::max(options.radius, radius_y_val) * 2.0;
  //
  //   LineString extended_edge;
  //   extended_edge.reserve(edge.numPoints() + 2);
  //
  //   // Extend start
  //   const Vector_3 v_start = SFCGAL::normalizeVector(
  //       edge.pointN(1).toPoint_3() - edge.pointN(0).toPoint_3());
  //   const Point_3 p_start =
  //       edge.pointN(0).toPoint_3() - v_start * extension_length;
  //   extended_edge.addPoint(Point(p_start));
  //
  //   // Original points
  //   for (size_t i = 0; i < edge.numPoints(); ++i) {
  //     extended_edge.addPoint(edge.pointN(i));
  //   }
  //
  //   // Extend end
  //   const Vector_3 v_end = SFCGAL::normalizeVector(
  //       edge.pointN(edge.numPoints() - 1).toPoint_3() -
  //       edge.pointN(edge.numPoints() - 2).toPoint_3());
  //   const Point_3 p_end = edge.pointN(edge.numPoints() - 1).toPoint_3() +
  //                         v_end * extension_length;
  //   extended_edge.addPoint(Point(p_end));
  //
  //   auto cutter_surf = sweep(extended_edge, *profile, sweep_opts);
  //   return std::make_unique<Solid>(std::move(cutter_surf));
  // }
  // *******************************************************************************************

  auto cutter_surf = sweep(edge, *profile, sweep_opts);

  return std::make_unique<Solid>(std::move(cutter_surf));
}
// NOLINTEND(readability-function-cognitive-complexity)

} // anonymous namespace

/// @} end of private section

// ----------------------------------------------------------------------------
// Public API Implementation
// ----------------------------------------------------------------------------

auto
chamfer(const Geometry &solid_geom, const Geometry &edge_geom,
        const ChamferOptions &options) -> std::unique_ptr<Geometry>
{
  // Validate inputs
  if (options.radius <= 0.0) {
    throw std::invalid_argument("Chamfer radius must be positive");
  }
  if (options.type == ChamferType::ROUND && options.segments < 1) {
    throw std::invalid_argument("Fillet segments must be at least 1");
  }

  // Convert solid to Surface_mesh_3 once for all edges
  Surface_mesh_3 mesh;
  if (solid_geom.geometryTypeId() == TYPE_SOLID) {
    mesh = solid_geom.as<Solid>().toSurfaceMesh();
  } else if (solid_geom.geometryTypeId() == TYPE_POLYHEDRALSURFACE) {
    mesh = solid_geom.as<PolyhedralSurface>().toSurfaceMesh();
  } else {
    throw std::invalid_argument(
        "Input geometry must be a Solid or PolyhedralSurface");
  }

  // Create all cutters first (before modifying the solid)
  std::vector<std::unique_ptr<Geometry>> cutters;

  if (edge_geom.geometryTypeId() == TYPE_LINESTRING) {
    try {
      auto cutter =
          create_cutter_for_edge(mesh, edge_geom.as<LineString>(), options);
      cutters.push_back(std::move(cutter));
    } catch (const std::invalid_argument &e) {
      // Expected: edge not found, concave, angle out of range
      SFCGAL_WARNING(std::string("Chamfer: skipping edge - ") + e.what());
    }
  } else if (edge_geom.geometryTypeId() == TYPE_MULTILINESTRING) {
    const auto &multi = edge_geom.as<MultiLineString>();
    cutters.reserve(multi.numGeometries());
    for (size_t i = 0; i < multi.numGeometries(); ++i) {
      try {
        auto cutter = create_cutter_for_edge(
            mesh, multi.geometryN(i).as<LineString>(), options);
        cutters.push_back(std::move(cutter));
      } catch (const std::invalid_argument &e) {
        // Expected: edge not found, concave, angle out of range
        SFCGAL_WARNING(std::string("Chamfer: skipping edge - ") + e.what());
      }
    }
  } else {
    throw std::invalid_argument(
        "Edge geometry must be LineString or MultiLineString");
  }

  if (cutters.empty()) {
    return solid_geom.clone();
  }

  // Combine all cutters using a sequential union
  auto combined_cutter = std::move(cutters[0]);
  for (size_t i = 1; i < cutters.size(); ++i) {
    combined_cutter = union3D(*combined_cutter, *cutters[i]);
  }

  // Apply single difference at the end
  auto result = difference3D(solid_geom, *combined_cutter);
  return collectionHomogenize(std::move(result));
}

} // namespace SFCGAL::algorithm
