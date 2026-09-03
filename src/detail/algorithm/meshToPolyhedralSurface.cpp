// Copyright (c) 2026-2026, SFCGAL team.
// SPDX-License-Identifier: LGPL-2.0-or-later

#include "SFCGAL/detail/algorithm/meshToPolyhedralSurface.h"

#include <CGAL/Polygon_mesh_processing/compute_normal.h>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace SFCGAL::detail::algorithm {

// ----------------------------------------------------------------------------------
// -- private interface
// ----------------------------------------------------------------------------------
/// @{
/// @privatesection

using VertexIndex = Surface_mesh_3::Vertex_index;

/**
 * Checks whether two faces are coplanar within given tolerances.
 *
 *
 * @param mesh       The input surface mesh.
 * @param face1      The first face to compare.
 * @param face2      The second face to compare.
 * @param normal1    normal vector of the first face.
 * @param normal2    normal vector of the second face.
 * @param epsAngle   Maximum allowed angle (in degrees) between the two normals.
 * @param epsDist    Maximum allowed distance between vertices of a face and the
 * plane of another face to consider them coplanar. of face1.
 *
 * @return true if the two faces are considered coplanar, false otherwise.
 */
static auto
areCoplanarFaces(const Surface_mesh_3 &mesh, FaceIndex face1, FaceIndex face2,
                 const Vector_3 &normal1, const Vector_3 &normal2,
                 const Kernel::FT &epsAngle, const Kernel::FT &epsDist) -> bool
{
  // A degenerate face has a null normal, hence no plane. Merging it would go
  // unnoticed: CGAL::Plane_3(pt0, NULL_VECTOR) builds the 0 = 0 equation, for
  // which every point lies at distance zero.
  if (normal1 == CGAL::NULL_VECTOR || normal2 == CGAL::NULL_VECTOR) {
    return false;
  }

  // Test if the normals are parallels
  const Kernel::FT deg2rad = CGAL_PI / Kernel::FT(180.0);
  const Kernel::FT cosEps  = std::cos(CGAL::to_double(epsAngle * deg2rad));
  const Kernel::FT dot =
      std::clamp(normal1 * normal2, Kernel::FT(-1.0), Kernel::FT(1.0));
  if (CGAL::abs(dot) < cosEps) {
    return false;
  }

  // Test if a vertex of face2 lies in the plane of face1
  const Point_3 &pt0 = mesh.point(mesh.source(mesh.halfedge(face1)));
  const CGAL::Plane_3<Kernel> plane(pt0, normal1);
  auto halfedges = CGAL::halfedges_around_face(mesh.halfedge(face2), mesh);
  return std::all_of(
      halfedges.begin(), halfedges.end(), [&](auto halfedge) -> auto {
        const Point_3   &pt         = mesh.point(mesh.source(halfedge));
        const Kernel::FT signedDist = plane.a() * pt.x() + plane.b() * pt.y() +
                                      plane.c() * pt.z() + plane.d();
        return signedDist * signedDist <= epsDist * epsDist;
      });
}

/// @} end of private section

auto
groupCoplanarFaces(const Surface_mesh_3 &mesh, const Kernel::FT &epsAngle,
                   const Kernel::FT &epsDist)
    -> std::vector<std::vector<FaceIndex>>
{
  std::vector<bool>     visited(mesh.num_faces(), false);
  std::vector<Vector_3> normals(mesh.num_faces(), CGAL::NULL_VECTOR);

  for (FaceIndex face : mesh.faces()) {
    normals[face] = PMP::compute_face_normal(face, mesh);
  }

  std::vector<std::vector<FaceIndex>> result;

  // Iterate over all faces to look for coplanar components
  for (const auto &seedFace : mesh.faces()) {
    if (visited[seedFace]) {
      continue;
    }

    std::queue<FaceIndex> queue;
    queue.push(seedFace);
    visited[seedFace] = true;
    std::vector<FaceIndex> coplanarGroup;

    // look for connected coplanar faces
    while (!queue.empty()) {
      FaceIndex currentFace = queue.front();
      queue.pop();
      coplanarGroup.emplace_back(currentFace);

      // Iterate over all neighboring faces
      for (const auto &halfedge :
           CGAL::halfedges_around_face(mesh.halfedge(currentFace), mesh)) {
        FaceIndex oppositeFace = mesh.face(mesh.opposite(halfedge));
        if (oppositeFace == Surface_mesh_3::null_face() ||
            visited[oppositeFace]) {
          continue;
        }

        if (areCoplanarFaces(mesh, seedFace, oppositeFace, normals[seedFace],
                             normals[oppositeFace], epsAngle, epsDist)) {
          queue.push(oppositeFace);
          visited[oppositeFace] = true;
        }
      }
    }

    result.push_back(std::move(coplanarGroup));
  }

  return result;
}

auto
createRing(const Surface_mesh_3                            &mesh,
           const CGAL::Face_filtered_graph<Surface_mesh_3> &filteredMesh,
           const HalfedgeIndex &ringIdx, const Kernel::Vector_3 &faceNormal,
           LineString &ring) -> Kernel::FT
{
  for (const VertexIndex vertex : vertices_around_face(ringIdx, filteredMesh)) {
    ring.addPoint(Point(mesh.point(vertex)));
  }

  Vector_3          sum        = CGAL::NULL_VECTOR;
  const std::size_t nrPts      = ring.numPoints();
  const Point_3     firstPoint = ring.startPoint().toPoint_3();
  for (std::size_t i = 0; i < nrPts; ++i) {
    sum += CGAL::cross_product(ring.pointN(i).toPoint_3() - firstPoint,
                               ring.pointN((i + 1) % nrPts).toPoint_3() -
                                   firstPoint);
  }
  ring.closes();

  return sum * faceNormal;
}

} // namespace SFCGAL::detail::algorithm
