// Copyright (c) 2026-2026, SFCGAL team.
// SPDX-License-Identifier: LGPL-2.0-or-later

#ifndef SFCGAL_DETAIL_ALGORITHM_MESH_TO_POLYHEDRAL_SURFACE_H_
#define SFCGAL_DETAIL_ALGORITHM_MESH_TO_POLYHEDRAL_SURFACE_H_

#include "SFCGAL/Kernel.h"
#include "SFCGAL/LineString.h"
#include "SFCGAL/numeric.h"

#include <CGAL/boost/graph/Face_filtered_graph.h>

namespace SFCGAL::detail::algorithm {

// ----------------------------------------------------------------------------------
// -- private interface
// ----------------------------------------------------------------------------------
/// @{
/// @privatesection

using FaceIndex     = Surface_mesh_3::Face_index;
using HalfedgeIndex = Surface_mesh_3::Halfedge_index;

/// @} end of private section

/**
 * Groups mesh faces into connected coplanar components.
 *
 * This function partitions the faces of a surface mesh into groups of connected
 * faces that are coplanar within specified angular and distance tolerances.
 * Connectivity is defined via shared edges (adjacent faces).
 *
 * @param mesh       The input surface mesh.
 * @param epsAngle   Maximum allowed angle (in degrees) between the two normals.
 * @param epsDist    Maximum allowed distance between vertices of a face and the
 * plane of another face to consider them coplanar. Keep it below
 * EPSILON_VALIDITY: a looser value groups faces that isValid() then rejects.
 *
 * @return A vector of face groups, where each group is a vector of face indices
 *         representing a connected coplanar component.
 */
auto
groupCoplanarFaces(const Surface_mesh_3 &mesh,
                   const Kernel::FT     &epsAngle = Kernel::FT(0.5),
                   const Kernel::FT &epsDist = Kernel::FT(EPSILON_COPLANARITY))
    -> std::vector<std::vector<FaceIndex>>;

/**
 * Builds a polygonal ring from a face and computes its oriented area.
 *
 * This function extracts the vertices of a face from a filtered mesh and
 * constructs a corresponding ring (closed polyline). It then its signed area.
 *
 * @param mesh          The original surface mesh containing vertex geometry.
 * @param filteredMesh  A face-filtered graph providing access to the target
 * face.
 * @param ringIdx       The halfedge index identifying the face to process.
 * @param faceNormal    The normal vector of the face, used for orientation.
 * @param ring          Output LineString that will be filled with the face
 * vertices in order and closed at the end.
 *
 * @return Kernel::FT   the signed area of the ring projected onto the normal
 * direction.
 */
auto
createRing(const Surface_mesh_3                            &mesh,
           const CGAL::Face_filtered_graph<Surface_mesh_3> &filteredMesh,
           const HalfedgeIndex &ringIdx, const Kernel::Vector_3 &faceNormal,
           LineString &ring) -> Kernel::FT;

} // namespace SFCGAL::detail::algorithm

#endif // SFCGAL_DETAIL_ALGORITHM_MESH_TO_POLYHEDRAL_SURFACE_H_
