// Copyright (c) 2026-2026, SFCGAL team.
// SPDX-License-Identifier: LGPL-2.0-or-later

#ifndef SFCGAL_ALGORITHM_MESH_TO_POLYHEDRAL_SURFACE_H_
#define SFCGAL_ALGORITHM_MESH_TO_POLYHEDRAL_SURFACE_H_

#include "SFCGAL/Kernel.h"
#include "SFCGAL/PolyhedralSurface.h"
#include "SFCGAL/numeric.h"

#include <memory>

namespace SFCGAL::algorithm {

using VertexIndex = Surface_mesh_3::Vertex_index;

/**
 * @brief Converts a surface mesh into a polyhedral surface by grouping coplanar
 * faces.
 *
 * This function processes the input mesh and groups adjacent faces that are
 * coplanar within given angular and distance tolerances. Each group of coplanar
 * faces is then converted into a single patch of the resulting polyhedral
 * surface. During patch reconstruction, interior rings (holes) are detected and
 * preserved.
 *
 * @param mesh      The input surface mesh.
 * @param epsAngle  Maximum allowed angle (in degrees) between the two normals.
 * @param epsDist   Maximum allowed distance between vertices of a face and the
 * plane of another face to consider them coplanar. Keep it below
 * EPSILON_VALIDITY: a looser value merges faces into patches that isValid()
 * then rejects as non planar.
 *
 * @return A unique pointer to a PolyhedralSurface where each patch corresponds
 *         to a connected coplanar region of the input mesh.
 *
 * @since 2.3
 */
SFCGAL_API
auto
meshToPolyhedralSurface(
    const Surface_mesh_3 &mesh, const Kernel::FT &epsAngle = Kernel::FT(0.5),
    const Kernel::FT &epsDist = Kernel::FT(EPSILON_COPLANARITY))
    -> std::unique_ptr<PolyhedralSurface>;

} // namespace SFCGAL::algorithm

#endif // SFCGAL_ALGORITHM_MESH_TO_POLYHEDRAL_SURFACE_H_
