// Copyright (c) 2025-2025, SFCGAL team.
// SPDX-License-Identifier: LGPL-2.0-or-later

#ifndef SFCGAL_ALGORITHM_MESHREPAIR_H_
#define SFCGAL_ALGORITHM_MESHREPAIR_H_

#include "SFCGAL/PolyhedralSurface.h"
#include "SFCGAL/TriangulatedSurface.h"
#include "SFCGAL/config.h"

#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Surface_mesh.h>

namespace SFCGAL::algorithm {

/**
 * @brief Result of mesh repair operation
 */
struct MeshRepairResult {
  bool        success = false;
  std::string message;
  int         facesRemoved = 0;
  int         verticesMerged = 0;
  bool        orientationFixed = false;

  MeshRepairResult() = default;
  MeshRepairResult(bool s, std::string m) : success(s), message(std::move(m)) {}
};

/**
 * @brief Make a PolyhedralSurface valid using CGAL mesh repair functions
 *
 * This function attempts to repair common mesh issues:
 * - Remove duplicate vertices and faces
 * - Fix face orientation
 * - Stitch border edges to close gaps
 * - Triangulate non-triangular faces if needed
 * - Remove degenerate elements
 *
 * @param surface The polyhedral surface to repair
 * @param tolerance Tolerance for vertex merging (default: 1e-10)
 * @param triangulate Whether to triangulate all faces (default: false)
 * @return MeshRepairResult with details of repairs performed
 */
SFCGAL_API auto
makeValid(PolyhedralSurface &surface, double tolerance = 1e-10,
          bool triangulate = false) -> MeshRepairResult;

/**
 * @brief Make a TriangulatedSurface valid using CGAL mesh repair functions
 *
 * @param surface The triangulated surface to repair
 * @param tolerance Tolerance for vertex merging (default: 1e-10)
 * @return MeshRepairResult with details of repairs performed
 */
SFCGAL_API auto
makeValid(TriangulatedSurface &surface, double tolerance = 1e-10)
    -> MeshRepairResult;

/**
 * @brief Create a valid PolyhedralSurface from an existing one
 *
 * Non-destructive version that returns a new valid surface.
 *
 * @param surface Input surface
 * @param tolerance Tolerance for vertex merging
 * @param triangulate Whether to triangulate all faces
 * @return Pair of (repaired_surface, repair_result)
 */
SFCGAL_API auto
createValidSurface(const PolyhedralSurface &surface, double tolerance = 1e-10,
                   bool triangulate = false)
    -> std::pair<std::unique_ptr<PolyhedralSurface>, MeshRepairResult>;

/**
 * @brief Properly connect roof to building walls
 *
 * This function ensures that roof edges are properly connected to the top
 * edges of building walls, creating a watertight solid.
 *
 * @param building The building walls (extruded footprint)
 * @param roof The roof surface
 * @param tolerance Tolerance for edge matching
 * @return A valid PolyhedralSurface representing the complete building
 */
SFCGAL_API auto
connectRoofToBuilding(const PolyhedralSurface &building,
                      const PolyhedralSurface &roof, double tolerance = 1e-10)
    -> std::unique_ptr<PolyhedralSurface>;

/**
 * @brief Ensure proper triangle/quadrilateral orientation in roof+building
 *
 * Fixes face orientation issues that commonly occur when combining
 * separately generated roof and building meshes.
 *
 * @param surface The combined roof+building surface
 * @return MeshRepairResult indicating success and changes made
 */
SFCGAL_API auto
fixRoofBuildingOrientation(PolyhedralSurface &surface) -> MeshRepairResult;

/**
 * @brief Advanced mesh repair using CGAL's polygon mesh processing
 *
 * Performs comprehensive mesh repair including:
 * - Duplicate removal
 * - Border stitching
 * - Orientation fixing
 * - Hole filling (optional)
 * - Self-intersection removal (optional)
 *
 * @param surface Surface to repair
 * @param fillHoles Whether to attempt filling holes
 * @param removeSelfIntersections Whether to remove self-intersections (expensive)
 * @param tolerance Geometric tolerance
 * @return MeshRepairResult with detailed repair information
 */
SFCGAL_API auto
advancedMeshRepair(PolyhedralSurface &surface, bool fillHoles = true,
                   bool removeSelfIntersections = false, double tolerance = 1e-10)
    -> MeshRepairResult;

} // namespace SFCGAL::algorithm

#endif // SFCGAL_ALGORITHM_MESHREPAIR_H_