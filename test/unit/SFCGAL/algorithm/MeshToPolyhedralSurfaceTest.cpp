// Copyright (c) 2026-2026, SFCGAL team.
// SPDX-License-Identifier: LGPL-2.0-or-later

#include <boost/test/unit_test.hpp>

#include "SFCGAL/Kernel.h"
#include "SFCGAL/PolyhedralSurface.h"
#include "SFCGAL/algorithm/isValid.h"
#include "SFCGAL/algorithm/meshToPolyhedralSurface.h"
#include "SFCGAL/numeric.h"

using namespace SFCGAL;
using namespace boost::unit_test;

namespace {

using Point_3     = Kernel::Point_3;
using VertexIndex = Surface_mesh_3::Vertex_index;

/// Out of plane offset large enough for the default criterion to refuse the
/// merge, expressed from EPSILON_COPLANARITY so that it follows it.
constexpr double OUT_OF_PLANE_OFFSET = 10 * EPSILON_COPLANARITY;

/**
 * The unit square in the tilted plane z = x / 2, split in two triangles along
 * the diagonal (0 0) - (1 1). The corner (1 0) belongs to one triangle only:
 * pushing it @p deltaZ out of the plane makes the two triangles non coplanar
 * by roughly 0.9 * deltaZ.
 */
auto
tiltedSquare(double deltaZ) -> Surface_mesh_3
{
  Surface_mesh_3 mesh;
  VertexIndex    v00 = mesh.add_vertex(Point_3(0, 0, 0));
  VertexIndex    v10 = mesh.add_vertex(Point_3(1, 0, 0.5 + deltaZ));
  VertexIndex    v11 = mesh.add_vertex(Point_3(1, 1, 0.5));
  VertexIndex    v01 = mesh.add_vertex(Point_3(0, 1, 0));

  mesh.add_face(v00, v10, v11);
  mesh.add_face(v00, v11, v01);
  return mesh;
}

} // namespace

BOOST_AUTO_TEST_SUITE(SFCGAL_algorithm_MeshToPolyhedralSurfaceTest)

/// Two exactly coplanar triangles become a single valid patch.
BOOST_AUTO_TEST_CASE(testCoplanarTrianglesAreMerged)
{
  const Surface_mesh_3 mesh = tiltedSquare(0.0);

  std::unique_ptr<PolyhedralSurface> const surface =
      algorithm::meshToPolyhedralSurface(mesh);

  BOOST_CHECK_EQUAL(surface->numPatches(), 1U);
  BOOST_CHECK(algorithm::isValid(*surface));
}

/// Triangles that miss coplanarity by more than EPSILON_COPLANARITY must stay
/// apart, otherwise the merged patch is not planar enough for isValid().
BOOST_AUTO_TEST_CASE(testNearlyCoplanarTrianglesAreNotMerged)
{
  const Surface_mesh_3 mesh = tiltedSquare(OUT_OF_PLANE_OFFSET);

  std::unique_ptr<PolyhedralSurface> const surface =
      algorithm::meshToPolyhedralSurface(mesh);

  BOOST_CHECK_EQUAL(surface->numPatches(), 2U);
  BOOST_CHECK(algorithm::isValid(*surface));
}

/// epsDist must remain able to loosen the criterion, not only to tighten it.
BOOST_AUTO_TEST_CASE(testEpsDistIsHonoured)
{
  const Surface_mesh_3 mesh = tiltedSquare(OUT_OF_PLANE_OFFSET);

  std::unique_ptr<PolyhedralSurface> const strict =
      algorithm::meshToPolyhedralSurface(mesh, Kernel::FT(0.5),
                                         Kernel::FT(EPSILON_COPLANARITY));
  BOOST_CHECK_EQUAL(strict->numPatches(), 2U);

  std::unique_ptr<PolyhedralSurface> const loose =
      algorithm::meshToPolyhedralSurface(mesh, Kernel::FT(0.5),
                                         Kernel::FT(100 * OUT_OF_PLANE_OFFSET));
  BOOST_CHECK_EQUAL(loose->numPatches(), 1U);
}

BOOST_AUTO_TEST_SUITE_END()
