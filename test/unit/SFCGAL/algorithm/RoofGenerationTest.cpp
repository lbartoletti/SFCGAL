// Copyright (c) 2026-2026, SFCGAL team.
// SPDX-License-Identifier: LGPL-2.0-or-later

#include <boost/test/unit_test.hpp>

#include "SFCGAL/Geometry.h"
#include "SFCGAL/Polygon.h"
#include "SFCGAL/Validity.h"
#include "SFCGAL/algorithm/isValid.h"
#include "SFCGAL/algorithm/roofGeneration.h"
#include "SFCGAL/io/wkt.h"

using namespace SFCGAL;
using namespace boost::unit_test;

BOOST_AUTO_TEST_SUITE(SFCGAL_algorithm_RoofGenerationTest)

/**
 * A skillion roof is clipped by a plane, and the resulting mesh is turned back
 * into a PolyhedralSurface by merging its coplanar faces. That merge used to
 * accept faces 1e-8 apart while isValid() refuses a patch whose vertices leave
 * the plane by more than 1e-9, so it built patches that were rejected right
 * away. Each footprint below has at least one primary edge that produced such
 * an invalid roof.
 */
BOOST_AUTO_TEST_CASE(testSkillionRoofIsValid)
{
  const std::string footprints[] = {
      "POLYGON ((0 0,10 0,10 4,4 4,4 10,0 10,0 0))", // L shape
      "POLYGON ((0 0,3 0,3 3,6 3,6 6,0 6,0 0))",     // mirrored L shape
      "POLYGON ((0 0,5 0,7 3,3 6,0 4,0 0))",         // convex pentagon
      // Issue https://gitlab.com/sfcgal/SFCGAL/-/work_items/328
      "Polygon ((0 0,5 0,6 7,0 5,0 0))",
      "Polygon ((165302.82031830900814384 5818491.14396010525524616, 165286.5676726532401517 5818495.23191135190427303, 165286.66786019498249516 5818495.87568337470293045, 165283.32827547111082822 5818496.88962440565228462, 165283.79581733245868236 5818499.31986449100077152, 165280.87924667366314679 5818500.25333437137305737, 165277.75116898241685703 5818501.26727590337395668, 165278.53040541795780882 5818503.97112056240439415, 165303.92238126794109121 5818498.5312434621155262, 165302.82031830900814384 5818491.14396010525524616))",
  };

  for (const std::string &wkt : footprints) {
    std::unique_ptr<Geometry> const geometry(io::readWkt(wkt));
    const Polygon                  &footprint = geometry->as<Polygon>();

    for (std::size_t edge = 0; edge < footprint.exteriorRing().numSegments();
         ++edge) {
      algorithm::RoofParameters parameters;
      parameters.type             = algorithm::RoofType::SKILLION;
      parameters.roofHeight       = 3.0;
      parameters.slopeAngle       = 30.0;
      parameters.primaryEdgeIndex = edge;

      std::unique_ptr<Geometry> const roof =
          algorithm::generateRoof(footprint, parameters);

      Validity const validity = algorithm::isValid(*roof);
      BOOST_CHECK_MESSAGE(validity, "skillion roof on "
                                        << wkt << ", primary edge " << edge
                                        << ": " << validity.reason());
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
