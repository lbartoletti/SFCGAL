// Copyright (c) 2012-2013, IGN France.
// Copyright (c) 2012-2024, Oslandia.
// Copyright (c) 2024-2026, SFCGAL team.
// SPDX-License-Identifier: LGPL-2.0-or-later

#ifndef SFCGAL_NUMERIC_H_
#define SFCGAL_NUMERIC_H_

#include <cmath>
#include <concepts>
#include <limits>

#include "SFCGAL/export.h"

#include "SFCGAL/Kernel.h"

namespace SFCGAL {

/// @brief Default epsilon value for floating point comparisons
constexpr double EPSILON = 1e-8;

/// @brief Absolute tolerance used to validate a geometry (isValid, isSimple,
/// isPlane3D). A point further than this from the plane of its polygon makes
/// that polygon invalid.
constexpr double EPSILON_VALIDITY = 1e-9;

/// @brief Absolute tolerance used to merge coplanar faces into a single patch.
/// Must stay strictly below EPSILON_VALIDITY.
constexpr double EPSILON_COPLANARITY = 1e-10;

static_assert(EPSILON_COPLANARITY < EPSILON_VALIDITY,
              "merging coplanar faces must be stricter than validating them, "
              "otherwise isValid() rejects the merged geometry");

#if defined(__clang__)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wfloat-equal"
#elif defined(__GCC__)
  #pragma gcc diagnostic push
  #pragma gcc diagnostic ignored "-Wfloat-equal"
#endif

/// @private
template <typename T>
concept NumericValue =
    std::same_as<T, double> || std::same_as<T, CGAL::Epeck::FT>;

/**
 * @brief Check if two values are almost equal within tolerance
 * @param first First value to compare
 * @param second Second value to compare
 * @param absoluteTolerance Tolerance for comparison
 * @return true if values are almost equal, false otherwise
 */
template <NumericValue T>
inline auto
almostEqualAbsolute(const T first, const T second, const T absoluteTolerance)
    -> bool
{
  // shortcut and handles inf values
  if (first == second) {
    return true;
  }

  if constexpr (std::is_same_v<T, double>) {
    if (std::isnan(first) || std::isnan(second)) {
      return std::isnan(first) && std::isnan(second);
    }
  }

  // enable ADL:
  // - std::abs for double
  // - CGAL::abs for Kernel::FT
  using std::abs;

  return abs(first - second) <= absoluteTolerance;
}

/**
 * @brief Check if two values are almost equal within tolerance
 * @param first First value to compare
 * @param second Second value to compare
 * @param relativeTolerance Tolerance for comparison
 * @return true if values are almost equal, false otherwise
 */
template <NumericValue T>
inline auto
almostEqualRelative(const T first, const T second, const T relativeTolerance)
    -> bool
{
  // shortcut and handles inf values
  if (first == second) {
    return true;
  }

  if constexpr (std::is_same_v<T, double>) {
    if (std::isnan(first) || std::isnan(second)) {
      return std::isnan(first) && std::isnan(second);
    }
  }

  // enable ADL:
  // - std::abs for double
  // - CGAL::abs for Kernel::FT
  using std::abs;

  return abs(first - second) <=
         relativeTolerance * std::max(abs(first), abs(second));
}

#if defined(__clang__)
  #pragma clang diagnostic pop
#elif defined(__GCC__)
  #pragma gcc diagnostic pop
#endif

/**
 * @brief shortcut to get NaN for double
 * @return NaN (Not a Number) value for double
 */
inline auto
NaN() -> double
{
  return std::numeric_limits<double>::quiet_NaN();
}

/**
 * @brief round a double to the nearest integer
 * @param value Value to round
 * @return Rounded value
 */
inline auto
round(const double &value) -> double
{
  if (value < 0.0) {
    return ::ceil(value - 0.5);
  } else {
    return ::floor(value + 0.5);
  }
}

#ifdef CGAL_USE_GMPXX
/**
 * @brief floor a rational to an integer
 * @param value Rational value to floor
 * @return Floor of the rational as integer
 */
SFCGAL_API auto
floor(const ::mpq_class &value) -> ::mpz_class;
/**
 * @brief ceil a rational to an integer
 * @param value Rational value to ceil
 * @return Ceiling of the rational as integer
 */
SFCGAL_API auto
ceil(const ::mpq_class &value) -> ::mpz_class;
/**
 * @brief round a rational to an integer
 * @param value Rational value to round
 * @return Rounded rational as integer
 */
SFCGAL_API auto
round(const ::mpq_class &value) -> ::mpz_class;
#endif

/**
 * @brief floor a rational to an integer
 * @param value CGAL rational value to floor
 * @return Floor of the rational as CGAL integer
 */
SFCGAL_API auto
floor(const CGAL::Gmpq &value) -> CGAL::Gmpz;
/**
 * @brief ceil a rational to an integer
 * @param value CGAL rational value to ceil
 * @return Ceiling of the rational as CGAL integer
 */
SFCGAL_API auto
ceil(const CGAL::Gmpq &value) -> CGAL::Gmpz;
/**
 * @brief round a rational to an integer
 * @param value CGAL rational value to round
 * @return Rounded rational as CGAL integer
 */
SFCGAL_API auto
round(const CGAL::Gmpq &value) -> CGAL::Gmpz;

/**
 * @brief Normalizes a vector
 * @param vector The vector to normalize
 * @return The normalized vector
 */
inline auto
normalizeVector(const Kernel::Vector_3 &vector) -> Kernel::Vector_3
{
  Kernel::FT length = CGAL::sqrt(CGAL::to_double(vector.squared_length()));
  // clang-tidy wrongly assumes that vector / length might leak
  return (length > 0) ? vector / length : vector;
} // NOLINT(clang-analyzer-cplusplus.NewDeleteLeaks)
} // namespace SFCGAL

#endif
