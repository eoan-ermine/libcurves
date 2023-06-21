#pragma once

#include <string>

#include <fmt/core.h>

#include "curves.hpp"

/**
 * @brief Return the name of this header-only library
 */
inline auto name() -> std::string
{
  return fmt::format("{}", "libcurves");
}
