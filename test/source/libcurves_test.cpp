#include "libcurves/libcurves.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Name is libcurves", "[library]")
{
  REQUIRE(name() == "libcurves");
}
