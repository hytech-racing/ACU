#include <gtest/gtest.h>

TEST(UnitBeingTested, specific_unit_test)
{
    uint32_t a = 12;
    uint32_t b = 10;
    ASSERT_EQ(a - 2, b);
}
