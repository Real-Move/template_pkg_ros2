/*
 * Copyright (c) 2024 Real-Move. All rights reserved.
 * Proprietary and confidential.
 * See LICENSE for full terms.
 */

#include <gtest/gtest.h>

#include "template_pkg_ros2/minimal_cpp_node.hpp"

TEST(test_minimal_cpp_node, add_two_ints_returns_expected_sum)
{
    MinimalNode node;
    int sum = 0;

    ASSERT_TRUE(node.addTwoInts(2, 2, sum));
    ASSERT_EQ(4, sum);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
