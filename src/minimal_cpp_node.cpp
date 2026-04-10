/*
 * Copyright (c) 2024 Real-Move. All rights reserved.
 * Proprietary and confidential.
 * See LICENSE for full terms.
 */

#include <template_pkg_ros2/minimal_cpp_node.hpp>

MinimalNode::MinimalNode() {}

MinimalNode::~MinimalNode() {}

void MinimalNode::print(std::string message)
{
    std::cout << message;
}

bool MinimalNode::addTwoInts(int a, int b, int &sum)
{
    sum = a + b;
    return true;
}

#ifndef TEMPLATE_PKG_ROS2_DISABLE_MINIMAL_CPP_NODE_MAIN
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    MinimalNode minimalNode;
    minimalNode.print("Hello from template_pkg_ros2 minimal C++ node.");

    return 0;
}
#endif
