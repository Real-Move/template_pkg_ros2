/*
 * Copyright (c) 2024 Real-Move. All rights reserved.
 * Proprietary and confidential.
 * See LICENSE for full terms.
 */

#include <template_pkg_ros2/ros2_template_node.hpp>

RosTemplateNode::RosTemplateNode() {}

RosTemplateNode::~RosTemplateNode() {}

void RosTemplateNode::print(std::string message)
{
    std::cout << message;
}

bool RosTemplateNode::addTwoInts(int a, int b, int &sum)
{
    sum = a + b;

    return true;
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    RosTemplateNode rosNode;
    rosNode.print("Hello from template_pkg_ros2 cpp node.");

    return 0;
}
