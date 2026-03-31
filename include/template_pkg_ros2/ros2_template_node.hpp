/*
 * Copyright (c) 2024 Real-Move. All rights reserved.
 * Proprietary and confidential.
 * See LICENSE for full terms.
 */

#ifndef TEMPLATE_PKG_ROS2__ROS2_TEMPLATE_NODE_HPP_
#define TEMPLATE_PKG_ROS2__ROS2_TEMPLATE_NODE_HPP_

#include <iostream>
#include <string>

//!  A template class for a ros2 node
/*!
  This is a template for ros2 nodes. Not to be mistaken for c++ template
*/
class RosTemplateNode
{
  public:
    RosTemplateNode();

    ~RosTemplateNode();

    void print(std::string message);

    bool addTwoInts(int a, int b, int &sum);
};

#endif // TEMPLATE_PKG_ROS2__ROS2_TEMPLATE_NODE_HPP_
