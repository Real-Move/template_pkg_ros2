/*
 * Copyright (c) 2024 Real-Move. All rights reserved.
 * Proprietary and confidential.
 * See LICENSE for full terms.
 */

#ifndef TEMPLATE_PKG_ROS2__MINIMAL_CPP_NODE_HPP_
#define TEMPLATE_PKG_ROS2__MINIMAL_CPP_NODE_HPP_

#include <iostream>
#include <string>

//!  A minimal helper class for a ROS 2 starter node
/*!
  This is a minimal placeholder for C++ ROS 2 node logic.
*/
class MinimalNode
{
  public:
    MinimalNode();

    ~MinimalNode();

    void print(std::string message);

    bool addTwoInts(int a, int b, int& sum);
};

#endif // TEMPLATE_PKG_ROS2__MINIMAL_CPP_NODE_HPP_
