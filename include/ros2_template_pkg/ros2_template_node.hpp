/*
 * MIT License
 *
 * Copyright (c) 2024 Real-Move
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef ROS2_TEMPLATE_PKG__ROS2_TEMPLATE_NODE_HPP_
#define ROS2_TEMPLATE_PKG__ROS2_TEMPLATE_NODE_HPP_

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

  bool addTwoInts(int a, int b, int & sum);
};

#endif  // ROS2_TEMPLATE_PKG__ROS2_TEMPLATE_NODE_HPP_
