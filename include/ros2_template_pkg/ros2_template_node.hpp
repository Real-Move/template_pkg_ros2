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