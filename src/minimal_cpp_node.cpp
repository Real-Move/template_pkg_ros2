/*
 * Copyright (c) 2024 Real-Move. All rights reserved.
 * Proprietary and confidential.
 * See LICENSE for full terms.
 */

#include "template_pkg_ros2/minimal_cpp_node.hpp"

MinimalNode::MinimalNode() {}

MinimalNode::~MinimalNode() {}

void MinimalNode::print(std::string message)
{
    std::cout << message;
}

bool MinimalNode::addTwoInts(int a, int b, int& sum)
{
    sum = a + b;
    return true;
}

#ifndef TEMPLATE_PKG_ROS2_DISABLE_MINIMAL_CPP_NODE_MAIN
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

int main(int argc, char** argv)
{
    try
    {
        const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
        int my_int = 10;
        std::string my_topic = "hello";

        for (std::size_t i = 1; i < args.size(); ++i)
        {
            const auto& option = args[i];
            if (option != "--my_int" && option != "--my_topic")
                throw std::invalid_argument("Unknown argument: " + option);
            if (++i == args.size())
                throw std::invalid_argument("Missing value for " + option);

            if (option == "--my_int")
            {
                std::size_t consumed = 0;
                my_int = std::stoi(args[i], &consumed);
                if (consumed != args[i].size())
                    throw std::invalid_argument("--my_int requires an integer");
            }
            else
            {
                my_topic = args[i];
            }
        }

        auto node = std::make_shared<rclcpp::Node>("minimal_cpp_node");
        // Custom arguments supply defaults; ROS parameter overrides take precedence.
        const auto configured_int = node->declare_parameter<int>("my_int", my_int);
        const auto configured_topic = node->declare_parameter<std::string>("my_topic", my_topic);
        RCLCPP_INFO_STREAM(node->get_logger(),
                           "my_int=" << configured_int << ", my_topic=" << configured_topic);
    }
    catch (const std::exception& error)
    {
        std::cerr << "minimal_cpp_node: " << error.what() << std::endl;
        if (rclcpp::ok())
            rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
#endif
