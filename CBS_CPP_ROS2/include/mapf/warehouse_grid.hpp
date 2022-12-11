#pragma once

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "builtin_interfaces/msg/duration.hpp"

#include "mapf/LowLevelSolver.h"
#include "mapf/HighLevelSolver.h"


const int cell_occupied = 100;
const int cell_size = 1;

namespace warehouse {


    class WarehouseGrid
    {
        public:
            WarehouseGrid();

            struct Point{
                int x;
                int y;
                Point(int a, int b) : x(a), y(b){ }
            };

            void build_warehouse_map(
                const std::string frame_id,
                const rclcpp::Clock::SharedPtr time_stamp,
                const double & res,
                const int & width,
                const int & height
            );

            void add_obstacle(
                const Point point_1,
                const Point point_2
            );

            void build_obstacles();

            void set_obstacle_cell(const Point point);

            std::vector<std::vector<Cell>> compute_paths();

            //visualise meshes

            visualization_msgs::msg::MarkerArray visualise_obstacles(
                const rclcpp::Clock::SharedPtr time_stamp
                );

            visualization_msgs::msg::Marker visualise_kiva1(
                const rclcpp::Clock::SharedPtr time_stamp,
                const int pos_x, const int pos_y, 
                const float r, const float g, const float b
                );

            visualization_msgs::msg::Marker visualise_kiva2(
                const rclcpp::Clock::SharedPtr time_stamp,
                const int pos_x, const int pos_y, 
                const float r, const float g, const float b
                );

            visualization_msgs::msg::Marker visualise_kiva2_goal(
                const rclcpp::Clock::SharedPtr time_stamp,
                const int pos_x, const int pos_y, 
                const float r, const float g, const float b
                );

            visualization_msgs::msg::Marker visualise_kiva1_goal(
                const rclcpp::Clock::SharedPtr time_stamp,
                const int pos_x, const int pos_y, 
                const float r, const float g, const float b
                );


            

            nav_msgs::msg::OccupancyGrid map_;

            std::vector<geometry_msgs::msg::Point> obstacle_list;

            std::vector<float> obstacle_x_viz_;
            std::vector<float> obstacle_y_viz_;



            double map_resolution_{0.04};
            unsigned int rows_{10};
            unsigned int cols_{10};

            int agentID_{0};

            Map map; // Grid map
            std::vector<std::vector<Cell>> optimalPaths_;
            std::vector<Agent> agents_;
            std::vector<std::vector<Cell>> cells_;



        private:


            // Marker Lifetime
            builtin_interfaces::msg::Duration marker_lifetime_;



    };

}//namespace warehouse