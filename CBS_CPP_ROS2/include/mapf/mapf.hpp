#pragma once

#include "rclcpp/rclcpp.hpp"
#include "mapf/warehouse_grid.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "mapf/LowLevelSolver.h"
#include "mapf/HighLevelSolver.h"

#include <std_srvs/srv/set_bool.hpp>


namespace warehouse{
    
    class Mapf: public rclcpp::Node
    {
        public:
            Mapf();
            void plan();
            void cbs_init();

             void compute_paths(
                const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response);

        private:
            WarehouseGrid wh_grid_;

            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr amazon_kiva_pub_1;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr amazon_kiva_pub_2;
            
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr amazon_kiva_1_goal;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr amazon_kiva_2_goal;

            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr wh_grid_pub_;

            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr call_cbs_;



            double map_resolution_{0.0016}; //0.016
            unsigned int rows_{10};
            unsigned int cols_{10};

            Map map; // Grid map
            std::vector<std::vector<Cell>> optimalPaths_;
            std::vector<Agent> agents_;
            std::vector<std::vector<Cell>> cells_;

        



         


    };


}//namespace warehouse
