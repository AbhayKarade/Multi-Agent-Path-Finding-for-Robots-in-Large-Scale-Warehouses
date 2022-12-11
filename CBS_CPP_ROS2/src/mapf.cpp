
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "mapf/mapf.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mapf/LowLevelSolver.h"
#include "mapf/HighLevelSolver.h"



using namespace std::chrono_literals;
using namespace std::placeholders;


namespace warehouse{

    Mapf::Mapf() :Node("mapf_cbs")
    {
        this->get_parameter("mapf.rows", rows_);
        this->get_parameter("mapf.cols", cols_);
        this->get_parameter("mapf.map_resolution", map_resolution_);

        RCLCPP_INFO(this->get_logger(), "Initialising warehouse environment %d X %d ", rows_, cols_);

        wh_grid_.build_warehouse_map(std::string("map"),
            this->get_clock(),
            map_resolution_,
            rows_,
            cols_ 
        );

        wh_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("warehouse_map", 10);

        call_cbs_ =  this->create_service<std_srvs::srv::SetBool>("Call_CBS",std::bind(&Mapf::compute_paths, this, _1, _2));

        // visualization publisher
        marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles", 10);


        amazon_kiva_pub_1 = this->create_publisher<visualization_msgs::msg::Marker>("amazon_kiva_one", 10);
        amazon_kiva_pub_2 = this->create_publisher<visualization_msgs::msg::Marker>("amazon_kiva_two", 10);

        amazon_kiva_1_goal = this->create_publisher<visualization_msgs::msg::Marker>("amazon_kiva_goal_one", 10);
        amazon_kiva_2_goal = this->create_publisher<visualization_msgs::msg::Marker>("amazon_kiva_goal_two", 10);


        

   


    }

    void Mapf::compute_paths(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {   

        wh_grid_.build_obstacles();
        wh_grid_pub_->publish(wh_grid_.map_);


        RCLCPP_INFO(this->get_logger(), "Map height %d ",wh_grid_.map_.info.height);



        marker_array_publisher_->publish(wh_grid_.visualise_obstacles(this->get_clock()));

        amazon_kiva_pub_1->publish( wh_grid_.visualise_kiva1(this->get_clock(), 2, 2, 0.0, 1.0, 0.0) ); 
        amazon_kiva_pub_2->publish( wh_grid_.visualise_kiva2(this->get_clock(), 6, 3, 0.0, 0.0, 1.0) ); 

        amazon_kiva_1_goal->publish(wh_grid_.visualise_kiva1_goal(this->get_clock(), 8, 5, 0.0, 1.0, 0.0) );
        amazon_kiva_2_goal->publish(wh_grid_.visualise_kiva2_goal(this->get_clock(), 6, 4, 0.0, 0.0, 1.0) );


        for (int i = 0; i < wh_grid_.map.cells.size(); i++) {
            for (int j = 0; j <wh_grid_.map.cells[0].size(); j++) {
                if (wh_grid_.map.cells[j][i].isObstacle)
                    std::cout << "X";
                else
                    std::cout << "_";
            }
            std::cout << std::endl;
        }
        

        auto optimalPaths = wh_grid_.compute_paths();


        for (auto path : optimalPaths) {
            RCLCPP_INFO(this->get_logger(), "Optimal path of agent \n");
            for (auto cell : path) {
                RCLCPP_INFO(this->get_logger(), "Cell x %d   Cell y: %d \n", cell.x, cell.y);
            }
	    }

    }





}//namespace warehouse

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<warehouse::Mapf>());
  rclcpp::shutdown();
  return 0;
}

