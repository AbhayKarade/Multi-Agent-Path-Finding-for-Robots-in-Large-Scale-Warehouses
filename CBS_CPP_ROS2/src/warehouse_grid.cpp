#include "mapf/warehouse_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include "mapf/LowLevelSolver.h"
#include "mapf/HighLevelSolver.h"

using namespace std::chrono_literals;

using GRID_POINT = geometry_msgs::msg::Point;


namespace warehouse{

    WarehouseGrid::WarehouseGrid()
    {
        //Intializing rows and cols for search
        for (int i = 0; i < rows_; i++) {
            std::vector<Cell> row;
            for (int j = 0; j < cols_; j++) {
                row.emplace_back(Cell(i, j));
            }
            cells_.emplace_back(row);
        }


        Agent agent(agentID_);
        agent.start = Cell(2, 2);
        agent.end = Cell(8,5);
        agents_.emplace_back(agent);


        Agent agent1(agentID_);
        agent1.start = Cell(6, 3);
        agent1.end = Cell(5, 4);
        agents_.emplace_back(agent1);

        map.cells = cells_;
	    map.agents = agents_;

    }



    void WarehouseGrid::build_warehouse_map(
        const std::string frame_id,
        const rclcpp::Clock::SharedPtr time_stamp,
        const double & res,
        const int & width,
        const int &height
    )
    {   
        map_.header.frame_id = frame_id;
        map_.header.stamp = time_stamp->now();
        map_.info.resolution = res;
        map_.info.width = width;
        map_.info.height = height;
        map_.info.origin.orientation.w = 1.0;
        map_.data.resize(map_.info.width * map_.info.height);
        rows_ = width;
        cols_ = height;
    }


    void WarehouseGrid::build_obstacles()
    {
        // set_obstacle_cell(Point(0,2));
        // set_obstacle_cell(Point(1,2));
        // set_obstacle_cell(Point(2,2));
        // set_obstacle_cell(Point(0,3));
        // set_obstacle_cell(Point(0,4));

        for(int i = 0; i<=6; i++)
        {
            set_obstacle_cell(Point(4,i));
        }


        for(int i = 8; i<=9; i++)
        {
            set_obstacle_cell(Point(4,i));
        }

        // for(int i = 5; i<=8; i++)
        // {
        //     set_obstacle_cell(Point(6,i));
        // }




        // for(int i = 0; i<=6; i++)
        // {
        //     set_obstacle_cell(Point(4,i));
        // }

        // for(int i = 0; i<=2; i++)
        // {
        //     set_obstacle_cell(Point(5,i));
        // }

        // for(int i = 2; i<=4; i++)
        // {
        //     set_obstacle_cell(Point(i,5));
        // }
        // for(int i = 8; i<=9; i++)
        // {
        //     set_obstacle_cell(Point(4,i));
        // }
        // for(int i = 8; i<=9; i++)
        // {
        //     set_obstacle_cell(Point(2,i));
        // }

        // for(int i = 4; i<=8; i++)
        // {
        //     set_obstacle_cell(Point(6,i));
        // }

        // for(int i = 8; i<=9; i++)
        // {
        //     set_obstacle_cell(Point(i,3));
        // }


        // for(int i = 8; i<=9; i++)
        // {
        //     set_obstacle_cell(Point(i,5));
        // }
        
        // for(int i = 7; i<=8; i++)
        // {
        //     set_obstacle_cell(Point(i,7));
        // }

        // for(int i = 0; i<=1; i++)
        // {
        //     set_obstacle_cell(Point(7,i));
        // }
    
    
    
    }

    /*
    *  @brief: Adds obstacle to warehouse grid environment of given points
    */
    void WarehouseGrid::add_obstacle(const Point point_1, const Point point_2)
    {
        for(int i = point_1.x; i< point_2.x; i++)
        {
            for (int j = point_1.y; j < point_2.y; j++)
            {
                map_.data[map_.info.width * j + i] = cell_occupied;
                map.cells[i][j].isObstacle = true;          
            }            
        }
    }



    void WarehouseGrid::set_obstacle_cell(const Point point)
    {
        map_.data[map_.info.width * point.y + point.x] = cell_occupied;
        map.cells[point.x][point.y].isObstacle = true;
    }

  

    std::vector<std::vector<Cell>>  WarehouseGrid::compute_paths()
    {
        auto started = std::chrono::high_resolution_clock::now();
        HighLevelSolver solver;
	    optimalPaths_ = solver.solve(map);

        auto done = std::chrono::high_resolution_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(done - started).count();

        std::cout << elapsedTime << " milliseconds " ;
        // RCLCPP_INFO(this->get_logger(), "milliseconds %d \n", elapsedTime);

        return optimalPaths_;
        
    }


  

    visualization_msgs::msg::MarkerArray WarehouseGrid::visualise_obstacles(const rclcpp::Clock::SharedPtr time_stamp)
    {
        visualization_msgs::msg::MarkerArray obstalces;
        visualization_msgs::msg::Marker obs_;

        for (int i = 0; i < (map_.info.width * map_.info.height); i++) {
        
            obs_.header.frame_id = "map";
            obs_.header.stamp = time_stamp->now();
            
            if (map_.data[i] == cell_occupied)        
            {
                obs_.type = visualization_msgs::msg::Marker::CUBE;
                obs_.action = visualization_msgs::msg::Marker::ADD;
                obs_.pose.position.x = (i % map_.info.width + 0.5) * cell_size;
                obs_.pose.position.y = ((i / map_.info.width) %  map_.info.height + 0.5) *cell_size;
                obs_.pose.position.z = 0.25;
                obs_.pose.orientation.x = 0.0;
                obs_.pose.orientation.y = 0.0;
                obs_.pose.orientation.z = 0.0;
                obs_.pose.orientation.w = 1.0;
                obs_.scale.x = cell_size;
                obs_.scale.y = cell_size;
                obs_.scale.z = 0.5*cell_size;
                obs_.ns = "obstacle";
                obs_.id = i;
                obs_.color.a = 1.0;
                obs_.color.r = 0.90;
                obs_.color.g = 0.65;
                obs_.color.b = 0.12;       
                obstalces.markers.push_back(obs_);
            }
        }        
        return obstalces;
    }


    visualization_msgs::msg::Marker WarehouseGrid::visualise_kiva1( const rclcpp::Clock::SharedPtr time_stamp, const int pos_x, const int pos_y, const float r, const float g, const float b)
    {
        visualization_msgs::msg::Marker kiva;
        kiva.header.frame_id = "map";
        kiva.header.stamp = time_stamp->now();
        kiva.ns = "warehouse_map_kiva_one";
        kiva.id = 1;
        kiva.type = visualization_msgs::msg::Marker::CYLINDER;

        kiva.action = visualization_msgs::msg::Marker::ADD; 

        kiva.pose.position.x = pos_x+0.5;
        kiva.pose.position.y = pos_y+0.5;
        kiva.pose.position.z = 0.125;
        kiva.pose.orientation.x = 0.0;
        kiva.pose.orientation.y = 0.0;
        kiva.pose.orientation.z = 0.0;
        kiva.pose.orientation.w = 1.0;
        kiva.scale.x = 0.8*cell_size;
        kiva.scale.y = 0.8*cell_size;
        kiva.scale.z = cell_size*0.25;
        kiva.color.a = 1.0;
        kiva.color.r = r;
        kiva.color.g = g;
        kiva.color.b = b;

        return kiva;
    }


    visualization_msgs::msg::Marker WarehouseGrid::visualise_kiva2( const rclcpp::Clock::SharedPtr time_stamp, const int pos_x, const int pos_y, const float r, const float g, const float b)
    {
        visualization_msgs::msg::Marker kiva;
        kiva.header.frame_id = "map";
        kiva.header.stamp = time_stamp->now();
        kiva.ns = "warehouse_map_kiva_two";
        kiva.id = 2;
        kiva.type = visualization_msgs::msg::Marker::CYLINDER;
        kiva.action = visualization_msgs::msg::Marker::ADD; 
   
        kiva.pose.position.x = pos_x+0.5;
        kiva.pose.position.y = pos_y+0.5;
        kiva.pose.position.z = 0.125;
        kiva.pose.orientation.x = 0.0;
        kiva.pose.orientation.y = 0.0;
        kiva.pose.orientation.z = 0.0;
        kiva.pose.orientation.w = 1.0;
        kiva.scale.x = 0.8*cell_size;
        kiva.scale.y = 0.8*cell_size;
        kiva.scale.z = cell_size*0.25;
        kiva.color.a = 1.0;
        kiva.color.r = r;
        kiva.color.g = g;
        kiva.color.b = b;

        return kiva;
    }


    visualization_msgs::msg::Marker WarehouseGrid::visualise_kiva1_goal( const rclcpp::Clock::SharedPtr time_stamp, const int pos_x, const int pos_y, const float r, const float g, const float b)
    {
        visualization_msgs::msg::Marker kiva;
        kiva.header.frame_id = "map";
        kiva.header.stamp = time_stamp->now();
        kiva.ns = "warehouse_map_kiva_one_goal";
        kiva.id = 1;
        kiva.type = visualization_msgs::msg::Marker::CYLINDER;

        kiva.action = visualization_msgs::msg::Marker::ADD; 

        kiva.pose.position.x = pos_x+0.5;
        kiva.pose.position.y = pos_y+0.5;
        kiva.pose.position.z = 0.125;
        kiva.pose.orientation.x = 0.0;
        kiva.pose.orientation.y = 0.0;
        kiva.pose.orientation.z = 0.0;
        kiva.pose.orientation.w = 1.0;
        kiva.scale.x = 0.8*cell_size;
        kiva.scale.y = 0.8*cell_size;
        kiva.scale.z = cell_size*0.25;
        kiva.color.a = 0.4;
        kiva.color.r = r;
        kiva.color.g = g;
        kiva.color.b = b;

        return kiva;
    }

        visualization_msgs::msg::Marker WarehouseGrid::visualise_kiva2_goal( const rclcpp::Clock::SharedPtr time_stamp, const int pos_x, const int pos_y, const float r, const float g, const float b)
    {
        visualization_msgs::msg::Marker kiva;
        kiva.header.frame_id = "map";
        kiva.header.stamp = time_stamp->now();
        kiva.ns = "warehouse_map_kiva_two_goal";
        kiva.id = 1;
        kiva.type = visualization_msgs::msg::Marker::CYLINDER;

        kiva.action = visualization_msgs::msg::Marker::ADD; 

        kiva.pose.position.x = pos_x+0.5;
        kiva.pose.position.y = pos_y+0.5;
        kiva.pose.position.z = 0.125;
        kiva.pose.orientation.x = 0.0;
        kiva.pose.orientation.y = 0.0;
        kiva.pose.orientation.z = 0.0;
        kiva.pose.orientation.w = 1.0;
        kiva.scale.x = 0.8*cell_size;
        kiva.scale.y = 0.8*cell_size;
        kiva.scale.z = cell_size*0.25;
        kiva.color.a = 0.4;
        kiva.color.r = r;
        kiva.color.g = g;
        kiva.color.b = b;

        return kiva;
    }



   

}//namespace warehouse