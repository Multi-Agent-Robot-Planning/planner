/**
 * @file envDataTypes.hpp
 * @author Nanditha Umasankar (unandiitha02@gmail.com)
 * @brief Data structures to build planner
 * 
 * 
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once
#ifndef ENV_DATA_TYPES_H
#define ENV_DATA_TYPES_H

#include <vector>
#include <memory>
#include <bits/stdc++.h>

/// @brief Point type that stores x, y coordinate of a point in the 2D map
struct Point2D{
    public:
        int x_, y_;

        Point2D() : x_(-1), y_(-1){}
        Point2D(int x, int y) : x_(x), y_(y){}

        bool operator==(const Point2D& p){
            return((this->x_ == p.x_) && (this->y_ == p.y_));
        }
};

/// @brief Edge type that stores the edge through two points p1, p2
struct Edge{
    public:
        Point2D p1_, p2_;

        Edge(){}
        Edge(Point2D p1, Point2D p2) : p1_(p1), p2_(p2){}

        bool operator==(const Edge& e){
            return((this->p1_ == e.p1_ && this->p2_ == e.p2_) || (this->p1_ == e.p2_ && this->p2_ == e.p1_));
        }    

        void print_edge(void){
            std::cout << "[(" << p1_.x_ << ", " << p1_.y_ << "), ("<< p2_.x_ << ", " << p2_.y_ << ")]";
        }
};

/// @brief 
enum EventType{
    IN,
    OUT,
    OPEN,
    CLOSE,
    FLOOR,
    CEILING
};

/// @brief 
class Event{
    public:
        std::vector<Point2D> vertices_;
        Point2D prev_vertex_;
        Point2D next_vertex_;
        EventType event_type_;
        int x_;
        Edge prev_edge_;
        Edge next_edge_;
        int y_max = INT_MIN, y_min = INT_MAX;

        Event(std::vector<Point2D>vertices, Point2D prev_vertex, Point2D next_vertex) : 
                                            vertices_(vertices), 
                                            prev_vertex_(prev_vertex), 
                                            next_vertex_(next_vertex)
                                            {
                                                x_ = vertices_[0].x_;
                                                prev_edge_ = Edge(prev_vertex_, vertices_[0]);
                                                next_edge_ = Edge(vertices_.back(), next_vertex_);

                                                find_max();
                                                find_min();
                                                set_event_type();
                                            }

        void find_max(void){
            for(Point2D v : vertices_){
                if(v.y_ > y_max)
                    y_max = v.y_;
            }
        };
        void find_min(void){
            for(Point2D v : vertices_){
                if(v.y_ < y_min)
                    y_min = v.y_;
            }
        };

        void set_event_type(void){
            if((prev_vertex_.x_ < x_) && (next_vertex_.x_ < x_)){
                if(prev_vertex_.y_ > next_vertex_.y_)
                    event_type_ = OUT;
                else
                    event_type_ = CLOSE;
            }
            else if((prev_vertex_.x_ > x_) && (next_vertex_.x_ > x_)){
                if(prev_vertex_.y_ > next_vertex_.y_)
                    event_type_ = OPEN;
                else
                    event_type_ = IN;
            }
            else if((prev_vertex_.x_ < x_) && (next_vertex_.x_ > x_)){
                event_type_ = FLOOR;
            }
            else{
                event_type_ = CEILING;
            }
        }

        void print_event_details(void){
            std::cout << "\n---------------------------------------------------------------------------------------------- " << std::endl;
            std::cout << "Event Type: " << event_type_ << std::endl;
            std::cout << "x: " << x_ << std::endl;
            std::cout << "Event Vertices" << std::endl;
            for(Point2D p: vertices_){
                std::cout << "(" << p.x_ << ", " << p.y_ << ") ";
            }

            std::cout << "\nPrev vertex: " << "(" << prev_vertex_.x_ << ", " << prev_vertex_.y_ << ") " << std::endl;
            std::cout << "Next vertex: " << "(" << next_vertex_.x_ << ", " << next_vertex_.y_ << ") " << std::endl;

            std::cout << "Prev: ";
            prev_edge_.print_edge();
            std::cout << "\nNext: ";
            next_edge_.print_edge();
            std::cout << "\n---------------------------------------------------------------------------------------------- " << std::endl;
        }
};

class Cell{
    public:
        int id_;
        Edge ceiling_;
        Edge floor_;

        int x_left_=-1;
        int x_right_=-1;

        std::vector<Cell> neighbors_;

        std::shared_ptr<Cell> prev_;
        std::shared_ptr<Cell> next_;    

        Cell(){};
        Cell( Edge floor, Edge ceiling, int x_left, int x_right, std::vector<Cell> neighbors = {}) : 
                                                ceiling_(ceiling), 
                                                floor_(floor), 
                                                x_left_(x_left),
                                                x_right_(x_right),
                                                neighbors_(neighbors)
                                                {}
        Cell( Edge floor, Edge ceiling, int x_left, std::vector<Cell> neighbors = {}) : 
                                                ceiling_(ceiling), 
                                                floor_(floor), 
                                                x_left_(x_left),
                                                neighbors_(neighbors)
                                                {}

        bool operator==(const Cell &c) const {
            return c.id_ == id_;
        }            

        Point2D vertical_edge(double x, Edge edge){
            int y = edge.p1_.y_ + (edge.p2_.y_ - edge.p1_.y_) * (x - edge.p1_.x_) / (edge.p2_.x_ - edge.p1_.x_);
            return Point2D(x, y);
        }

        std::vector<Point2D> get_vertices(){
            std::vector<Point2D> points;
            points.push_back(vertical_edge(x_left_, floor_));
            points.push_back(vertical_edge(x_right_, floor_));
            points.push_back(vertical_edge(x_right_, ceiling_));
            points.push_back(vertical_edge(x_left_, ceiling_));

            return std::vector<Point2D>(points);
        }

        Point2D get_centroid(){
            auto points = get_vertices();
            int x_mean = 0, y_mean = 0;
            for(auto p : points){
                x_mean += p.x_;
                y_mean += p.y_;
            }
            x_mean /= points.size();
            y_mean /= points.size();
            return Point2D(x_mean, y_mean);
        }

        void print_cell_details(void){
            std::cout << "\n---------------------------------------------------------------------------------------------- " << std::endl;
            std::cout << "id: " << id_ << std::endl;
            std::cout << "Cell Vertices" << std::endl;
            for(Point2D p: get_vertices()){
                std::cout << "(" << p.x_ << ", " << p.y_ << ") ";
            }

            std::cout << "\nLeft cell: " << x_left_ << std::endl;
            std::cout << "Right Cell: " << x_right_ << std::endl;

            std::cout << "Floor: ";
            floor_.print_edge();
            std::cout << "\nCeiling: ";
            ceiling_.print_edge();
            std::cout << "\n---------------------------------------------------------------------------------------------- " << std::endl;
        }
};

#endif