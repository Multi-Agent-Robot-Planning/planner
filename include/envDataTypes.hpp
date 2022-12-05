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
        int y_max = INT_MAX, y_min = INT_MIN;

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
                    y_max = v.y_;
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
            else if((prev_vertex_.x_ < x_) && (next_vertex_.x_ > x_))
                event_type_ = FLOOR;
            else
                event_type_ = CEILING;
        }
};

class Cell{
    public:
        int id_;
        Edge ceiling_;
        Edge floor_;

        double x_left_;
        double x_right_;

        std::shared_ptr<std::vector<Cell>> neighbors_;

        std::shared_ptr<Cell> prev_;
        std::shared_ptr<Cell> next_;    

        Cell();
        Cell( Edge floor, Edge ceiling, double x_left, double x_right, std::shared_ptr<std::vector<Cell>> neighbors = nullptr) : 
                                                ceiling_(ceiling), 
                                                floor_(floor), 
                                                x_left_(x_left),
                                                x_right_(x_right),
                                                neighbors_(neighbors)
                                                {}

        Point2D vertical_edge(double x, Edge edge){
            double y = 0;
            return Point2D(x, y);
        }

        std::shared_ptr<std::vector<Point2D>> get_vertices(){
            std::vector<Point2D> points;
            points.push_back(vertical_edge(x_left_, floor_));
            points.push_back(vertical_edge(x_right_, floor_));
            points.push_back(vertical_edge(x_right_, ceiling_));
            points.push_back(vertical_edge(x_left_, ceiling_));

            return std::make_shared<std::vector<Point2D>>(points);
        }

        Point2D get_centroid(){
            auto points = *get_vertices();
            int x_mean = 0, y_mean = 0;
            for(auto p : points){
                x_mean += p.x_;
                y_mean += p.y_;
            }
            x_mean /= points.size();
            y_mean /= points.size();
            return Point2D(x_mean, y_mean);
        }
};

#endif