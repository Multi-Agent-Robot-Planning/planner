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

/// @brief Point type that stores x, y coordinate of a point in the 2D map
struct Point2D{
    public:
        int x_, y_;

        Point2D() : x_(-1), y_(-1){}

        Point2D(int x, int y) : x_(x), y_(y){}
};

/// @brief Edge type that stores the edge through two points p1, p2
struct Edge{
    public:
        Point2D p1_, p2_;

        Edge(Point2D p1, Point2D p2) : p1_(p1), p2_(p2){}
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

        Event(std::vector<Point2D>vertices, Point2D prev_vertex, Point2D next_vertex) : 
                                            vertices_(vertices), 
                                            prev_vertex_(prev_vertex), 
                                            next_vertex_(next_vertex){
                                                x_ = vertices_[0].x_;
                                            }
};

#endif