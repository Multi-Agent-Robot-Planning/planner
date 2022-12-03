/**
 * @file coveragePlanner.cpp
 * @author Sundaram Seivur (sseivur@andrew.cmu.edu)
 * @brief 
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "coveragePlanner.hpp"
#include "envDataTypes.hpp"

std::vector<Point2D> map_boundary;
std::vector<std::vector<Point2D>> obstacles;

void coveragePlanner::get_event_type(std::vector<Point2D> polygon)
{
    for(int i=0; i<polygon.size(); i++)
    {
        std::vector<Point2D> previous_vertex = polygon[]
    }
}

bool coveragePlanner::event_comparator(Event e1, Event e2)
{
    return (e1.x_ > e2.x_);
}

void coveragePlanner::decompose_map(std::vector<Point2D> map_boundary, std::vector<std::vector<Point2D>> obstacles)
{
    get_event_type(map_boundary);

    for(int i=0; i<obstacles.size(); i++)
    {
        get_event_type(obstacles[i]);
    }
}
