#include<iostream>
#include<vector>
#include "envDataTypes.hpp"
#include <bits/stdc++.h>

class coveragePlanner
{

    std::vector<Event> events;
    std::vector<Cell> open_cells;
    std::vector<Cell> closed_cells;
    std::vector<Edge> all_edges;
    std::vector<Cell> final_cells;
    std::vector<std::pair<int, int>> final_path;

    void get_event_type(std::vector<Point2D> polygon);
    void decompose_map(std::vector<Point2D> map_boundary, std::vector<std::vector<Point2D>> obstacles);    
    bool event_comparator(Event e1, Event e2);
    std::pair<Edge, Edge> get_floor_ceiling(Event event);
    std::vector<Cell> clean_cells();
    std::vector<Point2D> build_path();
    void build_polygon_path(Point2D p);
    void cell_lawnmover_path(std::vector<int> x_vec_floor, std::vector<int> y_vec_floor, std::vector<int> x_vec_ceiling, std::vector<int> y_vec_ceiling);

};