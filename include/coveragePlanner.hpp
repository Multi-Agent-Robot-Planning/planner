#include<iostream>
#include<vector>
#include "envDataTypes.hpp"
#include <bits/stdc++.h>

class coveragePlanner
{
    int camera_fov;

    std::vector<Event> events;
    std::vector<Cell> open_cells;
    std::vector<Cell> closed_cells;
    std::vector<Edge> all_edges;
    std::vector<Cell> map_cells;
    std::vector<Cell> cell_traversal_path;

    void get_event_type(std::vector<Point2D> polygon);
    void decompose_map(std::vector<Point2D> map_boundary, std::vector<std::vector<Point2D>> obstacles);    
    bool event_comparator(Event e1, Event e2);
    std::pair<Edge, Edge> get_floor_ceiling(Event event);
    void clean_cells();

    std::vector<Point2D> build_path();
    std::vector<std::pair<int, int>> build_polygon_path(std::vector<std::pair<int, int>> cell_vertices);
    int leftmost_vertex_idx(std::vector<std::pair<int, int>> cell_vertices);
    std::vector<int> vertical_aligned_edge(int x_current, std::vector<int> x_vec_floor, std::vector<int> y_vec_floor);
    std::pair<int, int> vertical_intersection_with_line(int x_current, std::vector<int> points);
    std::vector<std::pair<int, int>> cell_lawnmover_path(std::vector<int> x_vec_floor, std::vector<int> y_vec_floor, std::vector<int> x_vec_ceiling, std::vector<int> y_vec_ceiling);
    std::pair<std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>> get_polygon_floor_ceiling(std::vector<std::pair<int, int>> cell_vertices);
    
    double cell_dist(Cell cell1, Cell cell2);
    void traverse_cells();

};