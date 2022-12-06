#include<iostream>
#include<vector>
#include "envDataTypes.hpp"
#include <bits/stdc++.h>
#include <boost/range/algorithm.hpp>
#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext.hpp>

class coveragePlanner
{
    int camera_fov;

    std::vector<Event> events;          
    std::vector<Edge> all_edges;          
    std::vector<Cell> map_cells;
    std::vector<Cell> cell_traversal_path;
    std::vector<std::vector<std::pair<int, int>>> cell_coverage_path;
    std::vector<std::vector<std::pair<int, int>>> discritized_cell_coverage_path;

    static bool event_comparator(Event e1, Event e2)
    {
        return (e1.x_ < e2.x_);
    }

    void get_event_type(std::vector<Point2D> polygon);  
    Point2D draw_line_edge(int x, Edge edge);
    std::pair<Edge, Edge> get_floor_ceiling(Event event);
    void clean_cells(std::vector<Cell> closed_cells);

    std::vector<std::pair<int, int>> build_polygon_path(std::vector<std::pair<int, int>> cell_vertices);
    int leftmost_vertex_idx(std::vector<std::pair<int, int>> cell_vertices);
    std::vector<int> vertical_aligned_edge(int x_current, std::vector<int> x_vec_floor, std::vector<int> y_vec_floor);
    std::pair<int, int> vertical_intersection_with_line(int x_current, std::vector<int> points);
    std::vector<std::pair<int, int>> cell_lawnmover_path(std::vector<int> x_vec_floor, std::vector<int> y_vec_floor, std::vector<int> x_vec_ceiling, std::vector<int> y_vec_ceiling);
    std::pair<std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>> get_polygon_floor_ceiling(std::vector<std::pair<int, int>> cell_vertices);
    
    double cell_dist(Cell cell1, Cell cell2);
    

    public:
        coveragePlanner(int cam_fov);
        void decompose_map(std::vector<std::pair<int, int>> map_boundary_pair, std::vector<std::vector<std::pair<int, int>>> obstacles_pair);  
        void traverse_cells();
        void build_path();
        std::vector<std::vector<std::pair<int, int>>> get_cell_coverage_path();
        std::vector<std::vector<std::pair<int, int>>> get_discretized_cell_coverage_path();
};