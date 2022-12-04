#include<iostream>
#include<vector>
#include "envDataTypes.hpp"
#include <bits/stdc++.h>

class coveragePlanner
{

    std::vector<Event> events;
    std::vector<Cell> open_cells;
    std::vector<Cell> closed_cells;

    void get_event_type(std::vector<Point2D> polygon);
    void decompose_map(std::vector<Point2D> map_boundary, std::vector<std::vector<Point2D>> obstacles);    
    bool event_comparator(Event e1, Event e2);
    void get_floor_ceiling();
    double cell_dist(Cell cell1, Cell cell2);
    void traverse_cells();

};