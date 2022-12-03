#include<iostream>
#include<vector>
#include "envDataTypes.hpp"
#include <bits/stdc++.h>

class coveragePlanner
{

    std::vector<Event> events;

    void get_event_type(std::vector<Point2D> polygon);
    void decompose_map(std::vector<Point2D> map_boundary, std::vector<std::vector<Point2D>> obstacles);    
    bool event_comparator(Event e1, Event e2);

};