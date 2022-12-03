#include<iostream>
#include<vector>

class coveragePlanner
{

    std::vector<Event> events;

    std::vector<Event> get_event_type(std::vector<point2D> polygon);
    void decompose_map(std::vector<point2D> map_boundary, std::vector<std::vector<point2D>> obstacles);    

};