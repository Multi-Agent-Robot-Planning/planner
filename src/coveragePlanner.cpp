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



//create events
class Event
{
    //will take the position of the obstacles and some other things maybe

};

//take obstacle center, height, width, and angle as input
//rotate the obstacles as necessary

//trapezoidal decomposition
//get events from polygon - first send the outer boundry (map)

class point2D
{
    private:
    int x;
    int y;
};

std::vector<point2D> map_boundary;
std::vector<std::vector<point2D>> obstacles;

std::vector<Event> coveragePlanner::get_event_type(std::vector<point2D> polygon)
{

}

void coveragePlanner::decompose_map(std::vector<point2D> map_boundary, std::vector<std::vector<point2D>> obstacles)
{
    get_event_type(map_boundary);

    for(int i=0; i<obstacles.size(); i++)
    {
        get_event_type(obstacles[i]);
    }
}
