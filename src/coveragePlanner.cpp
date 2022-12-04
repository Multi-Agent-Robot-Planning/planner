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
    int i = 0;

    while(i<polygon.size())
    {
        Point2D previous_vertex = polygon[(i-1) % polygon.size()];
        Point2D current_vertex = polygon[(i) % polygon.size()];
        std::vector<Point2D> inline_vertices;

        if(current_vertex.x_ = previous_vertex.x_)
            continue;

        while(polygon[(i + 1) % polygon.size()].x_ == polygon[(i) % polygon.size()].x_)
        {
            inline_vertices.push_back(polygon[(i + 1) % polygon.size()]);
            i++;
        }

        Point2D next_vertex =  polygon[(i + 1) % polygon.size()];
        events.push_back(Event(inline_vertices, previous_vertex, next_vertex));
        i++;
    }
}

bool coveragePlanner::event_comparator(Event e1, Event e2)
{
    return (e1.x_ > e2.x_);
}

Point2D draw_line_edge(int x, Edge edge)
{
    int y = edge.p1_.y_ + (edge.p2_.y_ - edge.p1_.y_) * (x - edge.p1_.x_) / (edge.p2_.x_ - edge.p1_.x_);
    return Point2D(x,y);
}

void coveragePlanner::get_floor_ceiling(Event event)
{
    for(Edge edge: all_edges)
    {
        if(edge == event.prev_edge_ || edge == event.next_edge_)
            continue;
        //draw vertical line from edge
        Point2D intersection = draw_line_edge(event.x_, edge);
        double dist_to_ceiling = intersection.y_ - event.
    }
}

void coveragePlanner::decompose_map(std::vector<Point2D> map_boundary, std::vector<std::vector<Point2D>> obstacles)
{
    get_event_type(map_boundary);

    for(int i=0; i<obstacles.size(); i++)
    {
        get_event_type(obstacles[i]);
    }
    std::sort(events.begin(), events.end(), event_comparator);

    for(Event event: events)
    {
        floor, ceiling = get_floor_ceiling();

        if(event.event_type_ == IN)
        {
            int i = 0;
            for(Cell cell: open_cells)
            {
                if(floor == cell.floor && ceiling == cell.ceiling)
                {
                    //new bottom cell
                    //new top cell
                    open_cells.push_back(newbottomcell);
                    open_cells.push_back(newtopcell);
                    closed_cells.push_back(cell);
                    open_cells.erase(open_cells.begin()+i);
                    i++;
                    break;
                }
            }
        }
        else if(event.event_type_ == OUT)
        {
            int upperCell_idx, lowerCell_idx; 
            int i = 0;
            for(Cell cell: open_cells)
            {
                if(cell.floor == event.prev_edge_)
                {
                    upperCell_idx = i;
                    Cell upper_cell = cell;
                }
                else if(cell.ceiling == event.next_edge_)
                {
                    lowerCell_idx = i;
                    Cell lower_cell = cell;
                }
                i++;
            }
                //create new cell
            open_cells.erase(open_cells.begin()+std::max(upperCell_idx, lowerCell_idx));
            open_cells.erase(open_cells.begin()+std::min(upperCell_idx, lowerCell_idx));
            
        }
        else if(event.event_type_ == OPEN)
        {
            open_cells.push_back(); //push back a new cell
        }
        else if(event.event_type_ == CLOSE)
        {
            int i = 0;
            for(Cell cell: open_cells)
            {
                if(event.prev_edge_ == cell.floor && event.next_edge_ == cell.ceiling)
                {
                    closed_cells.push_back(cell);
                    open_cells.erase(open_cells.begin() + i);
                    break;
                }
                i++;
            }
        }
        else if(event.event_type_ == FLOOR)
        {
            int i = 0;
            for(Cell cell: open_cells)
            {
                if(event.prev_edge_ = cell.floor && ceiling = cell.ceiling)
                {
                    //new cell
                    closed_cells.push_back(cell);
                    open_cells.erase(open_cells.begin() + i);
                    open_cells.push_back(new_cell);
                    break;
                }
            }
        }
        else if(event.event_type_ == CEILING)
        {
            int i = 0;
            for(Cell cell: open_cells)
            {
                if(floor = cell.floor && event.next_edge_ = cell.ceiling)
                {
                    //new cell
                    closed_cells.push_back(cell);
                    open_cells.erase(open_cells.begin() + i);
                    open_cells.push_back(new_cell);
                    break;
                }
            }
        }

        if(event.prev_vertex_.x_ < event.x_)
            //current eges remove event.prevedge
        else
            //current edges append prevedge

        if(event.nexr_vertex_.x_ < event.x_)
            //current eges remove event.nextedge
        else
            //current edges append nextedge
    }

    //remove degenerate cells and return
}

double coveragePlanner::cell_dist(Cell cell1, Cell cell2){
    Point2D centroid1 = cell1.get_centroid();
    Point2D centroid2 = cell2.get_centroid();
    double dist = sqrt(pow((centroid1.x_ - centroid2.x_), 2) + pow((centroid1.y_ - centroid2.y_), 2));
    return dist;
}

void coveragePlanner::traverse_cells(void){

    for(int i=0; i<closed_cells.size(); i++){
        closed_cells[i].id_ = i;
    }

    std::vector<int> unvisited(0, closed_cells.size());
    unvisited.erase(std::remove(unvisited.begin(), unvisited.end(), 0), unvisited.end());
    std::vector<int> path_list;
    path_list.push_back(0);

    while(!unvisited.empty()){
        
        auto cell = closed_cells.back();
        bool neighbors_visited = true;
        std::vector<int> next_ids;

        for(auto neighbor : *cell.neighbors_){
            if (std::find(unvisited.begin(), unvisited.end(), neighbor.id_) != unvisited.end()){
                next_ids.push_back(neighbor.id_);
                neighbors_visited = false;
            }
        }

        if(neighbors_visited)
            next_ids.assign(unvisited.begin(), unvisited.end()); 
    

        int min_dist = INT_MIN;
        int closest_cell = -1;
        for(auto id : next_ids){
            auto next_cell = closed_cells[id];
            double dist = cell_dist(cell, next_cell);
            if (dist < min_dist){
                min_dist = dist;
                closest_cell = id;
            }
        }
        unvisited.erase(std::remove(unvisited.begin(), unvisited.end(), closest_cell), unvisited.end());
        path_list.push_back(closest_cell);
    }    
}   

