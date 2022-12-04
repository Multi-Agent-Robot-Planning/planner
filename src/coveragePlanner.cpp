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

std::pair<Edge, Edge> coveragePlanner::get_floor_ceiling(Event event)
{   
    Edge floor, ceiling;
    double floor_dist = 0;
    double ceiling_dist = 0; 
    for(Edge edge: all_edges)
    {
        if(edge == event.prev_edge_ || edge == event.next_edge_)
            continue;
        //draw vertical line from edge
        Point2D intersection = draw_line_edge(event.x_, edge);
        double dist_to_ceiling = intersection.y_ - event.y_max;
        double dist_to_floor = event.y_min - intersection.y_;

        if(dist_to_floor > 0)
        {
            if(floor_dist == 0 || dist_to_floor < floor_dist)
            {
                floor_dist = dist_to_floor;
                floor = edge;
            }
        }
        else if(dist_to_ceiling > 0)
        {
            if(ceiling_dist == 0 || dist_to_ceiling < ceiling_dist)
            {
                ceiling_dist = dist_to_ceiling;
                ceiling = edge;
            }
        }
    }
    return std::make_pair(floor, ceiling);
}

std::vector<Cell> coveragePlanner::clean_cells()
{
    for(Cell cell: closed_cells)
    {
        if(cell.x_left_ != cell.x_right_)
        {
            final_cells.push_back(cell);
            continue;
        }
        for(Cell neighbor: *(cell.neighbors_))
        {
            std::vector<Cell> meta_neighbors = *(neighbor.neighbors_);
            int index = meta_neighbors.
        }
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
        std::pair<Edge, Edge> floor_ceil_pair = get_floor_ceiling(event);
        Edge floor = floor_ceil_pair.first;
        Edge ceiling = floor_ceil_pair.second;

        if(event.event_type_ == IN)
        {
            int i = 0;
            for(Cell cell: open_cells)
            {
                if(floor == cell.floor && ceiling == cell.ceiling)
                {
                    Cell lower_cell(floor, event.prev_edge_, event.x_, NULL, cell);
                    Cell higher_cell(event.next_edge_, ceiling, event.x_, NULL, cell);
                    open_cells.push_back(lower_cell);
                    open_cells.push_back(higher_cell);
                    cell.x_right_ = event.x_;
                    cell.neighbors_->push_back(lower_cell);
                    cell.neighbors_->push_back(higher_cell);
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
            Cell upper_cell, lower_cell;
            int i = 0;
            for(Cell cell: open_cells)
            {
                if(cell.floor == event.prev_edge_)
                {
                    upperCell_idx = i;
                    upper_cell = cell;
                }
                else if(cell.ceiling == event.next_edge_)
                {
                    lowerCell_idx = i;
                    lower_cell = cell;
                }
                i++;
            }
            Cell new_cell(floor, ceiling, event.x_, NULL, ); 
            open_cells.erase(open_cells.begin()+std::max(upperCell_idx, lowerCell_idx));
            open_cells.erase(open_cells.begin()+std::min(upperCell_idx, lowerCell_idx));
            upper_cell.x_right_ = event.x_;
            lower_cell.x_right_ = event.x_;
            upper_cell.neighbors_->push_back(new_cell);
            lower_cell.neighbors_->push_back(new_cell);
            closed_cells.push_back(upper_cell);
            closed_cells.push_back(lower_cell);
            open_cells.push_back(new_cell);
        }
        else if(event.event_type_ == OPEN)
        {
            open_cells.push_back(Cell(event.next_edge_, event.prev_edge_, event.x_, NULL)); //push back a new cell
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
                    Cell new_cell(event.next_edge_, ceiling, event.x_, NULL, ); 
                    cell.x_right_ = event.x_;
                    cell.neighbors_->push_back(new_cell);
                    closed_cells.push_back(cell);
                    open_cells.erase(open_cells.begin() + i);
                    open_cells.push_back(new_cell);
                    break;
                }
                i++;
            }
        }
        else if(event.event_type_ == CEILING)
        {
            int i = 0;
            for(Cell cell: open_cells)
            {
                if(floor = cell.floor && event.next_edge_ = cell.ceiling)
                {
                    Cell new_cell(floor, event.prev_edge_, event.x_, NULL, ); 
                    cell.x_right_ = event.x_;
                    cell.neighbors_->push_back(new_cell);
                    closed_cells.push_back(cell);
                    open_cells.erase(open_cells.begin() + i);
                    open_cells.push_back(new_cell);
                    break;
                }
                i++;
            }
        }

        if(event.prev_vertex_.x_ < event.x_)
            all_edges.erase(std::remove(all_edges.begin(), all_edges.end(), event.prev_edge_), all_edges.end());
        else
            all_edges.push_back(event.prev_edge_);

        if(event.next_vertex_.x_ < event.x_)
            all_edges.erase(std::remove(all_edges.begin(), all_edges.end(), event.next_edge_), all_edges.end());
        else
            all_edges.push_back(event.next_edge_);

    }

    clean_cells();
}

std::vector<Point2D> coveragePlanner::build_path()
{
    for(Cell cell: traverse_cells)
    {
        for(Point2D p: *(cell.get_vertices()))
        {
            final_path.push_back();
        }
    }
}

void coveragePlanner::build_polygon_path(Point2D p)
{
    std::pair<std::vector<Point2D>, std::vector<Point2D>> floor_ceiling_pair = get_polygon_floor_ceiling();
    std::vector<Point2D> floor_vertices = floor_ceiling_pair.first;
    std::vector<Point2D> ceiling_vertices = floor_ceiling_pair.second;

    std::vector<int> x_vec_floor;
    std::vector<int> y_vec_floor;
    std::vector<int> x_vec_ceiling;
    std::vector<int> y_vec_ceiling;
     
     for(int i = 0; i<floor_vertices.size(); i++)
     {
        x_vec_floor = floor_vertices[i][0];
        y_vec_floor = floor_vertices[i][1];
     }
     for(int i = 0; i<ceiling_vertices.size(); i++)
     {
        x_vec_ceiling = ceiling_vertices[i][0];
        y_vec_ceiling = ceiling_vertices[i][1];
     }
     cell_lawnmover_path(x_vec_floor, y_vec_floor,x_vec_ceiling, y_vec_ceiling);
}    

void coveragePlanner::cell_lawnmover_path(std::vector<int> x_vec_floor, std::vector<int> y_vec_floor, std::vector<int> x_vec_ceiling, std::vector<int> y_vec_ceiling)
{
    int x_min = x_vec_floor[0];
    int x_max = x_vec_floor.back();
    final_path.push_back(std::make_pair(x_vec_floor[0], y_vec_floor[0]));
    int x_current = x_min;

    while(1)
    {

    }
}

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

