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

#include "../include/coveragePlanner.hpp"
#include "../include/envDataTypes.hpp"


coveragePlanner::coveragePlanner(int cam_fov) : camera_fov(cam_fov){}

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

        inline_vertices.push_back(current_vertex);
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

void coveragePlanner::clean_cells(std::vector<Cell> closed_cells)
{
    for(Cell cell: closed_cells)
    {
        if(cell.x_left_ != cell.x_right_)
        {
            map_cells.push_back(cell);
            continue;
        }
        int i=0;
        for(Cell neighbor: *(cell.neighbors_))
        {
            std::vector<Cell> meta_neighbors = *(neighbor.neighbors_);
            int idx = std::find(meta_neighbors.begin(), meta_neighbors.end(), cell) != meta_neighbors.end();
            
            std::vector<Cell> new_meta_neighbors;
            new_meta_neighbors.push_back(meta_neighbors[0, idx]);
            new_meta_neighbors.insert(new_meta_neighbors.end(), (*cell.neighbors_).begin(), (*cell.neighbors_).begin()+i);
            new_meta_neighbors.insert(new_meta_neighbors.end(), (*cell.neighbors_).begin()+(i+1), (*cell.neighbors_).end());
            new_meta_neighbors.insert(new_meta_neighbors.end(), meta_neighbors.begin()+(idx+1), meta_neighbors.end());
        
            neighbor.neighbors_ = std::make_shared<std::vector<Cell>>(new_meta_neighbors);
        }
    }
}

void coveragePlanner::decompose_map(std::vector<std::pair<int, int>> map_boundary_pair, std::vector<std::vector<std::pair<int, int>>> obstacles_pair)
{
    std::vector<Point2D> map_boundary;
    for(int i=0; i<map_boundary_pair.size(); i++)
    {
        Point2D point;
        point.x_ = map_boundary_pair[i].first;
        point.y_ = map_boundary_pair[i].second;
        map_boundary.push_back(point);
    }

    std::vector<std::vector<Point2D>> obstacles;
    for(int i=0; i<obstacles.size(); i++)
    {
        for(int j=0; j<obstacles_pair[i].size(); i++)
        {
            Point2D point;
            point.x_ = (obstacles_pair.at(i).at(j)).first;
            point.y_ = (obstacles_pair.at(i).at(j)).second;
            map_boundary.push_back(point);
        }
    }


    std::vector<Cell> open_cells;
    std::vector<Cell> closed_cells;
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
                if(floor == cell.floor_ && ceiling == cell.ceiling_)
                {
                    Cell lower_cell(floor, event.prev_edge_, event.x_, NULL, std::make_shared<std::vector<Cell>>(cell));
                    Cell higher_cell(event.next_edge_, ceiling, event.x_, NULL, std::make_shared<std::vector<Cell>>(cell));
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
                if(cell.floor_ == event.prev_edge_)
                {
                    upperCell_idx = i;
                    upper_cell = cell;
                }
                else if(cell.ceiling_ == event.next_edge_)
                {
                    lowerCell_idx = i;
                    lower_cell = cell;
                }
                i++;
            }
            Cell new_cell(floor, ceiling, event.x_, NULL, std::make_shared<std::vector<Cell>>(upper_cell, lower_cell)); 
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
                if(event.prev_edge_ == cell.floor_ && event.next_edge_ == cell.ceiling_)
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
                if(event.prev_edge_ == cell.floor_ && ceiling == cell.ceiling_)
                {
                    Cell new_cell(event.next_edge_, ceiling, event.x_, NULL, std::make_shared<std::vector<Cell>>(cell)); 
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
                if(floor == cell.floor_ && event.next_edge_ == cell.ceiling_)
                {
                    Cell new_cell(floor, event.prev_edge_, event.x_, NULL, std::make_shared<std::vector<Cell>>(cell)); 
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

    clean_cells(closed_cells);
}

int coveragePlanner::leftmost_vertex_idx(std::vector<std::pair<int, int>> cell_vertices)
{
    std::pair<int, int> leftmost_vertex = std::make_pair(NULL, NULL);   // (-1, -1)
    int vertex_idx;
    int i = 0;
    for(std::pair<int, int> vertex: cell_vertices)
    {
        if(leftmost_vertex.first == NULL || vertex.first < leftmost_vertex.first)
        {
            leftmost_vertex = vertex;
            vertex_idx = i;
        }
        else if(vertex.first == leftmost_vertex.first && vertex.second < leftmost_vertex.second)
        {
            leftmost_vertex = vertex;
            vertex_idx = i;
        }
        i++;
    }
    return vertex_idx;
}

std::pair<std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>> coveragePlanner::get_polygon_floor_ceiling(std::vector<std::pair<int, int>> cell_vertices)
{
    int first_idx = leftmost_vertex_idx(cell_vertices);
    bool append_floor_vertices = true;
    std::vector<std::pair<int, int>> floor_vertices;
    floor_vertices.push_back(cell_vertices[first_idx]);
    std::pair<int, int> last_vertex = cell_vertices[first_idx];
    std::vector<std::pair<int, int>> ceiling_vertices;
    for(int i=1; i<cell_vertices.size(); i++)
    {
        std::pair<int, int> vertex = cell_vertices[(first_idx+i)%cell_vertices.size()];
        if(vertex.first < last_vertex.first && append_floor_vertices)
        {
            ceiling_vertices.push_back(last_vertex);
            append_floor_vertices = false;
        }
        else if(vertex.first > last_vertex.first && !append_floor_vertices)
        {
            assert(false);
        }

        if(append_floor_vertices)
            floor_vertices.push_back(vertex);
        else
            ceiling_vertices.push_back(vertex);

        last_vertex = vertex;
    }

    if(append_floor_vertices)
        ceiling_vertices.push_back(last_vertex);
    ceiling_vertices.push_back(cell_vertices[first_idx]);
    std::reverse(ceiling_vertices.begin(), cell_vertices.end());

    return std::make_pair(floor_vertices, ceiling_vertices);
}

std::vector<std::pair<int, int>> coveragePlanner::build_polygon_path(std::vector<std::pair<int, int>> cell_vertices)
{
    std::pair<std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>> floor_ceiling_pair = get_polygon_floor_ceiling(cell_vertices);
    std::vector<std::pair<int, int>> floor_vertices = floor_ceiling_pair.first;
    std::vector<std::pair<int, int>> ceiling_vertices = floor_ceiling_pair.second;

    std::vector<int> x_vec_floor;
    std::vector<int> y_vec_floor;
    std::vector<int> x_vec_ceiling;
    std::vector<int> y_vec_ceiling;
     
    for(int i = 0; i<floor_vertices.size(); i++)
    {
       x_vec_floor.push_back(floor_vertices[i].first);
       y_vec_floor.push_back(floor_vertices[i].second);
    }
    for(int i = 0; i<ceiling_vertices.size(); i++)
    {
       x_vec_ceiling.push_back(ceiling_vertices[i].first);
       y_vec_ceiling.push_back(ceiling_vertices[i].second);
    }
    return(cell_lawnmover_path(x_vec_floor, y_vec_floor,x_vec_ceiling, y_vec_ceiling));
}   

std::vector<std::pair<int, int>> coveragePlanner::build_path()
{
    std::vector<std::pair<int, int>> coverage_path;
    for(Cell cell: cell_traversal_path)
    {
        std::vector<std::pair<int, int>> cell_vertices;
        for(Point2D p: *(cell.get_vertices()))
        {
            cell_vertices.push_back(std::make_pair(p.x_, p.y_));
        }
        std::vector<std::pair<int, int>> polygon_path = build_polygon_path(cell_vertices);
        coverage_path.insert(coverage_path.end(), polygon_path.begin(), polygon_path.end());
    }
    return coverage_path;
}



std::vector<int> coveragePlanner::vertical_aligned_edge(int x_current, std::vector<int> x_vec, std::vector<int> y_vec){
    std::vector<int> points;
    for(int i=1; i<x_vec.size(); i++){
        if(x_vec[i] >= x_current){
            points.push_back(x_vec[i-1]); //x1
            points.push_back(y_vec[i-1]); //y1
            points.push_back(x_vec[i]); //x2
            points.push_back(y_vec[i]); //y2
            return points;
        }
    }
}

std::pair<int, int> coveragePlanner::vertical_intersection_with_line(int x_current, std::vector<int> points){
    int x1 = points[0], y1 = points[1], x2 = points[2], y2 = points[3];    
    int y = y1 + (y2 -y1) * (x_current - x1) / (x2 - x1);

    return std::make_pair(x_current, y);
}

std::vector<std::pair<int, int>> coveragePlanner::cell_lawnmover_path(std::vector<int> x_vec_floor, std::vector<int> y_vec_floor, std::vector<int> x_vec_ceiling, std::vector<int> y_vec_ceiling)
{
    int x_min = x_vec_floor[0];
    int x_max = x_vec_floor.back();
    std::vector<std::pair<int, int>> cell_path;
    cell_path.push_back(std::make_pair(x_vec_floor[0], y_vec_floor[0]));
    int x_current = x_min;
    bool moving_forward = true;
    std::vector<int> points;
    while(1)
    {
        if(moving_forward)
            points = vertical_aligned_edge(x_current, x_vec_ceiling, y_vec_ceiling);
        else
            points = vertical_aligned_edge(x_current, x_vec_floor, y_vec_floor);

        if(points[0] == points[2]){
            if(moving_forward)
                cell_path.push_back(std::make_pair(x_current, std::max(points[1], points[3])));
            else
                cell_path.push_back(std::make_pair(x_current, std::min(points[1], points[3])));
        }
        else
            cell_path.push_back(vertical_intersection_with_line(x_current, points));

        if(x_current >= x_max)
            break;

        x_current = std::min(x_current + camera_fov, x_max);

        if(moving_forward)
            points = vertical_aligned_edge(x_current, x_vec_ceiling, y_vec_ceiling);
        else
            points = vertical_aligned_edge(x_current, x_vec_floor, y_vec_floor);

        if(points[0] == points[2]){
            if(moving_forward)
                cell_path.push_back(std::make_pair(x_current, std::max(points[1], points[3])));
            else
                cell_path.push_back(std::make_pair(x_current, std::min(points[1], points[3])));
        }
        else
            cell_path.push_back(vertical_intersection_with_line(x_current, points));
    moving_forward = !moving_forward;
    }
    return cell_path;
}
  
double coveragePlanner::cell_dist(Cell cell1, Cell cell2){
    Point2D centroid1 = cell1.get_centroid();
    Point2D centroid2 = cell2.get_centroid();
    double dist = sqrt(pow((centroid1.x_ - centroid2.x_), 2) + pow((centroid1.y_ - centroid2.y_), 2));
    return dist;
}

void coveragePlanner::traverse_cells(void){

    for(int i=0; i<map_cells.size(); i++){
        map_cells[i].id_ = i;
    }

    std::vector<int> unvisited(0, map_cells.size());
    unvisited.erase(std::remove(unvisited.begin(), unvisited.end(), 0), unvisited.end());
    std::vector<int> path_list;
    path_list.push_back(0);

    while(!unvisited.empty()){
        
        auto cell = map_cells[path_list.back()];
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
            auto next_cell = map_cells[id];             // find cell with the 'id_ = id' not index id
            double dist = cell_dist(cell, next_cell);
            if (dist < min_dist){
                min_dist = dist;
                closest_cell = id;
            }
        }
        unvisited.erase(std::remove(unvisited.begin(), unvisited.end(), closest_cell), unvisited.end());
        path_list.push_back(closest_cell);
    }   
    for(int i=0; i<path_list.size(); i++){
        cell_traversal_path.push_back(map_cells[path_list[i]]);     // push cell with the 'id_ = id' not index id
    } 
}   

