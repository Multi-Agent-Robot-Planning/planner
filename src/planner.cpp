/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/

#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <iostream> 
#include <fstream> 
#include <assert.h> 
#include <regex> 

#include "include/coveragePlanner.hpp"


using namespace std;


tuple<double*, int, int> loadMap(string filepath) {
    FILE *f = fopen(filepath.c_str(), "r");
    if (f) {
    }
    else {
        printf("Opening file failed! \n");
        throw runtime_error("Opening map file failed!");
    }
    int height, width;
    if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
        throw runtime_error("Invalid loadMap parsing map metadata");
    }
    
    ////// Go through file and add to m_occupancy
    double* map = new double[height*width];

    double cx, cy, cz;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            char c;
            do {
                if (fscanf(f, "%c", &c) != 1) {
                    throw runtime_error("Invalid parsing individual map data");
                }
            } while (isspace(c));
            if (!(c == '0')) { 
                map[y+x*width] = 1; // Note transposed from visual
            } else {
                map[y+x*width] = 0;
            }
        }
    }
    fclose(f);
    return make_tuple(map, width, height);
}

vector<string> split(const string& str, const string& delim) {   
        // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
        const regex ws_re(delim);
        return { sregex_token_iterator(str.begin(), str.end(), ws_re, -1), sregex_token_iterator() };
}

double* doubleArrayFromString(string str) {
    vector<string> vals = split(str, ",");
    double* ans = new double[vals.size()];
    for (int i = 0; i < vals.size(); ++i) {
        ans[i] = stod(vals[i]);
    }
    return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << abs(v1[i]-v2[i]) << endl;
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    double* map;
    int x_size, y_size; 
    double** plan = NULL;
    int planlength = 0;
    int camera_fov = 3;

    // tie(map, x_size, y_size) = loadMap(argv[1]);
    // cout << "Map size: " << x_size << " , " << y_size << endl;

    // Planner
    coveragePlanner coverage_planner(camera_fov);
    // std::vector<std::pair<int, int>> map_boundary({std::make_pair(0, 0), std::make_pair(50, 0), std::make_pair(50, 50), std::make_pair(0, 50)});
    // std::vector<std::vector<std::pair<int, int>>> obstacles({{std::make_pair(20, 10), std::make_pair(10, 25), std::make_pair(20, 40), std::make_pair(40, 40), std::make_pair(40,10)}});
    
    std::vector<std::pair<int, int>> map_boundary({std::make_pair(0, 0), std::make_pair(50, 0), std::make_pair(50, 50), std::make_pair(0, 50)});
    std::vector<std::vector<std::pair<int, int>>> obstacles({{std::make_pair(16, 12), std::make_pair(16, 43), std::make_pair(18, 43), std::make_pair(18, 12)}, 
                                {std::make_pair(23, 12), std::make_pair(23, 43), std::make_pair(25, 43), std::make_pair(25, 12)},
                                {std::make_pair(38, 12), std::make_pair(38, 43), std::make_pair(41, 43), std::make_pair(41, 12)}});
    

    std::cout << "\nMap" << endl;
    for(auto p : map_boundary){
        cout << "(" << p.first << ", " << p.second << ")" << "\t";
    }

    std::cout << endl << "Obstacles" << endl;
    for(auto obs : obstacles){
        for(auto p : obs)
            cout << "(" << p.first << ", " << p.second << ")" << "\t";
        cout << endl;
    }
    coverage_planner.decompose_map(map_boundary, obstacles);
    coverage_planner.traverse_cells();
    coverage_planner.build_path();

    std::vector<std::vector<std::pair<int, int>>> full_cell_coverage_path = coverage_planner.get_cell_coverage_path();
    std::vector<std::vector<std::pair<int, int>>> full_discritized_cell_coverage_path = coverage_planner.get_discritized_cell_coverage_path();
    std::cout << endl << "Path" << endl;
    // for(auto cell_path : full_cell_coverage_path){
    //     for(auto p : cell_path)
    //         cout << "(" << p.first << ", " << p.second << ") ";
    //     cout << "\n";
    // }

    for(auto cell_path : full_discritized_cell_coverage_path){
        for(auto p : cell_path)
            cout << "(" << p.first << ", " << p.second << ") ";
        cout << "\n";
        cout << "\n";
    }
}
