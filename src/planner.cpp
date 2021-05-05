#include <iostream>
#include <fstream>
#include <vector>
#include<string>  
#include "../include/map.h"
#include "../include/motionPrimitives.h"
#include "../include/state.h"
#include "../include/astar.h"

using namespace std;

int main() {
    int threeDIn;
    int startX, startY, startZ, startTh, goalX, goalY, goalZ, goalTh;
    int mapNum;
    scanf("ThreeD: %d\n", &threeDIn), 
    scanf("Start: %d %d %d %d\n", &startX, &startY, &startZ, &startTh);
    scanf("Goal: %d %d %d %d\n", &goalX, &goalY, &goalZ, &goalTh);
    scanf("Map Num: %d\n", &mapNum);

    string output_filename = "matlab/plan.txt";
    string map_filename = "maps/map" + to_string(mapNum) + ".txt";
    Map map(map_filename);

    // map.printMap();
    vector<MP> mprims_highres, mprims_lowres;

    bool threeD = (threeDIn == 1) ? true : false;
    
    if(threeD) {
        mprims_highres = MPrims_highres3D();
        mprims_lowres = MPrims_lowres3D();
    } else {
        mprims_highres = MPrims_highres();
        mprims_lowres = MPrims_lowres();
    }

    StatePtr initState = make_shared<State>(startX, startY, startTh, 0, 0, nullptr, -1, 1, startZ);
    StatePtr goalState = make_shared<State>(goalX, goalY, goalTh, 0, 0, nullptr, -1, -1, goalZ);
    vector<StatePtr> path;
    
    // Do planning
    astar(initState, goalState, path, mprims_highres, mprims_lowres, &map, mapNum, output_filename, threeD);

    /* Write to output file:
    List of (state, action) tuples, where 
        State: (x, y, theta)
        Action: (motion_primitive ID)
    Motion Primitive Library 
    Matlab will visualize these 
    */

   return 0;
}
