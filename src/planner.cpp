#include <iostream>
#include <fstream>
#include <vector>
#include "../include/map.h"
#include "../include/motionPrimitives.h"
#include "../include/state.h"
#include "../include/astar.h"

using namespace std;

int main() {
    string output_filename = "matlab/plan.txt";
    Map map("maps/map5.txt");
    // map.printMap();
    vector<MP> mprims_highres, mprims_lowres;

    bool threeD = true;
    
    if(threeD) {
        mprims_highres = MPrims_highres3D();
        mprims_lowres = MPrims_lowres3D();
    } else {
        mprims_highres = MPrims_highres();
        mprims_lowres = MPrims_lowres();
    }

    int startX = 120;
    int startY = 570;
    int startTh = 0;
    int startZ = 5;
    int goalX = 200;
    int goalY = 200;
    int goalTh = 7;
    int goalZ = 6; 
    StatePtr initState = make_shared<State>(startX, startY, startTh, 0, 0, nullptr, -1, 1, startZ);
    StatePtr goalState = make_shared<State>(goalX, goalY, goalTh, 0, 0, nullptr, -1, -1, goalZ);
    vector<StatePtr> path;
    
    // Do planning
    astar(initState, goalState, path, mprims_highres, mprims_lowres, &map, output_filename, threeD);

    /* Write to output file:
    List of (state, action) tuples, where 
        State: (x, y, theta)
        Action: (motion_primitive ID)
    Motion Primitive Library 
    Matlab will visualize these 
    */

   return 0;
}
