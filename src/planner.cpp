#include <iostream>
#include <fstream>
#include <vector>
#include "../include/map.h"
#include "../include/motionPrimitives.h"
#include "../include/state.h"
#include "../include/astar.h"

using namespace std;

int main() {
    Map map("maps/map0.txt");
    vector<MP> mprims = MPrims();
    int poseX = 0;
    int poseY = 0;
    int poseTh = 0;
    State initState(poseX, poseY, poseTh, 0, 0, nullptr, -1);
    // Do planning

    /* Write to output file:
    List of (state, action) tuples, where 
        State: (x, y, theta)
        Action: (motion_primitive ID)
    Motion Primitive Library 
    Matlab will visualize these 
    */

   return 0;
}
