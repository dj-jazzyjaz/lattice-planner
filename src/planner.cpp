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
    map.printMap();
    vector<MP> mprims = MPrims();
    int startX = 0;
    int startY = 0;
    int startTh = 0;
    int goalX = 8;
    int goalY = 8;
    int goalTh = 8; 
    StatePtr initState = make_shared<State>(startX, startY, startTh, 0, 0, nullptr, -1);
    StatePtr goalState = make_shared<State>(goalX, goalY, goalTh, 0, 0, nullptr, -1);
    vector<StatePtr> path;
    
    // Do planning
    astar(initState, goalState, path, mprims, &map);

    /* Write to output file:
    List of (state, action) tuples, where 
        State: (x, y, theta)
        Action: (motion_primitive ID)
    Motion Primitive Library 
    Matlab will visualize these 
    */

   return 0;
}
