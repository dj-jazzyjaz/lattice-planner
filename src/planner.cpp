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
    Map map("maps/map7.txt");
    // map.printMap();
    vector<MP> mprims = MPrims();
    int startX = 0;
    int startY = 0;
    int startTh = 0;
    int goalX = 59;
    int goalY = 99;
    int goalTh = 7; 
    StatePtr initState = make_shared<State>(startX, startY, startTh, 0, 0, nullptr, -1);
    StatePtr goalState = make_shared<State>(goalX, goalY, goalTh, 0, 0, nullptr, -1);
    vector<StatePtr> path;
    
    // Do planning
    astar(initState, goalState, path, mprims, &map, output_filename);

    /* Write to output file:
    List of (state, action) tuples, where 
        State: (x, y, theta)
        Action: (motion_primitive ID)
    Motion Primitive Library 
    Matlab will visualize these 
    */

   return 0;
}
