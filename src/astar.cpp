#include "../include/astar.h"
#include "../include/state.h"
#include "../include/motionPrimitives.h"
#include <iostream>
#include <math.h>
#include <memory>
#include <random>
#include <stdio.h>
#include <vector>
#include <queue>
#include <time.h>

// Threshold of distance from goal to switch from using high-resolution primitives to low-resolution primitives
#define HI_RES_THRESH 25 // TODO: play around with this parameter

double computeH(int x1, int y1, int th1, int z1, const StatePtr s2, int num_Angle_Discretizations, bool threeD) 
{
    double dist = hypot(x1- s2->x, y1 - s2->y);
    if(threeD) dist = hypot(dist, (z1 - s2->z));
    dist = dist * 0.1;

    double s = abs(th1 - s2->t);
    double angleDiff = min(s, num_Angle_Discretizations - s);

    // TODO: play around more with this heuristic 
    double angleScale = dist == 0 ? 10 : (10/dist);
    // printf("Dist = %.2f, Angle Scale = %.2f, s = %.2f, h=%.2f\n", dist, angleScale, s, dist+angleScale*s);
    return dist; // + angleScale * s;
}

bool startAngleEqual(StatePtr prevState, int new_mp_type, int new_angle){
    bool start_angle_same = true;
            
    if(prevState->mp_type == new_mp_type) {
        start_angle_same = new_angle == prevState->t;
    } else if (new_mp_type == 0) {
        // state is lo res, new is hi res
        
        start_angle_same = new_angle == 2 * prevState->t;
    } else {
        // new is lo res, state is hi res
        start_angle_same = 2 * new_angle == prevState->t;
    }

    return start_angle_same;
}

// Set the parent of each node to be previous in a path from start->goal
bool astar(
  StatePtr startNode,
  StatePtr goalNode,
  vector<StatePtr>& path,
  const vector<MP>& mprims_hi_res,
  const vector<MP>& mprims_lo_res,
  const Map* map,
  int mapNum,
  string output_filename,
  bool threeD)
{
    clock_t t1, t2;
    double time;
    t1 = clock();

    if(!map->isFree(goalNode->x, goalNode->y)) {
        cerr << "Goal post is obstacle" << endl;
        return false;
    }
    path.clear();

    // Priority Queue
    priority_queue<shared_ptr<State>, std::vector<shared_ptr<State>>, StateCompare> pq;

    // Add initial state to PQ
    startNode->h = INFINITY;
    pq.push(startNode);

    // Closed Set
    stateSet closed; 

    bool found_goal = false;
    StatePtr found_goal_state;
    StatePtr closest_goal_state = startNode;
    int c = 0;
    int c_limit = 1000000; // Time out after ~20 seconds

    // Start search
    while (!pq.empty() && !found_goal && c < c_limit)
    {
        c++;
        // Pop state
        StatePtr state = pq.top();
        pq.pop();
        // printf("Searching iter %d:", c); state->print();
        if(c % 10000 == 0) {
            printf("Searching iter %d:", c); state->print();
        } 
        // Check if goalPose
        if (state->x == goalNode->x && state->y == goalNode->y && state->t == goalNode->t && (!threeD || state->z == goalNode->z))
        {
            printf("FOUND GOAL!\n");
            found_goal = true;
            found_goal_state = state;
            break;
        }
        if (state->h < closest_goal_state->h) closest_goal_state = state;
        // Skip if state has alread been searched
        bool isClosed = closed.find(state) != closed.end();
        if (isClosed){
            continue;
        }

        // Choose between hi and lo res
        // Use lo res when getting close to goal or navigating through obstacles.
        // Must be an angle that is available in the lo res 
        bool closeToGoal = hypot(state->x-goalNode->x, state->y-goalNode->y) < HI_RES_THRESH; 
        bool isHighRes = closeToGoal; 

        // if(isHighRes) printf("Using hi res on iter %d\n", c);
        // TODO: Dist to obs
        const vector<MP>& mprims = isHighRes ? mprims_hi_res : mprims_lo_res;
        for (auto mp : mprims)
        {
            // Add neighbor if the MP start_angle is equal to current state's angle
            int new_mp_type = isHighRes ? 0 : 1;
            bool start_angle_same = startAngleEqual(state, new_mp_type, mp.startangle_c);

            if(start_angle_same) {
                int newx = state->x + mp.endpose.x;
                int newy = state->y + mp.endpose.y;
                int newz = threeD ? state->z + mp.endpose.z : 0;
                int newth = mp.endpose.theta;
                if ((!threeD && map->isFree(newx, newy) && mp.isFree(map)) 
                    || (threeD && map->isAbove(newx, newy, newz) && mp.isAbove(map, state->x, state->y, state->z)))
                {
                    double g = state->g + max(1, mp.cost_mult);
                    double h = computeH(newx, newy, newth, newz, goalNode, isHighRes ? 16 : 8, threeD); // TODO: fix angle disc
                    StatePtr new_s = make_shared<State>(newx, newy, newth, g, h, state, mp.ID, new_mp_type, newz);
                    // printf("New state:"); new_s->print();
                    pq.push(new_s);         
                }
            }   
        }
        // Mark state as closed
        closed.insert(state);
    }
    int num_iters = c;
    StatePtr curr;
    if (!found_goal)
    {
        cout << "Failed to find goal" << endl;
        cout << "PQ Size " << pq.size() << " c" << c << endl;
        //cout << "Best h " << closest_goal_state->h << endl;
        printf("Best state: (%d, %d, %d) th=%d, h=%.2f\n", closest_goal_state->x, closest_goal_state->y, closest_goal_state->z, closest_goal_state->t, closest_goal_state->h);
        curr = closest_goal_state;
    } else {
        curr = found_goal_state;
    }

    // Construct list using backpointers
    c = 0; // Prevent timeout.
    while (curr->prev != nullptr && c < c_limit)
    {
        path.push_back(curr);
        curr = curr->prev;
        c++;
    }
    path.push_back(curr); // Insert start state
    reverse(path.begin(), path.end());

    t2 = clock();
    time = (t2 - t1) / (double)CLOCKS_PER_SEC;

    // Write path to file
    ofstream output;
    output.open(output_filename, ofstream::out | ofstream::trunc);
    cout << "Writing path to " << output_filename << endl;
    output << "Map Num: " << mapNum << endl;
    output << "ThreeD: " << threeD << endl;
    output << "Num states: " << path.size() << endl;
    output << "Path cost: " << path[path.size() - 1]->g << endl;
    output << "Num A* iters: " << num_iters << endl;
    output << "Plan Time: " << time << " s\n" << endl;

    if(threeD) output << "x y t mp_id mp_type z" << endl;
    else output << "x y t mp_id mp_type" << endl;
    
    int i;
    for (i = 0; i < path.size(); i++)
    {
        output << path[i]->x << " " << path[i]->y << " " << path[i]->t << " " << path[i]->mp_id << " " << path[i]->mp_type;
        if(threeD) output << " " << path[i]->z;
        output << endl;

        printf("State %d: ", i);
        if(i > 0) path[i]->print();//path[i]->printWithMp(path[i]->mp_type == 0 ? mprims_hi_res : mprims_lo_res, map);
    }

    //output << path[i]->x << " " << path[i]->y << " " << path[i]->t << " " << -1 << " " << -1 << endl;
    output.close();
    cout << "Path length: " << path.size() << endl;
    return true;
}