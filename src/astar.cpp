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

double computeH(int x1, int y1, int th1, const StatePtr s2, int num_Angle_Discretizations) 
{
    double dist = hypot(x1- s2->x, y1 - s2->y);

    double s = abs(th1 - s2->t);
    double angleDiff = min(s, num_Angle_Discretizations - s);

    double angleScale = dist == 0 ? 10 : (10/dist);
    // printf("Dist = %.2f, Angle Scale = %.2f, s = %.2f, h=%.2f\n", dist, angleScale, s, dist+angleScale*s);
    return dist + angleScale * s;
}

// Set the parent of each node to be previous in a path from start->goal
bool astar(
  StatePtr startNode,
  StatePtr goalNode,
  vector<StatePtr>& path,
  const vector<MP>& mprims_hi_res,
  const vector<MP>& mprims_lo_res,
  const Map* map,
  string output_filename)
{
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
    int c_limit = 50000; // Time out after ~20 seconds

    // Start search
    while (!pq.empty() && !found_goal && c < c_limit)
    {
        c++;
        // Pop state
        StatePtr state = pq.top();
        pq.pop();
        if(c % 100 == 0) {
            printf("Searching iter %d:", c); state->print();
        } 
        // Check if goalPose
        if (state->x == goalNode->x && state->y == goalNode->y && state->t == goalNode->t)
        {
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
        
        bool closeToGoal = hypot(state->x-goalNode->x, state->y-goalNode->y);
        bool angleDisc = state->t % 2 == 0; // TODO: Should we store theta in degress so that we can check if they match
        // TODO: Dist to obs
        const vector<MP>& mprims = (closeToGoal || !angleDisc) ? mprims_hi_res : mprims_lo_res;
        for (auto mp : mprims)
        {
            // Add neighbor if the MP start_angle is equal to current state's angle
            if(mp.startangle_c == state->t) {
                 // TODO: I don't think this is the right way to make new states:
                int newx = state->x + mp.endpose.x;
                int newy = state->y + mp.endpose.y;
                int newth = mp.endpose.theta;
                if (map->isFree(newx, newy))
                {
                    double g = state->g + mp.cost_mult;
                    double h = computeH(newx, newy, newth, goalNode, 16); // TODO: fix angle disc
                    StatePtr new_s = make_shared<State>(newx, newy, newth, g, h, state, mp.ID);
                    // printf("New state:"); new_s->print();
                    pq.push(new_s);         
                }
            }   
        }
        // Mark state as closed
        closed.insert(state);
    }

    StatePtr curr;
    if (!found_goal)
    {
        cout << "Failed to find goal" << endl;
        cout << "PQ Size " << pq.size() << " c" << c << endl;
        //cout << "Best h " << closest_goal_state->h << endl;
        printf("Best state: (%d, %d, %d) h=%.2f\n", closest_goal_state->x, closest_goal_state->y, closest_goal_state->t, closest_goal_state->h);
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

    // Write path to file
    ofstream output;
    output.open(output_filename, ofstream::out | ofstream::trunc);
    cout << "Writing path to " << output_filename << endl;;
    for (int i = 0; i < path.size()-1; i++)
    {
        output << path[i]->x << " " << path[i]->y << " " << path[i]->t << " " << path[i+1]->mp_id << endl;
    }
    output.close();
    cout << "Path length: " << path.size() << endl;
    return true;
}