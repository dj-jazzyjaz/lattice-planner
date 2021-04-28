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


// Set the parent of each node to be previous in a path from start->goal
bool astar(
  StatePtr startNode,
  StatePtr goalNode,
  vector<StatePtr>& path,
  const vector<MP>& mprims,
  const Map* map)
{
    path.clear();

    // Priority Queue
    priority_queue<shared_ptr<State>, std::vector<shared_ptr<State>>, StateCompare> pq;

    // Add initial state to PQ
    pq.push(startNode);

    // Closed Set
    stateSet closed; 

    bool found_goal = false;
    StatePtr found_goal_state;
    int c = 0;
    int c_limit = 20000000; // Time out after ~20 seconds

    // Start search
    while (!pq.empty() && !found_goal && c < c_limit)
    {
        c++;
        // Pop state
        StatePtr state = pq.top();
        pq.pop();

        // Check if goalPose
        if (state->x == goalNode->x && state->y == goalNode->y && state->t == goalNode->t)
        {
            found_goal = true;
            found_goal_state = state;
            break;
        }

        // Skip if state has alread been searched
        bool isClosed = closed.find(state) != closed.end();
        if (isClosed){
            continue;
        }
        
        for (auto mp : mprims)
        {
            // Add neighbor if the MP start_angle is equal to current state's angle
            if(mp.startangle_c == state->t) {
                 // TODO: I don't think this is the right way to make new states:
                int newx = state->x + mp.x;
                int newy = state->y + mp.y;
                int newth = mp.endpose;
                if (map->isFree(newx, newy))
                {
                    double g = state->g;
                    double h = 0; // TODO: create heuristic
                    StatePtr new_s = make_shared<State>(newx, newy, newth, g, h, state);
                    pq.push(new_s);         
                }
            }
            
        }
        // Mark state as closed
        closed.insert(state);
    }

    if (!found_goal)
    {
        cout << "Failed to find goal" << endl;
        cout << "PQ Size " << pq.size() << " c" << c << endl;
        return;
    }

    // Construct list using backpointers
    vector<shared_ptr<State>> path;
    shared_ptr<State>curr = found_goal_state;
    c = 0; // Prevent timeout.
    while (curr->prev != nullptr && c < c_limit)
    {
        path.push_back(curr);
        curr = curr->prev;
        c++;
    }
    path.push_back(curr); // Insert start state
    reverse(path.begin(), path.end());

    // Print Path
    cout << "Start";
    init_state->print();
    for (int i = 0; i < path.size(); i++)
    {
        path[i]->print();
    }
    cout << "Path length: " << path.size() << endl;
}