/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <queue>
#include <vector>
#include <algorithm>
#include <set>
#include <iostream>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <mex.h>
#include <time.h>
#include "heuristic.h"

/* Input Arguments */
#define MAP_IN prhs[0]
#define ROBOT_IN prhs[1]
#define TARGET_TRAJ prhs[2]
#define TARGET_POS prhs[3]
#define CURR_TIME prhs[4]
#define COLLISION_THRESH prhs[5]

/* Output Arguments */
#define ACTION_OUT plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * XSIZE + (X - 1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

using namespace std;
int path_update_time = 0;
int path_count = 0;
int temp = 0;

static inline double getHCost(int stateX, int stateY, int stateT, gridMap *hMap)
{
    return (*hMap)[Coord(stateX, stateY)];
}

static inline bool isOnTargetTraj(shared_ptr<State>s, stateSet *targetSet)
{
    return (*targetSet).find(s) != (*targetSet).end();
}

static inline bool isOnTargetXY(shared_ptr<State>s , stateXYSet *targetSetXY) {
    return (*targetSetXY).find(s) != (*targetSetXY).end();
}

static inline bool isOnTargetXYBeforeT(shared_ptr<State>s , stateXYSet *targetSetXY) {
    if ((*targetSetXY).find(s) == (*targetSetXY).end()) return false;
    shared_ptr<State> targetState = *((*targetSetXY).find(s));
    return targetState->t > s->t;
}

// Perform 3D A*
vector<shared_ptr<State>> currentPath;
static void astar3D(
    double *map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int currTime,
    int target_steps,
    double *target_traj,
    gridMap *heuristicMap)
{
    //8-connected grid or stay in place
    int dX[NUMOFDIRS + 1] = {-1, -1, -1, 0, 0, 1, 1, 1, 0};
    int dY[NUMOFDIRS + 1] = {-1, 0, 1, -1, 1, -1, 0, 1, 0};

    // Priority Queue
    priority_queue<shared_ptr<State>, std::vector<shared_ptr<State>>, StateCompare> pq;

    // Create goal states
    stateSet targetSet;
    targetSet.rehash(target_steps);
    stateXYSet targetSetXY;
    int targetStart = 0; // TODO: set this later? 
    for (int i = targetStart; i < target_steps; i++)
    {
        shared_ptr<State> s = make_shared<State>(target_traj[i], target_traj[i + target_steps], i, 0, 0, nullptr);
        targetSet.insert(s);
        targetSetXY.insert(s);
    }

    // Add initial state to PQ
    double h = getHCost(robotposeX, robotposeY, 0, heuristicMap);
    shared_ptr<State>init_state = make_shared<State>(robotposeX, robotposeY, currTime, 0, h, nullptr);
    pq.push(init_state);

    stateXYSet closed; // Closed Set
    bool found_goal = false;
    shared_ptr<State>goal_state;
    int c = 0;
    int c_limit = 20000000; // Time out after ~20 seconds
    while (!pq.empty() && !found_goal && c < c_limit)
    {
        c++;
        // Pop state
        shared_ptr<State>state = pq.top();
        pq.pop();

        // Check if goalPose
        if (isOnTargetTraj(state, &targetSet))
        {
            found_goal = true;
            goal_state = state;
            break;
        }

        // Skip if state is closed but NOT in place, or time > target trajectory time.
        bool isClosed = closed.find(state) != closed.end();
        bool overTime = state->t >= target_steps;
        bool inPlace = state->prev != nullptr && state->x == state->prev->x && state->y == state->prev->y && isOnTargetXYBeforeT(state, &targetSetXY);
        if ((isClosed && !inPlace) || overTime){
            continue;
        }

        if(inPlace) {
            // If robot did not move on last round, do not move on this round.
            int costCell = (int)map[GETMAPINDEX(state->x, state->y, x_size, y_size)];
            double g = state->g + costCell;
            double h = state->h;
            shared_ptr<State>new_s = make_shared<State>(state->x, state->y, state->t + 1, g, h, state);
            pq.push(new_s);         
        } else {
            // Add neighbors to the list
            for (int dir = 0; dir < NUMOFDIRS + 1; dir++)
            {
                int newx = state->x + dX[dir];
                int newy = state->y + dY[dir];
                if (checkInDims(newx, newy, x_size, y_size))
                {
                    int costCell = (int)map[GETMAPINDEX(newx, newy, x_size, y_size)];
                    if (costCell >= collision_thresh)
                        continue;
                    double g = state->g + costCell;
                    double h = 20 * getHCost(newx, newy, state->t + 1, heuristicMap);
                    shared_ptr<State>new_s = make_shared<State>(newx, newy, state->t + 1, g, h, state);
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
    shared_ptr<State>curr = goal_state;
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
    currentPath = path;
    path_update_time = currTime;
}

gridMap hMapObj;
gridMap *hMap = &hMapObj;

static void planner(
    double *map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    double *target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    double *action_ptr)
{
    int planTime = max((x_size * y_size) / 700000, 2);
    if (temp == 0)
    {   cout << "Allocated Plan Time = " << planTime << endl; 
        clock_t t1, t2;
        double time;
        t1 = clock();
        heuristic2D(map, collision_thresh, x_size, y_size, target_steps, target_traj, hMap);
        t2 = clock();
        time = (t2 - t1) / (double)CLOCKS_PER_SEC;
        cout << "Time to compute 2D Heuristic:" << time << "s" << endl;
        temp++;
    }

    if (currentPath.size() == 0 || currentPath[path_count]->t < curr_time)
    {
        // Replan a*
        if(temp > 2) planTime += temp * 2;  // Increase plan time if it failed to plan last time.
        clock_t t1, t2;
        cout << "Begin 3D A*" << endl;
        double time;
        t1 = clock();
        astar3D(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, curr_time + planTime, target_steps, target_traj, hMap);
        t2 = clock();
        time = (t2 - t1) / (double)CLOCKS_PER_SEC;
        cout << "A* time: " << time << endl;
        path_count = 0;
        temp++;
    }

    // No plan, or just replanned
    if (currentPath.size() <= path_count || currentPath[path_count]->t > curr_time)
    {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }

    shared_ptr<State>nextState = currentPath[path_count];
    action_ptr[0] = nextState->x;
    action_ptr[1] = nextState->y;
    path_count++;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])

{

    /* Check for proper number of arguments */
    if (nrhs != 6)
    {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs",
                          "Six input arguments required.");
    }
    else if (nlhs != 1)
    {
        mexErrMsgIdAndTxt("MATLAB:planner:maxlhs",
                          "One output argument required.");
    }

    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double *map = mxGetPr(MAP_IN);

    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if (robotpose_M != 1 || robotpose_N != 2)
    {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidrobotpose",
                          "robotpose vector should be 1 by 2.");
    }
    double *robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];

    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);

    if (targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidtargettraj",
                          "targettraj vector should be M by 2.");
    }
    double *targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;

    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if (targetpose_M != 1 || targetpose_N != 2)
    {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidtargetpose",
                          "targetpose vector should be 1 by 2.");
    }
    double *targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];

    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);

    /* Create a matrix for the return action */
    ACTION_OUT = mxCreateNumericMatrix((mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL);
    double *action_ptr = (double *)mxGetData(ACTION_OUT);

    /* Get collision threshold for problem */
    int collision_thresh = (int)mxGetScalar(COLLISION_THRESH);

    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");

    // Write to an output file

    // Need matlab script that takes in a list of (state, motion primitive) and visualizes them. 
    return;
}