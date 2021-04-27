#pragma once
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

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * XSIZE + (X - 1))

#define NUMOFDIRS 8

using namespace std;
class StateCompare
{
public:
    template <typename T>
    bool operator()(shared_ptr<T> a, shared_ptr<T> b)
    {
        return (*a) < (*b);
    }
};

class State
{
public:
    int x;
    int y;
    int t;
    double g;    // Current cost
    double h;    // Heurstic
    double f;    // Cost
    shared_ptr<State>prev; // Back pointer to previous state

    State(int x, int y, int t, double g, double h, shared_ptr<State>prev)
    {
        this->x = x;
        this->y = y;
        this->t = t;
        this->g = g;
        this->h = h;
        this->f = g + h;
        this->prev = prev;
    }

    State() {}

    // Compare states based off f (Not sure why signs are reversed)
    bool operator<(const State &s2) const
    {
        return this->f > s2.f;
    }

    bool operator>(const State &s2) const
    {
        return this->f < s2.f;
    }

    void print()
    {
        cout << "(" << this->x << "," << this->y << ") t=" << this->t << " g=" << this->g << " h=" << this->h << " f=" << this->f << endl;
    }
};

template <class T>
inline void hash_combine(std::size_t &seed, const T &v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

/* DATA STRUCTURE FOR HMAP
*/
typedef std::tuple<int, int> Coord;

struct coordHash : std::unary_function<Coord, std::size_t>
{
    std::size_t operator()(Coord const &e) const
    {
        std::size_t seed = 0;
        hash_combine(seed, get<0>(e));
        hash_combine(seed, get<1>(e));
        return seed;
    }
};

struct coordEqual_to : std::binary_function<Coord, Coord, bool>
{
    bool operator()(Coord const &c1, Coord const &c2) const
    {
        return (get<0>(c1) == get<0>(c2) &&
                get<1>(c1) == get<1>(c2));
    }
};

// Maps (x, y) to an int (used to Heuristic Map)
typedef unordered_map<Coord, int, coordHash, coordEqual_to> gridMap;

/* DATA STRUCTURE FOR STATES (X, Y, T)
*/
struct stateHash : std::unary_function<shared_ptr<State>, std::size_t>
{
    std::size_t operator()(shared_ptr<State>const s) const
    {
        std::size_t seed = 0;
        hash_combine(seed, s->x);
        hash_combine(seed, s->y);
        hash_combine(seed, s->t);
        return seed;
    }
};

struct stateEqual_to : std::binary_function<shared_ptr<State>, shared_ptr<State>, bool>
{
    bool operator()(shared_ptr<State>const s1, shared_ptr<State>const s2) const
    {
        return (s1->x == s2->x &&
                s1->y == s2->y &&
                s1->t == s2->t);
    }
};

struct stateXYHash : std::unary_function<shared_ptr<State>, std::size_t>
{
    std::size_t operator()(shared_ptr<State>const s) const
    {
        std::size_t seed = 0;
        hash_combine(seed, s->x);
        hash_combine(seed, s->y);
        return seed;
    }
};

struct stateXYEqual_to : std::binary_function<shared_ptr<State>, shared_ptr<State>, bool>
{
    bool operator()(shared_ptr<State>const s1, shared_ptr<State>const s2) const
    {
        return (s1->x == s2->x &&
                s1->y == s2->y);
    }
};

typedef unordered_set<shared_ptr<State>, stateHash, stateEqual_to> stateSet;
typedef unordered_set<shared_ptr<State>, stateXYHash, stateXYEqual_to> stateXYSet;

bool inline checkInDims(int x, int y, int size_x, int size_y)
{
    return x > 0 && y > 0 && x <= size_x && y <= size_y;
}

static void printQueue(priority_queue<shared_ptr<State>, std::vector<shared_ptr<State>>, StateCompare> pq)
{
    vector<shared_ptr<State>> L;
    int max = 20;
    int c = 0;
    while (!pq.empty() && c < max)
    {
        c++;
        pq.top()->print();
        L.push_back(pq.top());
        pq.pop();
    }
    for (int i = 0; i < L.size(); i++)
    {
        pq.push(L[i]);
    }
}

static void heuristic2D(
    double *map,
    int collision_thresh,
    int x_size,
    int y_size,
    int target_steps,
    double *target_traj,
    gridMap *gMap

)
{
    cout << "heuristic 2d" << endl;
    //8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};

    // Priority Queue
    priority_queue<shared_ptr<State>, std::vector<shared_ptr<State>>, StateCompare> pq;

    // Initialize with all states
    int poseX, poseY;
    for (int i = 0; i < target_steps; i++)
    {
        poseX = (int)target_traj[i];
        poseY = (int)target_traj[i + target_steps];
        shared_ptr<State> init_state = make_shared<State>(poseX, poseY, 0, 0, 0, nullptr);
        pq.push(init_state);
    }

    while (!pq.empty())
    {
        // Pop state
        shared_ptr<State>state = pq.top();
        pq.pop();

        bool isClosed = gMap->find(Coord(state->x, state->y)) != gMap->end();
        if (isClosed)
            continue;

        // Add neighbors to the list
        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = state->x + dX[dir];
            int newy = state->y + dY[dir];

            // Check if cell is occupied
            if (checkInDims(newx, newy, x_size, y_size)) // TODO change this
            {
                int costCell = (int)map[GETMAPINDEX(newx, newy, x_size, y_size)];
                if (costCell >= collision_thresh)
                {
                    gMap->insert(make_pair<Coord, int>(Coord(newx, newy), 99999999));
                    continue;
                }
                double g = state->g + costCell;

                double h = 0;
                shared_ptr<State>new_s = make_shared<State>(newx, newy, 0, g, h, state);
                pq.push(new_s);
            }
        }
        // Mark state as closed
        gMap->insert(make_pair<Coord, int>(Coord(state->x, state->y), (int)state->g));
    }
}