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
#include <time.h>
#include <functional>
#include <memory>
#include "map.h"

using namespace std;
class State;
typedef shared_ptr<State> StatePtr;

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
    StatePtr prev; // Back pointer to previous state
    int mp_id; // ID of motion primitive that leads to this state

    State(int x, int y, int t, double g, double h, shared_ptr<State>prev, int mp_id)
    {
        this->x = x;
        this->y = y;
        this->t = t;
        this->g = g;
        this->h = h;
        this->f = g + h;
        this->prev = prev;
        this->mp_id = mp_id;
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

    void printShort() {
        cout << "(" << this->x << ", " << this->y << ", " << this->t << ", " << this->mp_id << ")" << endl;
    }
};

template <class T>
inline void hash_combine(std::size_t &seed, const T &v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

/* DATA STRUCTURE FOR COORD (x, y)
Hashes a state based on x, y. Therefore, two states with different theta but same
x, y will be hashed to the same value.
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
Hashes a state based on x, y, theta
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


// TODO: We can create a 2D heurstic with A* on (x, y) to use for 3D (x, y, t)
static void heuristic2D(
    Map* map,
    int poseX,
    int poseY,
    gridMap *gMap
)
{
    /*cout << "heuristic 2d" << endl;
    // Priority Queue
    priority_queue<shared_ptr<State>, std::vector<shared_ptr<State>>, StateCompare> pq;

    // Initialize with all states
    StatePtr init_state = make_shared<State>(poseX, poseY, 0, 0, 0, nullptr, -1);
    pq.push(init_state);

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
    }*/
}