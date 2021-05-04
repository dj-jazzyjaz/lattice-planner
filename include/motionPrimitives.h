#pragma once
#include <iostream>
#include <vector>
#include "./map.h"

using namespace std;

class Vec3 {
public:
    float x, y, theta;
    Vec3(){}
    Vec3(float x, float y, float theta){
        this->x = x;
        this->y = y;
        this->theta = theta;
    }
};


class MP {
public:
    int ID;
    int startangle_c;
    Vec3 endpose;
    int cost_mult;
    vector<Vec3> intermediate_poses;

    MP(){}
    MP(int ID, int startangle_c, Vec3 endpose, int cost_mult, vector<Vec3> intermediate_poses):
        ID(ID),  startangle_c(startangle_c), endpose(endpose), cost_mult(cost_mult), intermediate_poses(intermediate_poses) 
    {}

    bool isFree(Map m);
};

vector<MP> MPrims_highres();
vector<MP> MPrims_lowres();





