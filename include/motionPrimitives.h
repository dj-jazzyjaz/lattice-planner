#pragma once
#include <iostream>
#include <vector>
#include "./map.h"

using namespace std;

class Vec4 {
public:
    float x, y, theta, z;
    Vec4(){}
    Vec4(float x, float y, float theta, float z=0){
        this->x = x;
        this->y = y;
        this->theta = theta;
        this->z = z;
    }
};


class MP {
public:
    int ID;
    int startangle_c;
    Vec4 endpose;
    int cost_mult;
    vector<Vec4> intermediate_poses;

    MP(){}
    MP(int ID, int startangle_c, Vec4 endpose, int cost_mult, vector<Vec4> intermediate_poses):
        ID(ID),  startangle_c(startangle_c), endpose(endpose), cost_mult(cost_mult), intermediate_poses(intermediate_poses) 
    {}

    bool isFree(const Map* m);
    bool isAbove(const Map* m, int x, int y, int z);
};

vector<MP> MPrims_highres();
vector<MP> MPrims_lowres();
vector<MP> MPrims_highres3D();
vector<MP> MPrims_lowres3D();





