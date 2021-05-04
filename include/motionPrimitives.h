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

class Vec4 {
public:
    float x, y, theta, z;
    Vec4(){}
    Vec4(float x, float y, float theta, float z):x(x), y(y), theta(theta), z(z){}
};

union Vec {
    Vec3 endpose_3;
    Vec4 endpose_4;
};

class MP {
public:
    int ID;
    int startangle_c;
    Vec3 endpose;
    int cost_mult;
    bool threeD;
    vector<Vec3> intermediate_poses;

    MP(){}

    MP(int ID, int startangle_c, Vec3 endpose, int cost_mult, vector<Vec3> intermediate_poses):
        ID(ID),  startangle_c(startangle_c), endpose(endpose), cost_mult(cost_mult), intermediate_poses(intermediate_poses) 
    {}

    bool isFree(Map m);
};

vector<MP> MPrims_highres();
vector<MP> MPrims_lowres();





