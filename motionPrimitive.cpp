#include <iostream>
#include <vector>

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
    {
        
    }
};

int main() {
    
    MP mp = MP(0, 
       0, 
       Vec3(1, 2, 3),
       1,
       {Vec3(4, 5, 6), Vec3(7, 8, 9)}
    );

    cout << "x = " << mp.intermediate_poses.front().x << endl;
    cout.flush();

    return 0;
}