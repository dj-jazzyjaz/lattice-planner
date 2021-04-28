#pragma once
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

class Map{
public:
    int width, height;
    int* map_data;
    int obstacle_threshold = 5;
    Map(){}

    Map(string fileName) {
        string line;
        std::ifstream inFile;
        inFile.open("map0.txt");
        inFile.ignore();
        inFile >> width;
        inFile.ignore();
        inFile >> height;
        
        cout << "w, h" << width << " " << height << endl;
        getline(inFile, line);
        inFile.ignore();
        map_data = new int[width*height];
        int idx = 0;
        for (int i; inFile >> i;) {
            //vect.push_back(i);    
            map_data[idx] = i;
            idx++;
            if (inFile.peek() == ',')
                inFile.ignore();
        }
    }

    int getValue(int x, int y) {
        if(!inDims(x, y)) {
            cerr << "Not within dims" << endl;
            return 0;
        }
        else {
            return map_data[x + y*width];
        }
    }

    bool isFree(int x, int y) {
        // Returns true is (x, y) is within the map and free space
        return inDims(x, y) && map_data[x + y*width] <= obstacle_threshold;
    }

    bool inDims(int x, int y) {
        return x > 0 && y > 0 && x <= width && y <= height;
    }
};
