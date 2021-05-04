#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <cfloat>

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
        inFile.open(fileName);
        if(inFile.bad() || !inFile.is_open()) {
            cerr << "Bad open" << endl;
            return;
        }
        inFile.ignore();
        getline(inFile, line);
        inFile >> width;
        inFile.ignore();
        inFile >> height;
        
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

    int getValue(int x, int y) const  {
        if(!inDims(x, y)) {
            cerr << "Not within dims" << endl;
            return 0;
        }
        else {
            return map_data[x + y*width];
        }
    }

    bool isFree(int x, int y) const {
        // Returns true is (x, y) is within the map and free space
        return inDims(x, y) && map_data[x + y*width] <= obstacle_threshold;
    }

    int computeDistFromObstacle(int x, int y) const {
        if(!inDims(x, y)) return 0;
        if(!isFree(x, y)) return 0;
        // Todo: could make this better
        double nearestDist = DBL_MAX;
        for(int r = 0; r < height; r++) {
            for(int c = 0; c < height; c++) {
                if(!isFree(r, c)) {
                    if(hypot(r-y, c-x) < nearestDist) nearestDist = hypot(r-y, c-x);
                }
            }
        }
    }

    bool inDims(int x, int y) const {
        return x >= 0 && y >= 0 && x < width && y < height;
    }

    void printMap() {
        printf("*** Printing map of size %d by %d *** \n", height, width);
        for(int r = 0; r < height; r++) {
            for(int c = 0; c < width; c++) {
                printf("%d ", getValue(c, r));
            }
            printf("\n");
        }
    }
};
