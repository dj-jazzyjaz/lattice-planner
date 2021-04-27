#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main() {
    filebuf fb;

    int width, height; 
    std::vector<int> vect;

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

    int* map = new int[width*height];
    int idx = 0;
    for (int i; inFile >> i;) {
        //vect.push_back(i);    
        map[idx] = i;
        idx++;
        if (inFile.peek() == ',')
            inFile.ignore();
    }

    for(int i = 0; i < width*height; i++)
        cout << map[i] << " ";
}
