#include <iostream>
#include "ofxCsv.h"

class pm_from_csv {
public:
    void loadCSV();
    void drawCSV(float position, float length);
    vector<float> pitch;
    wng::ofxCsv csv;
    int numRows;
    float maxpitch, minpitch;
};