#include <iostream>
#include "ofxMaxim.h"
#include "ofMain.h"



class audio {
public:
    void loadAudio();
    void drawAudio();
    string path;
    std::vector<float> env;
    ofxMaxiSample wav;
    int length;
};

