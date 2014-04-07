#pragma once

#include "ofMain.h"
#include "PMThread.h"
#include "audio.h"

class ofApp : public ofBaseApp{
    
public:
    
    // main functions
    void setup();
    void update();
    void draw();
    void audioRequested(float * output, int bufferSize, int nChannels);
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    void exit();
    
    // predom. melody calculation
    PMThread T;
    vector<float> pitchD;
    float maxpitch;
    
    // audio playback
    audio wave;
    bool loadFlag;
    bool playFlag;
    int bufferSize;
    int sampleRate;

    // GUI
    ofImage bg;
    
    
    
    
    
    
};
