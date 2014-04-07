#include "pm_from_csv.h"

//--------------------------------------------------------------
void pm_from_csv::loadCSV(){
    ofFileDialogResult csvFile=ofSystemLoadDialog();
    string path=csvFile.getPath();
    maxpitch=0.0;
    minpitch=10000;
    csv.loadFile(path);
    numRows=csv.numRows;
    for (int i=0; i<numRows; i++){
        pitch.push_back(ofToFloat(csv.data[i][1]));
        if (pitch[i]>0){
            if (pitch[i]>maxpitch){
                maxpitch=pitch[i];
            }
            if (pitch[i]<minpitch){
                minpitch=pitch[i];
            }
        }
    }
}

//--------------------------------------------------------------
void pm_from_csv::drawCSV(float position, float length){
    
    ofEnableAlphaBlending();
    
    // melodic contour
    ofFill();
    ofSetColor(25,25,25,50);
    float x2, y2;
    for (int i=0; i<pitch.size();i++){
        if (pitch[i]>0){
            x2=(float(4*ofGetWidth()/5)/float(pitch.size()))*i+ofGetWidth()/10+4.0;
            y2=((float(ofGetHeight())*0.84-((pitch[i]-minpitch)/(maxpitch-minpitch)*0.5333*float(ofGetHeight()))));
            ofCircle(x2, y2, 2.0);
        }
    }
    
    // cursor
    ofSetLineWidth(4.0);
    ofSetColor(5,5,5,150);
    float xc=float(4*ofGetWidth()/5)*(position/(float(length)))+4.0+ofGetWidth()/10;
    ofLine(xc, ofGetHeight()*0.84, xc, ofGetHeight()*0.3);
    ofSetColor(255,255,255,255);
    
    ofSetColor(255,255,255,255);
}