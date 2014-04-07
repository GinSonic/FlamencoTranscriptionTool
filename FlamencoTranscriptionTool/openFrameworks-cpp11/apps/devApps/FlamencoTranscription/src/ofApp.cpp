#include "ofApp.h"

using namespace std;
using namespace essentia;
using namespace essentia::standard;

//--------------------------------------------------------------
void ofApp::setup(){
    
    // audio load and playback
    playFlag=false;
    loadFlag=false;
    
    // predominant melody calculation
    T.pm_calc=false;
    T.startThread(true, false);
}

//--------------------------------------------------------------
void ofApp::update(){
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    if (T.pm.status=="compute pitch salience"){
        ofDrawBitmapString(ofToString(T.pm.progress), ofGetWidth()/2, ofGetHeight()/2);
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    if(key=='p' && loadFlag){
        if ( T.lock() ) {
            T.pm_calc = true;
            T.unlock();
        }
    }
    
    if(key=='l'){
        wave.loadAudio();
    }
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}
//--------------------------------------------------------------
void ofApp::exit() {
    
    // stop the thread
    T.stopThread();
}
