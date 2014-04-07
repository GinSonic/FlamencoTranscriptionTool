#include "ofApp.h"

using namespace std;
using namespace essentia;
using namespace essentia::standard;

//--------------------------------------------------------------
void ofApp::setup(){
    
    // init
    ofSetWindowShape(1000, 600);
    bg.loadImage(ofToDataPath("GUI.png"));

    // audio load and playback
    playFlag=false;
    loadFlag=false;
    
    // predominant melody calculation
    T.pm_calc=false;
    T.startThread(true, false);
    
    // audio setup
    sampleRate 	= 44100;
	bufferSize	= 4096;
    ofSoundStreamSetup(2,0,this, sampleRate, bufferSize, 4);
}

//--------------------------------------------------------------
void ofApp::update(){
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    // init
    ofSetLineWidth(3);
    bg.draw(0, 0,ofGetWidth(), ofGetHeight());
    
    // waveform
    //ofSetColor(50, 50, 50);
    if (loadFlag){
        wave.drawAudio();
    }
    
    // predominant melody computation progress
    if (T.pm.status=="compute pitch salience"){
        ofDrawBitmapString(ofToString(T.pm.progress), ofGetWidth()/2, ofGetHeight()/2);
    }
}
//--------------------------------------------------------------
void ofApp::audioRequested(float * output, int bufferSize, int nChannels){
    if ((playFlag==true)&(loadFlag==true)){
        for (int i=0; i<bufferSize; i++){
            double s=wave.wav.playOnce();
            output[i*nChannels    ] = s;
			output[i*nChannels + 1] = s;
        }
    }
    if (wave.wav.position>=wave.wav.length){
        wave.wav.setPosition(0.0);
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
        loadFlag=false;
        wave.loadAudio();
        loadFlag=true;
    }
    
    if(key==' '){
        playFlag=!playFlag;
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
    
    cout << x << endl;
    // set playback position
    if (x>ofGetWidth()/10 && x<9*ofGetWidth()/10 && y<0.2*ofGetHeight()+40 &&  y>0.2*ofGetHeight()-40){
        wave.wav.setPosition((float(x)-float(ofGetWidth())*0.1)/(float(ofGetWidth())*0.8));
        cout << (float(x)-float(ofGetWidth())*0.1)/(float(ofGetWidth())*0.8) << endl;
    }
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
