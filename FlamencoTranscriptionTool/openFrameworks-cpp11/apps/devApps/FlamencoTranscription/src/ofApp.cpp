#include "ofApp.h"

using namespace std;
using namespace essentia;
using namespace essentia::standard;

//--------------------------------------------------------------
void ofApp::setup(){
    
    string audioFilename = ofToDataPath("pavon.wav");
    essentia::init();
    AlgorithmFactory& factory = standard::AlgorithmFactory::instance();
    Algorithm* audio = factory.create("MonoLoader","filename", audioFilename, "sampleRate", 44100);
    vector<Real> audioBuffer;
    audio->output("audio").set(audioBuffer);
    audio->compute();
    
    Algorithm* fc = factory.create("FrameCutter", "frameSize", 2048);
    cout << "-------- computing predominant melody ---------" << endl;
    vector<Real> _pitch, _pitchConfidence;
    pm.compute(audioBuffer, _pitch, _pitchConfidence);
    maxpitch=0.0;
    for (int i=0; i<_pitch.size();i++) {
        pitchD.push_back(_pitch[i]);
        if(_pitch[i]>maxpitch){
            maxpitch=_pitch[i];
        }
    }

    essentia::shutdown();
    cout << "Done" <<endl;
}

//--------------------------------------------------------------
void ofApp::update(){
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    for (int i=0; i<pitchD.size();i++){
        float x=(float(ofGetWidth()-100)/float(pitchD.size())/2)*i+50;
        float y;
        if (pitchD[i]>0){
            y=(ofGetHeight()*0.75+120)-(pitchD[i]/maxpitch)*240;
            ofCircle(x, y, 3);
        }
        
        
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    /*
    if ( TO.lock() ) {
        dirThread = true;
        cout << "\ndirThread = " << dirThread;
        TO.unlock();
    }
     */
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

