#include "audio.h"

//--------------------------------------------------------------
void audio::loadAudio(){
    ofFileDialogResult audioFile=ofSystemLoadDialog();
    path=audioFile.getPath();
    env.clear();
    wav.load(audioFile.getPath(),0);
    length=wav.length;
    int ch=wav.myChannels;
    wav.setPosition(0.0);
    float av=0.0;
    int count=0;
    float max=0.0;
    for (int i=0; i<length/ch; i++){
        double s=wav.play();
        av+=s*s;
        if (count==100){
            env.push_back(sqrt(av/100.0));
            if (sqrt(av/100.0)>max){
                max=sqrt(av/100.0);
            }
            count=0;
            av=0.0;
        }
        count++;
    }
    
    for (int i=0; i<env.size(); i++){
        env[i]/=max;
        //cout << env[i] << endl;
    }
    wav.setPosition(0.0);
}

//--------------------------------------------------------------
void audio::drawAudio(){
    
    //waveform
    ofEnableAlphaBlending();
    ofSetColor(25,25,25,50);
    float x1, x2, y1, y2,y12, y22;
    x1=ofGetWidth()/10+4;
    y1=ofGetHeight()*0.2;
    y12=ofGetHeight()*0.2;
    for (int i=0; i<env.size();i++){
        x2=(float(4*ofGetWidth()/5)/float(env.size()))*i+ofGetWidth()/10+4.0;
        y2=(ofGetHeight()*0.2)-(env[i]*35);
        y22=(ofGetHeight()*0.2)+(env[i]*35);
        ofLine(x1, y1, x2, y2);
        ofLine(x1, y12, x2, y22);
        x1=x2;
        y1=y2;
        y12=y22;
    }
    
    // cursor
    ofSetLineWidth(4.0);
    ofSetColor(5,5,5,150);
    float xc=float(4*ofGetWidth()/5)*(wav.position/(float(length)))+4.0+ofGetWidth()/10;
    ofLine(xc, ofGetHeight()*0.2-40, xc, ofGetHeight()*0.2+40);
    ofSetColor(255,255,255,255);
    
}

