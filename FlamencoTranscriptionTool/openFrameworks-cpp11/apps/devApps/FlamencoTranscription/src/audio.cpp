#include "audio.h"

//--------------------------------------------------------------
void audio::loadAudio(){
    ofFileDialogResult audioFile=ofSystemLoadDialog();
    path=audioFile.getPath();
    env.clear();
    wav.load(audioFile.getPath(),1);
    wav.getLength();
    wav.position=0;
    float av=0.0;
    int count=0;
    float max=0.0;
    for (int i=0; i<wav.length; i++){
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
    length=wav.length;
}

//--------------------------------------------------------------
void audio::drawAudio(){
    
}