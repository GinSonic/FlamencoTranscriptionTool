#include <essentia/algorithmfactory.h>
#include <essentia/essentiamath.h>
#include "predominantmelody.h"

using namespace std;
using namespace essentia;
using namespace essentia::standard;

class PMThread : public ofThread {
    
    void threadedFunction() {

            while(isThreadRunning()) {
            
                if (pm_calc){
                    cout << "computing pred mel..."<< endl;
                    essentia::init();
                    string audioFilename = ofToDataPath("pavon.wav");
                    AlgorithmFactory& factory = standard::AlgorithmFactory::instance();
                    Algorithm* audio = factory.create("MonoLoader","filename", audioFilename, "sampleRate", 44100);
                    vector<Real> audioBuffer;
                    audio->output("audio").set(audioBuffer);
                    audio->compute();
                    
                    Algorithm* fc = factory.create("FrameCutter", "frameSize", 2048);
                    cout << "-------- computing predominant melody ---------" << endl;
                    vector<Real> _pitch, _pitchConfidence;
                    pm.compute(audioBuffer, _pitch, _pitchConfidence);
                    float maxpitch=0.0;
                    /*
                    for (int i=0; i<_pitch.size();i++) {
                        pitchD.push_back(_pitch[i]);
                        if(_pitch[i]>maxpitch){
                            maxpitch=_pitch[i];
                        }
                    }
                     */
                    essentia::shutdown();
                    cout << "Done..." << endl;
                    pm_calc=false;
                }
            }
        
    }
public:
    bool pm_calc;
    essentia::standard::PredominantMelody pm;
};

        /*
         
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
         
         
         */