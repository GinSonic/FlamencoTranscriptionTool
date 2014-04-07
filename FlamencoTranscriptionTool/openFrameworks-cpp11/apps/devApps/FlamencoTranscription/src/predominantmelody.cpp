/*
 * Copyright (C) 2006-2013  Music Technology Group - Universitat Pompeu Fabra
 *
 * This file is part of Essentia
 *
 * Essentia is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Affero General Public License as published by the Free
 * Software Foundation (FSF), either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the Affero GNU General Public License
 * version 3 along with this program.  If not, see http://www.gnu.org/licenses/
 */

#include "predominantmelody.h"
#include <Accelerate/Accelerate.h>

using namespace std;

namespace essentia {
namespace standard {


PredominantMelody::PredominantMelody() {

    sampleRate = 44100;
    frameSize = 2048;
    hopSize = 128;
    windowType = "hann";
    zeroPaddingFactor = 4;
    maxSpectralPeaks = 100;

    referenceFrequency = 55.0;
    binResolution = 10.0;
    magnitudeThreshold = 40;
    magnitudeCompression = 1.0;
    numberHarmonics = 20;
    harmonicWeight = 0.8;

    minFrequency = 80.0;
    maxFrequency = 20000.0;

    peakFrameThreshold = 0.9;
    peakDistributionThreshold = 0.9;
    pitchContinuity = 27.5625;
    timeContinuity = 100;
    minDuration = 100;

    voicingTolerance = 0.2;
    filterIterations = 3;
    voiceVibrato = false;
    guessUnvoiced = false;
    
    std::vector<size_t> _contoursStartIndices;
    std::vector<size_t> _contoursEndIndices;
    std::vector<Real> _contoursBinsMean;
    std::vector<Real> _contoursSaliencesTotal;  // total salience (sum of per-frame saliences)
    std::vector<Real> _contoursSaliencesMean;
    std::vector<Real> _contoursBinsStddev;
    
    std::vector<Real> _melodyPitchMean;               // melody pitch mean function
    std::vector <std::pair <int,int> > _duplicates;   // pairs of contour duplicates
    std::vector<size_t> _contoursSelected;    // indices of the selected contours
    std::vector<size_t> _contoursIgnored;     // indices of the ignored contours
    std::vector<size_t> _contoursSelectedInitially;
    std::vector<size_t> _contoursIgnoredInitially;
    
    essentia::init();
    
}

void PredominantMelody::compute(vector<Real> signal, vector<Real>& pitch, vector<Real>& pitchConfidence) {
  if (signal.empty()) {
    pitch.clear();
    pitchConfidence.clear();
    return;
  }
    AlgorithmFactory& factory = standard::AlgorithmFactory::instance();
  // Pre-processing
  vector<Real> frame;
    
    Algorithm* _frameCutter = factory.create("FrameCutter");
    // Pre-processing
    _frameCutter->configure("frameSize", frameSize,"hopSize", hopSize,"startFromZero", false);
    
    Algorithm* _windowing = factory.create("Windowing");
    _windowing->configure("size", frameSize,"zeroPadding", (zeroPaddingFactor-1) * frameSize,"type",windowType);
    
    // Spectral peaks
    Algorithm* _spectralPeaks = factory.create("SpectralPeaks");
    _spectralPeaks->configure("minFrequency", 1,"maxFrequency", 10000,"maxPeaks", maxSpectralPeaks, "sampleRate", sampleRate,"magnitudeThreshold", 0,"orderBy", "magnitude");
    
    // Pitch salience contours
    Algorithm* _pitchSalienceFunction = factory.create("PitchSalienceFunction");
    _pitchSalienceFunction->configure("binResolution", binResolution,"referenceFrequency", referenceFrequency,"magnitudeThreshold", magnitudeThreshold,"magnitudeCompression", magnitudeCompression,"numberHarmonics", numberHarmonics,"harmonicWeight", harmonicWeight);
    
    // exaggerated min/max values to take all peaks
    // independend of the range of salience function
    Algorithm* _pitchSalienceFunctionPeaks = factory.create("PitchSalienceFunctionPeaks");
    _pitchSalienceFunctionPeaks->configure("binResolution", binResolution,"referenceFrequency", referenceFrequency,"minFrequency", 1,"maxFrequency", 20000);
    
    Algorithm* _pitchContours = factory.create("PitchContours");
    _pitchContours->configure("sampleRate", sampleRate,"hopSize", hopSize,"binResolution", binResolution,"peakFrameThreshold", peakFrameThreshold,"peakDistributionThreshold", peakDistributionThreshold,"pitchContinuity", pitchContinuity,"timeContinuity", timeContinuity,"minDuration", minDuration);
    
    
  _frameCutter->input("signal").set(signal);
  _frameCutter->output("frame").set(frame);

  vector<Real> frameWindowed;
  _windowing->input("frame").set(frame);
  _windowing->output("frame").set(frameWindowed);
    
  /*
  // Spectral peaks
  vector<Real> frameSpectrum;
  _spectrum->input("frame").set(frameWindowed);
  _spectrum->output("spectrum").set(frameSpectrum);
   */
    vector<Real> frameSpectrum;
    vector<Real> frameFrequencies;
    vector<Real> frameMagnitudes;
  

  // Pitch salience contours
    vector<Real> frameSalience;
  _pitchSalienceFunction->input("frequencies").set(frameFrequencies);
  _pitchSalienceFunction->input("magnitudes").set(frameMagnitudes);
  _pitchSalienceFunction->output("salienceFunction").set(frameSalience);

  vector<Real> frameSalienceBins;
  vector<Real> frameSalienceValues;
  _pitchSalienceFunctionPeaks->input("salienceFunction").set(frameSalience);
  _pitchSalienceFunctionPeaks->output("salienceBins").set(frameSalienceBins);
  _pitchSalienceFunctionPeaks->output("salienceValues").set(frameSalienceValues);
    
    _spectralPeaks->input("spectrum").set(frameSpectrum);
    _spectralPeaks->output("frequencies").set(frameFrequencies);
    _spectralPeaks->output("magnitudes").set(frameMagnitudes);

  vector<vector<Real> > peakBins;
  vector<vector<Real> > peakSaliences;
    
    int p=0;
    status="compute pitch salience";
  while (true) {
    // get a frame
    _frameCutter->compute();
    if (!frame.size()) {
      break;
    }

    _windowing->compute();
    

    // calculate spectrum
      frameSpectrum.clear();
    powerSpectrum(frameWindowed, frameSpectrum, frameSize*zeroPaddingFactor);

    // calculate spectral peaks
      _spectralPeaks->compute();

    // calculate salience function
    _pitchSalienceFunction->compute();

    // calculate peaks of salience function
    _pitchSalienceFunctionPeaks->compute();

    peakBins.push_back(frameSalienceBins);
    peakSaliences.push_back(frameSalienceValues);
      p++;
      progress=(float(p)*float(hopSize))/float(signal.size());
  }
status="compute predominant melody...";
  // calculate pitch contours
  vector<vector<Real> > contoursBins;
  vector<vector<Real> > contoursSaliences;
  vector<Real> contoursStartTimes;
  Real duration;

  _pitchContours->input("peakBins").set(peakBins);
  _pitchContours->input("peakSaliences").set(peakSaliences);
  _pitchContours->output("contoursBins").set(contoursBins);
  _pitchContours->output("contoursSaliences").set(contoursSaliences);
  _pitchContours->output("contoursStartTimes").set(contoursStartTimes);
  _pitchContours->output("duration").set(duration);

  _pitchContours->compute();
   
  pitchContoursMelody(contoursBins,contoursSaliences, contoursStartTimes, duration, pitch,pitchConfidence);



  //_pitchContoursMelody->compute();

}

void PredominantMelody::powerSpectrum(vector<Real> input, vector<Real>& output, int size){
    float in[size];
    float out[size];
    vDSP_vclr(in, 1, size);
    for (int i=0; i<input.size(); i++){
        in[i]=input[i];
    }
    
    DSPSplitComplex cFrame;
    float realpart[size];
    float imagpart[size];
    cFrame.realp = realpart;
    cFrame.imagp = imagpart;
    
    vDSP_ctoz((DSPComplex *)in, 2, &cFrame, 1, size/2);

    //!!!!
    FFTSetup mFFTSetup;
    mFFTSetup = vDSP_create_fftsetup(log2f(size),FFT_RADIX2);
    
    // Calculate the FFT
    // ( I'm assuming here you've already called vDSP_create_fftsetup() )
    vDSP_fft_zrip(mFFTSetup, &cFrame, 1, log2f(size), FFT_FORWARD);
    
    // Don't need that frequency
    cFrame.imagp[0] = 0.0;
    
    // Scale the data
    float scale = (float) 1.0 / (2 * (float)(size));
    vDSP_vsmul(cFrame.realp, 1, &scale, cFrame.realp, 1, size/2);
    vDSP_vsmul(cFrame.imagp, 1, &scale, cFrame.imagp, 1, size/2);
    
    // Convert the complex data into something usable
    // spectrumData is also a (float*) of size mNumFrequencies
    vDSP_zvabs(&cFrame, 1, out, 1, size);
 
    for (int i=0; i<size; i++){
        output.push_back(out[i]);
    }
    
    vDSP_destroy_fftsetup(mFFTSetup);
}
  
   
void PredominantMelody::pitchContoursMelody(vector<vector<Real> > _contoursBins,vector<vector<Real> >_contoursSaliences, vector<Real> _contoursStartTimes, Real _duration, vector<Real>& _pitch, vector<Real>& _pitchConfidence){
    
    // minimum and maximum allowed cent bins for contours


    
    Real _duplicateMaxDistance = _outlierMaxDistance;
    Real _duplicateMinDistance = (1200.0-50)/binResolution;
    Real _frameDuration = hopSize / sampleRate;
    // 5-second moving average
    int averagerSize = floor(5 / _frameDuration);
    averagerSize = averagerSize % 2 == 0 ? averagerSize + 1 : averagerSize; // make the size odd
    size_t _averagerShift = averagerSize / 2;
    
    Real _vibratoPitchStddev = 40 / binResolution; // 40 cents
    
   
    
    // parameters voice vibrato detection
    // frame size computed given than we need 350ms of audio (as with default settings)
    Real _vibratoSampleRate = sampleRate / hopSize;
    int _vibratoFrameSize = int(0.350 * _vibratoSampleRate);
    int _vibratoHopSize = 1;
    int _vibratoZeroPaddingFactor = 4;
    int _vibratoFFTSize = _vibratoFrameSize * _vibratoZeroPaddingFactor;
    _vibratoFFTSize = pow(2, ceil(log(_vibratoFFTSize)/log(2)));
    Real _vibratoMinFrequency = 5.0;
    Real _vibratoMaxFrequency = 8.0;
    Real _vibratodBDropLobe = 15;
    Real _vibratodBDropSecondPeak = 20;
    
    // conversion to hertz
    Real _centToHertzBase = pow(2, binResolution / 1200.0);
    
       
        // configure algorithms
    AlgorithmFactory& factory3 = standard::AlgorithmFactory::instance();
    Algorithm* _movingAverage = factory3.create("MovingAverage");
    Algorithm* _frameCutter = factory3.create("FrameCutter");
    Algorithm* _windowing = factory3.create("Windowing");
    Algorithm* _spectralPeaks = factory3.create("SpectralPeaks");
        _movingAverage->configure("size", averagerSize);
        _frameCutter->configure("frameSize", _vibratoFrameSize, "hopSize", _vibratoHopSize, "startFromZero", true);
        //_spectrum->configure("size", _vibratoFFTSize);
        _windowing->configure("type", "hann");
        _windowing->configure("zeroPadding", _vibratoFFTSize - _vibratoFrameSize);
        _spectralPeaks->configure("sampleRate", _vibratoSampleRate);
        _spectralPeaks->configure("maxPeaks", 3); // we are only interested in the three most prominent peaks
        _spectralPeaks->configure("orderBy", "magnitude");
    
    if (_duration < 0) {
        throw EssentiaException("PitchContoursMelody: specified duration of the input signal must be non-negative");
    }
    
_numberFrames = (size_t) round(_duration / _frameDuration);
    _numberContours = _contoursBins.size();
    
    if (_numberContours != _contoursSaliences.size() && _numberContours != _contoursStartTimes.size()) {
        throw EssentiaException("PitchContoursMelody: contoursBins, contoursSaliences, and contoursStartTimes input vectors must have the same size");
    }
    
    _pitch.resize(_numberFrames);
    _pitchConfidence.resize(_numberFrames);
    
    // no frames -> empty pitch vector output
    if (!_numberFrames) {
        return;
    }
    
    for (size_t i=0; i<_numberContours; i++) {
        if (_contoursBins[i].size() != _contoursSaliences[i].size()) {
            throw EssentiaException("PitchContoursMelody: contoursBins and contoursSaliences input vectors must have the same size");
        }
        if (_contoursStartTimes[i] < 0) {
            throw EssentiaException("PitchContoursMelody: contoursStartTimes input vector must contain non-negative values");
        }
        for (size_t j=0; j<_contoursBins[i].size(); j++) {
            if (_contoursBins[i][j] < 0) {
                throw EssentiaException("PitchContoursMelody: contour bin numbers must be non-negative");
            }
            if (_contoursSaliences[i][j] < 0) {
                throw EssentiaException("PitchContoursMelody: contour pitch saliences must be non-negative");
            }
        }
    }
    
    // no contours -> zero pitch vector output
    if (_contoursBins.empty()) {
        fill(_pitch.begin(), _pitch.end(), (Real) 0.0);
        fill(_pitchConfidence.begin(), _pitchConfidence.end(), (Real) 0.0);
        return;
    }
    
    // voicing detection
    voicingDetection(_contoursBins, _contoursSaliences, _contoursStartTimes);
    
    // create a list of all possible duplicates
    detectContourDuplicates(_contoursBins);
    
    // filter octave errors and pitch outliers
    _melodyPitchMean.resize(_numberFrames);
    
    for (int i=0; i<filterIterations; i++) {
        computeMelodyPitchMean(_contoursBins);
       removeContourDuplicates();
        computeMelodyPitchMean(_contoursBins);
        removePitchOutliers();
    }
    
    // final melody selection: for each frame, select the peak
    // belonging to the contour with the highest total salience
    
    Real centBin=0, hertz;
    for (size_t i=0; i<_numberFrames; i++) {
        Real maxSalience = 0;
        Real confidence = 0;
        for (size_t j=0; j<_contoursSelected.size(); j++) {
            size_t jj = _contoursSelected[j];
            if (_contoursStartIndices[jj] <= i && _contoursEndIndices[jj] >= i) {
                // current frame belongs to this contour
                size_t shift = i - _contoursStartIndices[jj];
                if (_contoursSaliencesTotal[jj] > maxSalience) {
                    maxSalience = _contoursSaliencesTotal[jj];
                    confidence = _contoursSaliencesMean[jj];
                    centBin = _contoursBins[jj][shift];
                }
            }
        }
        if (maxSalience==0 && guessUnvoiced) {
            for (size_t j=0; j<_contoursIgnored.size(); j++) {
                size_t jj = _contoursIgnored[j];
                if (_contoursStartIndices[jj] <= i && _contoursEndIndices[jj] >= i) {
                    // current frame belongs to this contour
                    size_t shift = i - _contoursStartIndices[jj];
                    if (_contoursSaliencesTotal[jj] > maxSalience) {
                        maxSalience = _contoursSaliencesTotal[jj]; // store salience with negative sign in the case of unvoiced frames
                        confidence = 0.0 - _contoursSaliencesMean[jj];
                        centBin = _contoursBins[jj][shift];
                    }
                }
            }
        }
        
        if(maxSalience != 0) {
            // a peak was found, convert cent bins to Hertz
            // slow formula: _referenceFrequency * pow(2, centBin*_binResolution / 1200.0);
            hertz = referenceFrequency * pow(_centToHertzBase, centBin);
        } else {
            hertz = 0;
        }
        _pitch[i] = hertz;
        _pitchConfidence[i] = confidence;
    }
}
    

    
bool PredominantMelody::detectVoiceVibrato(vector<Real> contourBins, const Real binMean) {
        
        /*
         Algorithm details are taken from personal communication with Justin Salamon. There should be only one (and it should be
         the highest) peak between 5 and 8 Hz, associated with human voice vibrato.  If there is more than 1 peak in this interval,
         we may not be sure in vibrato --> go to search in next frame.
         
         Find the 2nd and the 3rd highest peaks above 8Hz (we don't care in peaks below 5Hz, and they are normally not expected
         to appear). The second peak should be 15 dBs quieter, and the third peak should be 20 dBs quieter than the highest peak.
         If so, the voice peak is prominent enough --> human voice vibrato found in the contour.
         */
        
        if (!voiceVibrato) {
            return false;
        }
        
        // subtract mean from the contour pitch trajectory
        for (size_t i=0; i<contourBins.size(); i++) {
            contourBins[i] -= binMean;
        }
        
        // apply FFT and check for a prominent peak in the expected frequency range for human vibrato (5-8Hz)
    int _vibratoFrameSize;
    int _vibratoHopSize;
    int _vibratoZeroPaddingFactor;
    int _vibratoFFTSize;
    Real _vibratoMinFrequency;
    Real _vibratoMaxFrequency;
    Real _vibratodBDropLobe;
    Real _vibratodBDropSecondPeak;
    
        vector<Real> frame;
        _frameCutter->input("signal").set(contourBins);
        _frameCutter->output("frame").set(frame);
        
        vector<Real> frameWindow;
        _windowing->input("frame").set(frame);
        _windowing->output("frame").set(frameWindow);
        
        vector<Real> vibratoSpectrum;
        //_spectrum->input("frame").set(frameWindow);
        //_spectrum->output("spectrum").set(vibratoSpectrum);
        
        vector<Real> peakFrequencies;
        vector<Real> peakMagnitudes;
        _spectralPeaks->input("spectrum").set(vibratoSpectrum);
        _spectralPeaks->output("frequencies").set(peakFrequencies);
        _spectralPeaks->output("magnitudes").set(peakMagnitudes);
        
        _frameCutter->reset();
        
        while (true) {
            // get a frame
            _frameCutter->compute();
            if (!frame.size()) {
                break;
            }
            
            _windowing->compute();
            //_spectrum->compute();
            powerSpectrum(frameWindow, vibratoSpectrum, _vibratoFFTSize);
            _spectralPeaks->compute();
            
            int numberPeaks = peakFrequencies.size();
            if (!numberPeaks) {
                continue;
            }
            
            if (peakFrequencies[0] < _vibratoMinFrequency || peakFrequencies[0] > _vibratoMaxFrequency) {
                continue;
            }
            
            if (numberPeaks > 1) {  // there is at least one extra peak
                if (peakFrequencies[1] <= _vibratoMaxFrequency) {
                    continue;
                }
                if (20 * log10(peakMagnitudes[0]/peakMagnitudes[1]) < _vibratodBDropLobe) {
                    continue;
                }
            }
            
            if (numberPeaks > 2) {  // there is a second extra peak
                if (peakFrequencies[2] <= _vibratoMaxFrequency) {
                    continue;
                }
                if (20 * log10(peakMagnitudes[0]/peakMagnitudes[2]) < _vibratodBDropSecondPeak) {
                    continue;
                }
            }
            // prominent peak associated with voice is found
            return true;
        }
        return false;
    }
    
void PredominantMelody::voicingDetection(const vector<vector<Real> >& contoursBins,
                                               const vector<vector<Real> >& contoursSaliences,
                                               const vector<Real>& contoursStartTimes) {
        
        _contoursStartIndices.resize(_numberContours);
        _contoursEndIndices.resize(_numberContours);
        _contoursBinsMean.resize(_numberContours);
        _contoursSaliencesTotal.resize(_numberContours);
        _contoursSaliencesMean.resize(_numberContours);
        _contoursBinsStddev.resize(_numberContours);  // TODO make a local variable
    
        _contoursSelected.clear();
        _contoursIgnored.clear();
        
        vector<Real> contoursBinsMin;
        vector<Real> contoursBinsMax;
        contoursBinsMin.resize(_numberContours);
        contoursBinsMax.resize(_numberContours);
        
        // get contour salience and pitch statistics
        for (size_t i=0; i<_numberContours; i++) {
            _contoursBinsMean[i] = mean(contoursBins[i]);
            _contoursBinsStddev[i] = stddev(contoursBins[i], _contoursBinsMean[i]);
            _contoursSaliencesMean[i] = mean(contoursSaliences[i]);
            contoursBinsMin[i] = contoursBins[i][argmin(contoursBins[i])];
            contoursBinsMax[i] = contoursBins[i][argmax(contoursBins[i])];
        }
    Real binsInOctave = 1200.0 / binResolution;
    Real numberBins = floor(6000.0 / binResolution) - 1;
        Real averageSalienceMean = mean(_contoursSaliencesMean);
        Real salienceThreshold = averageSalienceMean - voicingTolerance * stddev(_contoursSaliencesMean, averageSalienceMean);
        Real _minBin = max(0.0, floor(binsInOctave * log2(minFrequency/referenceFrequency) + 0.5));
        Real _maxBin = min(0.0 + numberBins, floor(binsInOctave * log2(maxFrequency/referenceFrequency) + 0.5));
        // voicing detection
        for (size_t i=0; i<_numberContours; i++) {

            // ignore contours with peaks outside of the allowed range
            if (contoursBinsMin[i] >= _minBin && contoursBinsMax[i] <= _maxBin) {

                if (_contoursSaliencesMean[i] >= salienceThreshold || _contoursBinsStddev[i] > _vibratoPitchStddev
                    || detectVoiceVibrato(contoursBins[i], _contoursBinsMean[i]))  {
 
                    _contoursStartIndices[i] = (size_t) round(contoursStartTimes[i] / _frameDuration);
                    _contoursEndIndices[i] = _contoursStartIndices[i] + contoursBins[i].size() - 1;
                    _contoursSaliencesTotal[i] = accumulate(contoursSaliences[i].begin(), contoursSaliences[i].end(), 0.0);
                    _contoursSelected.push_back(i);
                }
                else {
                    if (guessUnvoiced) {
                        _contoursStartIndices[i] = (size_t) round(contoursStartTimes[i] / _frameDuration);
                        _contoursEndIndices[i] = _contoursStartIndices[i] + contoursBins[i].size() - 1;
                        _contoursSaliencesTotal[i] = accumulate(contoursSaliences[i].begin(), contoursSaliences[i].end(), 0.0);
                        _contoursIgnored.push_back(i);
                    }
                }
            }
        }
        _contoursSelectedInitially = _contoursSelected;
        _contoursIgnoredInitially = _contoursIgnored;
 
    
    }
    
void PredominantMelody::computeMelodyPitchMean(const vector<vector<Real> >&contoursBins) {
        
        /*
         Additional suggestion by Justin Salamon: implement a soft bias against the lowest frequencies:
         if f < 150Hz --> bias = f / 150Hz * 0.3
         In our evaluation, the results are ok without such a bias, when only using a hard threshold of
         80Hz for the minimum frequency allowed for salience peaks. Therefore the bias is not implemented.
         */
        
        vector<Real> melodyPitchMeanSmoothed;
        Real sumSalience;
        
        // compute melody pitch mean (weighted mean for all present contours) for each frame
        Real previous = 0.0;
        for (size_t i=0; i<_numberFrames; i++) {
            _melodyPitchMean[i]=0.0;
            sumSalience = 0.0;
   
            for (size_t j=0; j<_contoursSelected.size(); j++) {
                size_t jj = _contoursSelected[j];
                if (_contoursStartIndices[jj] <= i && _contoursEndIndices[jj] >= i) {
                    // current frame belongs to this contour
                    size_t shift = i - _contoursStartIndices[jj];
                    _melodyPitchMean[i] += _contoursSaliencesTotal[jj] * contoursBins[jj][shift];
                    sumSalience += _contoursSaliencesTotal[jj];
                }
            }
            if (sumSalience > 0) {
                _melodyPitchMean[i] /= sumSalience;
            } else {
                // no contour was found for current frame --> use value from previous bin
                _melodyPitchMean[i] = previous;
            }
            previous = _melodyPitchMean[i];
        }
        
        // replace zeros from the beginnig by the first non-zero value
        for (size_t i=0; i<_numberFrames; i++) {
            if (_melodyPitchMean[i] > 0) {
                for (size_t j=0; j<i; j++) {
                    _melodyPitchMean[j] = _melodyPitchMean[i];
                }
                break;
            }
        }
        
        // run 5-second moving average filter to smooth melody pitch mean
        // we want to align filter output for symmetrical averaging,
        // and we want the filter to return values on the edges as the averager output computed at these positions
        // to avoid smoothing to zero
        AlgorithmFactory& factory2 = standard::AlgorithmFactory::instance();
    Algorithm* _movingAverage = factory2.create("MovingAverage");
        _movingAverage->input("signal").set(_melodyPitchMean);
        _movingAverage->output("signal").set(melodyPitchMeanSmoothed);
        _movingAverage->reset();
        
        _melodyPitchMean.resize(_numberFrames + _averagerShift, _melodyPitchMean.back());
        _melodyPitchMean.insert(_melodyPitchMean.begin(), _averagerShift, _melodyPitchMean.front());
        _movingAverage->compute();
        _melodyPitchMean = vector<Real>(melodyPitchMeanSmoothed.begin() + 2*_averagerShift, melodyPitchMeanSmoothed.end());
    }
    
void PredominantMelody::detectContourDuplicates(const vector<vector<Real> >& contoursBins) {
        /*
         To compare contour trajectories we compute the distance between their pitch values on a per-frame basis for the
         region in which they overlap, and compute the mean over this region. If the mean distance is within 1200+-50 cents,
         the contours are considered octave duplicates.
         
         There is no requirement on the length of overlap region, according to [1] and personal communication with the
         author, but it can be introduced. However, algorithm already works well without such a requirement.
         */
        
        _duplicates.clear();  // re-initialize
        
        for(size_t i=0; i<_contoursSelected.size(); i++) {
            size_t ii = _contoursSelected[i];
            
            for (size_t j=i+1; j<_contoursSelected.size(); j++) {
                size_t jj = _contoursSelected[j];
                size_t start, end;
                bool overlap = false;
                
                if (_contoursStartIndices[ii] >= _contoursStartIndices[jj]
                    && _contoursStartIndices[ii] <= _contoursEndIndices[jj]) {
                    // .......[CONTOUR1]......
                    // ....[CONTOUR2].........
                    // or
                    // .......[CONTOUR1]......
                    // ....[CONTOUR2.....]....
                    start = _contoursStartIndices[ii];
                    end = min(_contoursEndIndices[ii], _contoursEndIndices[jj]);
                    overlap = true;
                }
                else if (_contoursStartIndices[jj] <= _contoursEndIndices[ii]
                         && _contoursStartIndices[jj] >= _contoursStartIndices[ii]) {
                    // ....[CONTOUR1].........
                    // .......[CONTOUR2]......
                    // or
                    // ....[CONTOUR1.....]....
                    // .......[CONTOUR2]......
                    start = _contoursStartIndices[jj];
                    end = min(_contoursEndIndices[ii], _contoursEndIndices[jj]);
                    overlap = true;
                }
                if (overlap) {
                    // compute the mean distance for overlap region
                    Real distance = 0;
                    size_t shift_i = start - _contoursStartIndices[ii];
                    size_t shift_j = start - _contoursStartIndices[jj];
                    
                    for (size_t ioverlap=start; ioverlap<=end; ioverlap++) {
                        distance += contoursBins[ii][shift_i] - contoursBins[jj][shift_j];
                        shift_i++;
                        shift_j++;
                    }
                    distance = abs(distance) / (end-start+ 1);
                    // recode cents to bins
                    if (distance > _duplicateMinDistance && distance < _duplicateMaxDistance) {
                        // contours ii and jj differ for around 1200 cents (i.e., 1 octave) --> they are duplicates
                        _duplicates.push_back(make_pair(ii,jj));
                    }
                }
            }
        }
    }
    
    
void PredominantMelody::removeContourDuplicates() {
        
        // each iteration we start with all contours that passed the voiding detection stage,
        // but use the most recently computed melody pitch mean.
        
        // reinitialize the list of selected contours
        _contoursSelected = _contoursSelectedInitially;
        _contoursIgnored = _contoursIgnoredInitially;
        
        // compute average melody pitch mean on the intervals corresponding to all contours
        vector<Real> contoursMelodyPitchMean;
        contoursMelodyPitchMean.resize(_numberContours);
        for (size_t i=0; i<_contoursSelected.size(); i++) {
            size_t ii = _contoursSelected[i];
            contoursMelodyPitchMean[ii] = accumulate(_melodyPitchMean.begin() + _contoursStartIndices[ii], _melodyPitchMean.begin() + _contoursEndIndices[ii] + 1, 0);
            contoursMelodyPitchMean[ii] /= (_contoursEndIndices[ii] - _contoursStartIndices[ii] + 1);
        }
        
        // for each duplicates pair, remove the contour furtherst from melody pitch mean
        for (size_t c=0; c<_duplicates.size(); c++) {
            size_t ii = _duplicates[c].first;
            size_t jj = _duplicates[c].second;
            Real ii_distance = abs(_contoursBinsMean[ii] - contoursMelodyPitchMean[ii]);
            Real jj_distance = abs(_contoursBinsMean[jj] - contoursMelodyPitchMean[jj]);
            if (ii_distance < jj_distance) {
                // remove contour jj
                _contoursSelected.erase(std::remove(_contoursSelected.begin(), _contoursSelected.end(), jj), _contoursSelected.end());
                if (guessUnvoiced) {
                    _contoursIgnored.push_back(jj);
                }
            } else {
                // remove contour ii
                _contoursSelected.erase(std::remove(_contoursSelected.begin(), _contoursSelected.end(), ii), _contoursSelected.end());
                if (guessUnvoiced) {
                    _contoursIgnored.push_back(ii);
                }
            }
        }
    }
    
void PredominantMelody::removePitchOutliers() {
        
        // compute average melody pitch mean on the intervals corresponding to all contour
        // remove pitch outliers by deleting contours at a distance more that one octave from melody pitch mean
        Real _outlierMaxDistance = (1200.0+50)/binResolution; // a bit more than 1 octave
        
        for (std::vector<size_t>::iterator iter = _contoursSelected.begin(); iter != _contoursSelected.end();) {
            size_t ii = *iter;
            Real contourMelodyPitchMean = accumulate(_melodyPitchMean.begin() + _contoursStartIndices[ii], _melodyPitchMean.begin() + _contoursEndIndices[ii] + 1, 0.0);
            //cout << "contourMelodyPitchMean: "<< contourMelodyPitchMean << endl;
            contourMelodyPitchMean /= (_contoursEndIndices[ii] - _contoursStartIndices[ii] + 1);
            //cout << "contourMelodyPitchMean: "<< contourMelodyPitchMean << endl;
            //cout << "contoursBinMean: "<< _contoursBinsMean[ii] << endl;
            if (abs(_contoursBinsMean[ii] - contourMelodyPitchMean) > _outlierMaxDistance) {
                // remove contour
                iter = _contoursSelected.erase(iter);
                if (guessUnvoiced) {
                    _contoursIgnored.push_back(ii);
                }
            }
            else {
                ++iter;
            }
        }
    }
    
 /*
PredominantMelody::~PredominantMelody() {
    // Pre-processing
    delete _frameCutter;
    delete _windowing;

    // Spectral peaks
    delete _spectralPeaks;

    // Pitch salience contours
    delete _pitchSalienceFunction;
    delete _pitchSalienceFunctionPeaks;
    delete _pitchContours;

    // Melody
    //delete _pitchContoursMelody;
}

*/

}
}
