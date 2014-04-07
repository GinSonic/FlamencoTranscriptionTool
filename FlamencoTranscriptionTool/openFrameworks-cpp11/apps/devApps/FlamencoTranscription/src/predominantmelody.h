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

#ifndef ESSENTIA_PREDOMINANTMELODY_H
#define ESSENTIA_PREDOMINANTMELODY_H

#include <essentia/algorithmfactory.h>
#include <essentia/essentiamath.h>

namespace essentia {
namespace standard {


class PredominantMelody {

    protected:
        Algorithm* _frameCutter;
        Algorithm* _windowing;
        Algorithm* _spectralPeaks;
        Algorithm* _pitchSalienceFunction;
        Algorithm* _pitchSalienceFunctionPeaks;
        standard::Algorithm* _pitchContours;
        standard::Algorithm* _pitchContoursMelody;
        Algorithm* _movingAverage;


    public:
        std::vector<Real> _signal;
        std::vector<Real> _pitch;
        std::vector<Real> _pitchConfidence;
    
        Real sampleRate;
        int frameSize;
        int hopSize;
        std::string windowType = "hann";
        int zeroPaddingFactor;
        int maxSpectralPeaks;
        Real referenceFrequency;
        Real binResolution;
        Real magnitudeThreshold;
        Real magnitudeCompression;
        int numberHarmonics;
        Real harmonicWeight;
        
        Real minFrequency;
        Real maxFrequency;
        
        Real peakFrameThreshold;
        Real peakDistributionThreshold;
        Real pitchContinuity;
        Real timeContinuity;
        Real minDuration;
        
        Real voicingTolerance;
        int filterIterations;
        bool voiceVibrato;
        bool guessUnvoiced;
    
    Real _frameDuration;
    size_t _numberFrames;
    size_t _averagerShift;
    Real _outlierMaxDistance;
    Real _duplicateMaxDistance;
    Real _duplicateMinDistance;
    
    
    size_t _numberContours;
    Real _vibratoPitchStddev;
    
    Real _minBin;
    Real _maxBin;
    
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
    
        PredominantMelody();
    
        void compute(std::vector<Real> signal, std::vector<Real>& pitch, std::vector<Real>& pitchConfidence );
        void powerSpectrum(std::vector<Real> input, std::vector<Real>& output, int size);
        void pitchContoursMelody(std::vector<std::vector<Real> > _contoursBins,std::vector<std::vector<Real> >_contoursSaliences, std::vector<Real> _contoursStartTimes, Real _duration, std::vector<Real>& _pitch, std::vector<Real>& _pitchConfidence);
    
    bool detectVoiceVibrato(std::vector<Real> contourBins, const Real binMean);
    void voicingDetection(const std::vector<std::vector<Real> >& contoursBins,
                          const std::vector<std::vector<Real> >& contoursSaliences,
                          const std::vector<Real>& contoursStartTimes);
    
    void computeMelodyPitchMean(const std::vector<std::vector<Real> >& contoursBins);
    void detectContourDuplicates(const std::vector<std::vector<Real> >& contoursBins);
    void removeContourDuplicates();
    void removePitchOutliers();
    ~PredominantMelody();

    
};

} // namespace streaming
} // namespace essentia

#endif // ESSENTIA_PREDOMINANTMELODY_H
