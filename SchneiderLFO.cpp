/*
  ==============================================================================

    SchneiderLFO.cpp
    Created: 24 Oct 2023 1:38:02pm
    Author:  mitchschneider_
    
    An improved interface for Will Pirkle's LFO class
    All LFOs are normalized to a value between [0,1] for unipolar and [-1,1] for bipolar

  ==============================================================================
*/

#include "fxobjects_Schneider.cpp"

class SchneiderLFO : LFO
{
public:
    
    SchneiderLFO(){};
    ~SchneiderLFO(){};
    
    void resetLFO(float sampleRate)
    {
        lfo.reset(sampleRate);
    }
    
    void setLfoDepth(float amplitude)
    {
        if (params.amplitude != amplitude)
        {
            params.amplitude = amplitude;
            setLfoParams(params);
        }
    }
    
    void setLfoRateInHz(float frequency)
    {
        if (params.frequency_Hz != frequency)
        {
            params.frequency_Hz = frequency;
            setLfoParams(params);
        }
    }
    
    void setLfoType(generatorWaveform type)
    {
        if (params.waveform != type)
        {
            params.waveform = type;
            setLfoParams(params);
        }
    }
    
    void setLfoParams(OscillatorParameters params)
    {
        lfo.setParameters(params);
    }
    
    float generateUnipolarMaxDownLFO()
    {
        float unipolarModValue = bipolarToUnipolar(lfo.renderAudioOutput().normalOutput);
        
        float attenuatedModValue = lfo.getParameters().amplitude * (maxValue - (1.0 - unipolarModValue)*(maxValue - minValue)) - offset;
        
        return attenuatedModValue;
    }
    
    float generateUnipolarMaxDownQuadPhasePosLFO()
    {
        float unipolarModValue = bipolarToUnipolar(lfo.renderAudioOutput().quadPhaseOutput_pos);
        
        float attenuatedModValue = lfo.getParameters().amplitude * (maxValue - (1.0 - unipolarModValue)*(maxValue - minValue)) - offset;
        
        return attenuatedModValue;
    }
    
    float generateUnipolarMinUpLFO()
    {
        float unipolarModValue = bipolarToUnipolar(lfo.renderAudioOutput().normalOutput);
        
        float attenuatedModValue = lfo.getParameters().amplitude * (unipolarModValue*(maxValue - minValue) + minValue);
        
        return attenuatedModValue;
    }
    
    float generateBipolarLFO()
    {
        return lfo.getParameters().amplitude * lfo.renderAudioOutput().normalOutput;
    }
    
    float generateQuadPhasePosBipolarLFO()
    {
        return lfo.renderAudioOutput().quadPhaseOutput_pos * lfo.getParameters().amplitude;
    }
    
    float generateQuadPhaseNegBipolarLFO()
    {
        return lfo.renderAudioOutput().quadPhaseOutput_neg * lfo.getParameters().amplitude;
    }
    
    float generateInvertedBipolarLFO()
    {
        return lfo.renderAudioOutput().invertedOutput * lfo.getParameters().amplitude;
    }
    
private:
    
    LFO lfo;
    OscillatorParameters params;
    float maxValue = 1.0;
    float minValue = 0.0;
    float offset = 1.0;

};
