/*
  ==============================================================================

    SchneiderDelay.cpp
    Created: 6 Nov 2023 8:50:26am
    Author:  mitchschneider_
    
    Basic MultiTap Delay built on top of Will Pirkle's CircularBuffer class
  ==============================================================================
*/
#pragma once
#include <JuceHeader.h>
#include <fxobjects_Schneider.h>

struct MultiTapDelayParameters
{
    MultiTapDelayParameters() {}
    MultiTapDelayParameters& operator=(const MultiTapDelayParameters& params)
    {
        if (this == &params)
            return *this;

        wetLevel_dB = params.wetLevel_dB;
        dryLevel_dB = params.dryLevel_dB;
        feedback_Pct = params.feedback_Pct;

        tap1_mSec = params.tap1_mSec;
        tap2_mSec = params.tap2_mSec;
        tap3_mSec = params.tap3_mSec;
        tap4_mSec = params.tap4_mSec;

        return *this;
    }

    double wetLevel_dB = -3.0;    ///< wet output level in dB
    double dryLevel_dB = -3.0;    ///< dry output level in dB
    double feedback_Pct = 0.0;    ///< feedback as a % value

    double tap1_mSec = 0.0;
    double tap2_mSec = 0.0;
    double tap3_mSec = 0.0;
    double tap4_mSec = 0.0;
};

class MultiTapDelay : public AudioDelay
{
public:
    MultiTapDelay() {}
    ~MultiTapDelay() {}
    
    virtual bool processAudioFrame(const float* inputFrame, float* outputFrame, uint32_t inputChannels, uint32_t outputChannels) override
    {
        // --- make sure we have input and outputs
        if (inputChannels == 0 || outputChannels == 0)
            return false;

        // --- if only one output channel, revert to mono operation
        if (outputChannels == 1)
        {
            // --- process left channel only
            outputFrame[0] = (float)processAudioSample(inputFrame[0]);
            return true;
        }

        // --- if we get here we know we have 2 output channels
        //
        // --- pick up inputs
        //
        // --- LEFT channel
        double xnL = inputFrame[0];

        // --- RIGHT channel (duplicate left input if mono-in)
        double xnR = inputChannels > 1 ? inputFrame[1] : xnL;

        double ynL_1 = delayBuffer_L.readBuffer(delayInSamples_1);
        double ynR_1 = delayBuffer_R.readBuffer(delayInSamples_1);
        double ynL_2 = delayBuffer_L.readBuffer(delayInSamples_2);
        double ynR_2 = delayBuffer_R.readBuffer(delayInSamples_2);
        double ynL_3 = delayBuffer_L.readBuffer(delayInSamples_3);
        double ynR_3 = delayBuffer_R.readBuffer(delayInSamples_3);
        double ynL_4 = delayBuffer_L.readBuffer(delayInSamples_4);
        double ynR_4 = delayBuffer_R.readBuffer(delayInSamples_4);
        
        double combinedTaps_L = ynL_1 + ynL_2 + ynL_3 + ynL_4;
        double combinedTaps_R = ynR_1 + ynR_2 + ynR_3 + ynR_4;
        // --- create input for delay buffer with LEFT channel info
        double dnL = xnL + (parameters.feedback_Pct / 100.0) * ynL_1;

        // --- create input for delay buffer with RIGHT channel info
        double dnR = xnR + (parameters.feedback_Pct / 100.0) * ynR_1;

        // --- write to LEFT delay buffer with LEFT channel info
        delayBuffer_L.writeBuffer(dnL);

        // --- write to RIGHT delay buffer with RIGHT channel info
        delayBuffer_R.writeBuffer(dnR);

        // --- form mixture out = dry*xn + wet*yn
        double outputL = dryMix*xnL + wetMix*combinedTaps_L;

        // --- form mixture out = dry*xn + wet*yn
        double outputR = dryMix*xnR + wetMix*combinedTaps_R;

        // --- set left channel
        outputFrame[0] = (float)outputL;

        // --- set right channel
        outputFrame[1] = (float)outputR;

        return true;
    }

    void setMultiTapParameters(MultiTapDelayParameters _parameters)
    {
        // --- check mix in dB for calc
        if (_parameters.dryLevel_dB != parameters.dryLevel_dB)
            dryMix = pow(10.0, _parameters.dryLevel_dB / 20.0);
        if (_parameters.wetLevel_dB != parameters.wetLevel_dB)
            wetMix = pow(10.0, _parameters.wetLevel_dB / 20.0);

        // --- save; rest of updates are cheap on CPU
        parameters = _parameters;
        
        // --- calculate total delay time in samples + fraction
        delayInSamples_1 = parameters.tap1_mSec*(samplesPerMSec);
        delayInSamples_2 = parameters.tap2_mSec*(samplesPerMSec);
        delayInSamples_3 = parameters.tap3_mSec*(samplesPerMSec);
        delayInSamples_4 = parameters.tap4_mSec*(samplesPerMSec);
    }

private:
    MultiTapDelayParameters parameters; ///< object parameters
    double delayInSamples_1 = 0.0;
    double delayInSamples_2 = 0.0;
    double delayInSamples_3 = 0.0;
    double delayInSamples_4 = 0.0;
};


class AnalogDelay : public AudioDelay
{
public:
    
    AnalogDelay() {};
    ~AnalogDelay() {};
    
    virtual bool reset(double _sampleRate) override
    {
        if (sampleRate == _sampleRate)
        {
            delayBuffer_L.flushBuffer();
            delayBuffer_R.flushBuffer();
            return true;
        }
        createDelayBuffers(_sampleRate, bufferLength_mSec);
        
        lpf.reset(_sampleRate);
        filterParams.algorithm = filterAlgorithm::kLPF1;
        filterParams.Q = 0.707;
        filterParams.boostCut_dB = -6;
        filterParams.fc = 1000;
        lpf.setParameters(filterParams);
        
        return true;
    }
    
    virtual double processAudioSample(double xn) override
    {
        // --- read delay
        double yn = delayBuffer_L.readBuffer(delayInSamples_L);

        // --- create input for delay buffer
        double dn = lpf.processAudioSample(xn + (parameters.feedback_Pct / 100.0) * yn);

        // --- write to delay buffer
        delayBuffer_L.writeBuffer(dn);

        // --- form mixture out = dry*xn + wet*yn
        double output = dryMix*xn + wetMix*yn;

        return output;
    }
    
    virtual bool processAudioFrame(const float* inputFrame, float* outputFrame, uint32_t inputChannels, uint32_t outputChannels) override
    {
        if (inputChannels == 0 || outputChannels == 0)
            return false;

        if (parameters.algorithm != delayAlgorithm::kNormal &&
            parameters.algorithm != delayAlgorithm::kPingPong)
            return false;

        if (outputChannels == 1)
        {
            outputFrame[0] = (float)processAudioSample(inputFrame[0]);
            return true;
        }

        double xnL = inputFrame[0];

        double xnR = inputChannels > 1 ? inputFrame[1] : xnL;

        double ynL = delayBuffer_L.readBuffer(delayInSamples_L);

        double ynR = delayBuffer_R.readBuffer(delayInSamples_R);

        // --- create input for delay buffer with LEFT channel info
        double dnL = lpf.processAudioSample( xnL + (parameters.feedback_Pct / 100.0) * ynL);

        // --- create input for delay buffer with RIGHT channel info
        double dnR = lpf.processAudioSample(xnR + (parameters.feedback_Pct / 100.0) * ynR);
        
        // --- decode
        if (parameters.algorithm == delayAlgorithm::kNormal)
        {
            // --- write to LEFT delay buffer with LEFT channel info
            delayBuffer_L.writeBuffer(dnL);

            // --- write to RIGHT delay buffer with RIGHT channel info
            delayBuffer_R.writeBuffer(dnR);
        }
        else if (parameters.algorithm == delayAlgorithm::kPingPong)
        {
            // --- write to LEFT delay buffer with RIGHT channel info
            delayBuffer_L.writeBuffer(dnR);

            // --- write to RIGHT delay buffer with LEFT channel info
            delayBuffer_R.writeBuffer(dnL);
        }

        // --- form mixture out = dry*xn + wet*yn
        double outputL = dryMix*xnL + wetMix*ynL;

        // --- form mixture out = dry*xn + wet*yn
        double outputR = dryMix*xnR + wetMix*ynR;

        // --- set left channel
        outputFrame[0] = (float)outputL;

        // --- set right channel
        outputFrame[1] = (float)outputR;

        return true;
    }
    
protected:
    AudioFilter lpf;
    AudioFilterParameters filterParams;
};


class LCRDelay : public AudioDelay
{
    
};


class DuckingDelay : public AudioDelay
{
    
};
