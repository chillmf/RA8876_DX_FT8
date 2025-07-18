/*-------------------------------------------------------------------------------
   AudioSDRpreProcessor.cpp

   Function: A input pre-proccessor to "condition"  quadrature (IQ) input signals before passing
              to the AudioSDR software-defined-radio Teensy 3.6 Audio block.

   Author:   Derek Rowell (drowell@mit.edu)
   Date:     April 26, 2019

   Notes:    Includes the following functions:
             a) Automatically detect and correct the random Teensy single-sample delay
                bug in the I2S input stream,
             b) Manually turn on and off I2S error correction (overides the automatic correction)
             c) Return the current I2S error correction state.
             d) Swap the I and Q input channels.   The convention used in the AudioSDR system
               is that the I channel should be connected to the I2S input 0 (left) and that
               the Q channel should be connect to input 1 (right).   If your hardware does not
               use this convention, you can use this software fix.
  ---
  Copyright (c) 2019 Derek Rowell
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
------------------------------------------------------------------------------- */

#include "AudioSDRpreProcessor.h"
#include "utility/dspinst.h"

#include "arm_math.h"

#include "arm_const_structs.h"

#include "Streaming.h"
// -----
void AudioSDRpreProcessor::update(void)
{
  audio_block_t *blockI = receiveWritable(0); // real (quadrature I) data
  audio_block_t *blockQ = receiveWritable(1); // imaginary (quadrature Q) data

  if (blockI == NULL || blockQ == NULL)
  {
    if (blockI != NULL)
      release(blockI);
    if (blockQ != NULL)
      release(blockQ);
    return;
  }

  //
  //---------------------------------------------------------------------------------------------
  // Teensy I2S single-sample delay IQ lag compensation:
  //   Note: The Teensy I2S bug causes a randomly occuring single-sample delay in I2S input
  //     channel 1 (blockQ in this case) on power-up or program reload.   To correct this,
  //     simply delay the samples in I2S input channel 0 (blockI) by a single sample so that
  //     the channels are synchronized again.
  // ---
  if (I2Scorrection == 1)
  {
    int16_t temp = blockI->data[n_block - 1]; // save the most recent sample for the next buffer
    for (int i = n_block - 1; i > 0; i--)
      blockI->data[i] = blockI->data[i - 1];
    blockI->data[0] = savedSample;
    savedSample = temp;
  }
  else if (I2Scorrection == -1)
  {
    int16_t temp = blockQ->data[n_block - 1]; // save the most recent sample for the next buffer
    for (int i = n_block - 1; i > 0; i--)
      blockQ->data[i] = blockQ->data[i - 1];
    blockI->data[0] = savedSample;
    savedSample = temp;
  }

  //
  //---------------------------------------------------------------------------------------------
  //   I2S single-sample delay detection - look for spectral images in the data FFT.
  //   The method recognizes that errors in the phase (and amplitude) of the I and Q channels
  //   will generate symmetrical image lines in the complex spectrum, reflecting similar amplitudes in lines
  //   j and (n__FTT j).   If there is no I2S error, the magnitude ratio between these lines will be large.
  //   The decision on the existence of a delay error is based on the ratio between the powers of the
  //   strongest spectral line and its image.
  // ---
  if (autoDetectFlag)
  {
    const int16_t n_FFT = 128;
    const int16_t min = 5;
    int maxLine = 0;
    //                                // At this point the output data block has already been updated
    for (int i = 0; i < 128; i++)
    {
      // Take 128 point FFT and compute the magnitude squared
      buffer[2 * i] = float(blockI->data[i]) / 32767.0;
      buffer[2 * i + 1] = float(blockQ->data[i]) / 32767.0;
    }
    // Take 128 point FFT and compute the magnitude squared
    arm_cfft_f32(&arm_cfft_sR_f32_len128, buffer, 0, 1);
    arm_cmplx_mag_squared_f32(buffer, buffer, 128); // "power" spectrum in elements 0 to 127

    // Find the strongest spectral line and compute the average line power across the whole spectrum.
    float average_power = 0.0;
    float maximum_power = 0.0;
    for (int i = min; i < (n_FFT - min); i++)
    {
      // Ignore spectral lines around dc (noise)
      average_power += buffer[i];
      if (buffer[i] > maximum_power)
      {
        maxLine = i;
        maximum_power = buffer[i];
      }
    }

    average_power /= (n_FFT - 2 * min); // average power over all spectral lines
    // Find the ratio of the amplitude of the maximum power line to its spectral image
    float imbalance_ratio = maximum_power / buffer[n_FFT - maxLine];
    //  Make sure the maximum power line is well above the spectral "floor"
    if (maximum_power > spectralAvgMultiplier * average_power)
    {
      // Limit to "strong" spectral lines
      if (imbalance_ratio < minImbalanceRatio)
        failureCount++; // Ratio too low, increment failure counter
      else
        failureCount = 0; // Success - start the count over

      if (failureCount > maxFailureCount)
      {
        // Too many failures (low ratios)in a row...
        I2Scorrection++;
        if (I2Scorrection > 1)
          I2Scorrection = -1; // Try a new correction factor (-1, 0, or 1)...
        failureCount = 0;     // and start over...
        successCount = 0;
      }
      successCount++;
    }
    if (successCount > maxSuccessCount)
    {
      autoDetectFlag = false; // Turn autoCorrection off and accept the current correction
    }
  }

  // ----------------------------------------------------------------------
  // Swap I and Q channels to I in channel and Q in channel 0 to correct for
  // incorrect quadrature input connections
  if (IQswap)
  {
    for (int i = 0; i < 128; i++)
    {
      int temp = blockI->data[i];
      blockI->data[i] = blockQ->data[i];
      blockQ->data[i] = temp;
    }
  }

  transmit(blockI, 0);
  transmit(blockQ, 1);
  release(blockQ);
  release(blockI);
}
// -------------------------- Public Functions ----------------------
// ---
// --- Enable auto detection and correction of the I2S input error
void AudioSDRpreProcessor::startAutoI2SerrorDetection(void)
{
  autoDetectFlag = true;
  I2Scorrection = 0;
  failureCount = 0;
  successCount = 0;
  // autoDetectFlag = false;
}

// ---
// --- Disable auto detection and correction of the I2S input error
void AudioSDRpreProcessor::stopAutoI2SerrorDetection(void)
{
  autoDetectFlag = false;
  I2Scorrection = 0; // Revert to no compensati9n
}

// --- Return the state of the auto detection
//     true = auto detection is active, false = auto detection is inactve
bool AudioSDRpreProcessor::getAutoI2SerrorDetectionStatus(void) { return autoDetectFlag; }
//
// --- Manually set I2S error correction mode
void AudioSDRpreProcessor::setI2SerrorCompensation(int correction)
{
  I2Scorrection = correction;
  autoDetectFlag = false; // Cancel auto correction if active
}

// ---
// ---  Fetch the current state of the I2S error correction (on or off)
int16_t AudioSDRpreProcessor::getI2SerrorCompensation(void) { return I2Scorrection; }
// ---
// --- Swap quadrature inputs from I on channel 0 to I on channel 1
void AudioSDRpreProcessor::swapIQ(boolean swap) { IQswap = swap; }
