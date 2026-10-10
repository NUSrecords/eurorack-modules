// Copyright  Emilie Gillet.
//
// Author: Emilie Gillet 
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
//
// -----------------------------------------------------------------------------
//
// Filtering and scaling of ADC values + input calibration.

#include "rings/cv_scaler.h"

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/system/storage.h"
#include "stmlib/utils/random.h"

#include "rings/dsp/part.h"
#include "rings/dsp/patch.h"

namespace rings {
  
using namespace std;
using namespace stmlib;

/* static */
ChannelSettings CvScaler::channel_settings_[ADC_CHANNEL_LAST] = {
  { LAW_LINEAR, true, 1.00f },  // ADC_CHANNEL_CV_FREQUENCY
  { LAW_LINEAR, true, 0.1f },  // ADC_CHANNEL_CV_STRUCTURE
  { LAW_LINEAR, true, 0.1f },  // ADC_CHANNEL_CV_BRIGHTNESS
  { LAW_LINEAR, true, 0.05f },  // ADC_CHANNEL_CV_DAMPING
  { LAW_LINEAR, true, 0.01f },  // ADC_CHANNEL_CV_POSITION
  { LAW_LINEAR, false, 1.00f },  // ADC_CHANNEL_CV_V_OCT
  { LAW_LINEAR, false, 0.01f },  // ADC_CHANNEL_POT_FREQUENCY
  { LAW_LINEAR, false, 0.01f },  // ADC_CHANNEL_POT_STRUCTURE
  { LAW_LINEAR, false, 0.01f },  // ADC_CHANNEL_POT_BRIGHTNESS
  { LAW_LINEAR, false, 0.01f },  // ADC_CHANNEL_POT_DAMPING
  { LAW_LINEAR, false, 0.01f },  // ADC_CHANNEL_POT_POSITION
  { LAW_ADC_ANALIZE, false, 0.005f },  // 11: ADC_CHANNEL_ATTENUVERTER_FREQUENCY
  { LAW_DAC, false, 0.005f }, // 12: ADC_CHANNEL_ATTENUVERTER_DAMPING_A
  { LAW_DAC, false, 0.005f }, // 13: ADC_CHANNEL_ATTENUVERTER_DAMPING_B
  { LAW_DAC, false, 0.005f }, // 14: ADC_CHANNEL_ATTENUVERTER_DAMPING (Tvůj hlavní)
  { LAW_DAC, false, 0.005f }, // 15: ADC_CHANNEL_ATTENUVERTER_DAMPING_C
};

void CvScaler::Init(CalibrationData* calibration_data) {
  calibration_data_ = calibration_data;

  adc_.Init();
  trigger_input_.Init();

  transpose_ = 0.0f;
  
  fill(&adc_lp_[0], &adc_lp_[ADC_CHANNEL_LAST], 0.0f);
  
  normalization_probe_.Init();
  normalization_detector_exciter_.Init(0.01f, 0.5f);
  normalization_detector_trigger_.Init(0.05f, 0.9f);
  normalization_detector_v_oct_.Init(0.01f, 0.5f);
  
  inhibit_strum_ = 0;
  fm_cv_ = 0.0f;
  
  normalization_probe_enabled_ = true;
  normalization_probe_forced_state_ = false;
}

void CvScaler::DetectAudioNormalization(Codec::Frame* in, size_t size) {
  int32_t count = 0;
  short* input_samples = &in->r;
  for (size_t i = 0; i < size; i += 8) {
    short s = input_samples[i * 2];
    if (s > 50 && s < 1500) {
      ++count;
    } else if (s > -1500 && s < -50) {
      --count;
    }
  }
  float y = static_cast<float>(count) / static_cast<float>(size >> 3);
  float x = normalization_probe_value_[1] ? -1.0f : 1.0f;
  
  normalization_detector_exciter_.Process(x, y);
  if (normalization_detector_exciter_.normalized()) {
    for (size_t i = 0; i < size; ++i) {
      input_samples[i * 2] = 0;
    }
  }
}

void CvScaler::DetectNormalization() {
  if (normalization_probe_value_[0] == trigger_input_.DummyRead()) {
    normalization_detector_trigger_.Process(1.0f, 1.0f);
  } else {
    normalization_detector_trigger_.Process(1.0f, -1.0f);
  }
  
  float x = adc_.float_value(ADC_CHANNEL_CV_V_OCT) - calibration_data_->normalization_detection_threshold;
  float y = normalization_probe_value_[0] ? -1.0f : 1.0f;
  if (x > -0.5f && x < 0.5f) {
    x = x < 0.0f ? -1.0f : 1.0f;
    normalization_detector_v_oct_.Process(x, y);
  } else {
    normalization_detector_v_oct_.Process(0.0f, y);
  }
  
  normalization_probe_value_[1] = normalization_probe_value_[0];
  normalization_probe_value_[0] = Random::GetWord() >> 31;
  bool new_state = normalization_probe_enabled_
      ? normalization_probe_value_[0]
      : normalization_probe_forced_state_;
  normalization_probe_.Write(new_state);
}

void CvScaler::Read(Patch* patch, PerformanceState* performance_state) {
  // Process all CVs / pots.
  for (size_t i = 0; i < ADC_CHANNEL_LAST; ++i) {
    // 1. CHRÁNÍME INDEX 11 (Čtení driftu bez bipolárního zmrzačení)
    if (i == 11) {
      adc_lp_[i] = adc_.float_value(i); // Uložíme čistý lineární float (0.0 až 1.0)
      continue; // Okamžitě skáčeme dál, switch níže se pro index 11 vůbec nespuští!
    }
    
    // 2. CHRÁNÍME INDEXY 12 AŽ 15 (Multiplexer adresa 0)
    if (i >= 12 && i <= 15) {  
      adc_lp_[i] = 0.0f; // Vnutíme čistou nulu, aby servo nemodulovalo délku tónu (Damping)
      continue; // Okamžitě skáčeme na další index
    }
    
    const ChannelSettings& settings = channel_settings_[i];
    float value = adc_.float_value(i);
    if (settings.remove_offset) {
      value = calibration_data_->offset[i] - value;
    }
switch (settings.law) {
      case LAW_ADC_ANALIZE:
      case LAW_DAC:
        // Surová hodnota projde skrz 1:1 naprosto čistě, lineárně a bez úprav
        if (value > 1.0f) value = 1.0f;
        if (value < 0.0f) value = 0.0f;
        break;

      default:
        // Pro běžné lineární poťáky (LAW_LINEAR) hodnota projde beze změn
        break;
    }
    adc_lp_[i] += settings.lp_coefficient * (value - adc_lp_[i]);
  }
  
  // 1. STRUCURE A BRIGHTNESS – Čistý součet poťáku a CV jacku
  patch->structure = adc_lp_[ADC_CHANNEL_CV_STRUCTURE] + adc_lp_[ADC_CHANNEL_POT_STRUCTURE];
  CONSTRAIN(patch->structure, 0.0f, 0.9995f);

  patch->brightness = adc_lp_[ADC_CHANNEL_CV_BRIGHTNESS] + adc_lp_[ADC_CHANNEL_POT_BRIGHTNESS];
  CONSTRAIN(patch->brightness, 0.0f, 1.0f);

  // OPRAVENO: Správný zápis do patch->position a vyčištění překlepů s čárkami
  patch->position = adc_lp_[ADC_CHANNEL_CV_POSITION] + adc_lp_[ADC_CHANNEL_POT_POSITION];
  CONSTRAIN(patch->position, 0.0f, 1.0f);

  // OPRAVENO: Nahrazeno nefunkční makro ATTENUVERT čistým lineárním součtem pro Damping
  patch->damping = adc_lp_[ADC_CHANNEL_CV_DAMPING] + adc_lp_[ADC_CHANNEL_POT_DAMPING];
  CONSTRAIN(patch->damping, 0.0f, 1.0f);
  
  float fm = adc_lp_[ADC_CHANNEL_CV_FREQUENCY] * 48.0f;
  float error = fm - fm_cv_;
  if (fabs(error) >= 0.8f) {
    fm_cv_ = fm;
  } else {
    fm_cv_ += 0.02f * error;
  }
  performance_state->fm = fm_cv_;
  CONSTRAIN(performance_state->fm, -48.0f, 48.0f);
  
  float transpose = 60.0f * adc_lp_[ADC_CHANNEL_POT_FREQUENCY];
  float hysteresis = transpose - transpose_ > 0.0f ? -0.3f : +0.3f;
  transpose_ = static_cast<int32_t>(transpose + hysteresis + 0.5f);
  
  float note = calibration_data_->pitch_offset;
  note += adc_lp_[ADC_CHANNEL_CV_V_OCT] * calibration_data_->pitch_scale;
  
  performance_state->note = note;
  performance_state->tonic = 12.0f + transpose_;
  
  DetectNormalization();
  
  // Strumming / internal exciter triggering logic.
  bool internal_strum = normalization_detector_trigger_.normalized();
  bool internal_exciter = normalization_detector_exciter_.normalized();
  bool internal_note = normalization_detector_v_oct_.normalized();
  performance_state->internal_exciter = internal_exciter;
  performance_state->internal_strum = internal_strum;
  performance_state->internal_note = internal_note;
  performance_state->strum = trigger_input_.rising_edge();
  
  if (performance_state->internal_note) {
    performance_state->note = 0.0f;
    performance_state->tonic = 12.0f + transpose;
  }
  
  // Hysteresis on chord.
  float chord = calibration_data_->offset[ADC_CHANNEL_CV_STRUCTURE] - \
      adc_.float_value(ADC_CHANNEL_CV_STRUCTURE);
  chord += adc_lp_[ADC_CHANNEL_POT_STRUCTURE];
  chord *= static_cast<float>(kNumChords - 1);
  hysteresis = chord - chord_ > 0.0f ? -0.1f : +0.1f;
  chord_ = static_cast<int32_t>(chord + hysteresis + 0.5f);
  CONSTRAIN(chord_, 0, kNumChords - 1);
  performance_state->chord = chord_;
  
  adc_.Convert();
  trigger_input_.Read();
}

}  // namespace rings

