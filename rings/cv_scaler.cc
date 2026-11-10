// Copyright 2015 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
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
// See http://creativecommons.org/licenses/MIT/ for more information.
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
  // LAW, REMOVE_OFFSET, SCALE, VIRTUAL_DISCONNECTED, LP_COEFFICIENT
  { LAW_LINEAR, true, 0.0f, true, 0.0f },       // ADC_CHANNEL_CV_FREQUENCY -> VIRTUÁLNĚ ODPOJENO!
  { LAW_LINEAR, true, 1.0f, false, 0.10f },      // ADC_CHANNEL_CV_STRUCTURE
  { LAW_LINEAR, true, 1.0f, false, 0.10f },      // ADC_CHANNEL_CV_BRIGHTNESS
  { LAW_LINEAR, true, 1.0f, false, 0.05f },      // ADC_CHANNEL_CV_DAMPING
  { LAW_LINEAR, true, 1.0f, false, 0.01f },      // ADC_CHANNEL_CV_POSITION
  { LAW_LINEAR, false, 1.0f, false, 1.00f },     // ADC_CHANNEL_CV_V_OCT --tady by jsme mohli nastavit riditelný Tone-slide přes central ladení na ms !)
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//al to až na konec !
  { LAW_LINEAR, false, 1.0f, false, 0.10f },     // ADC_CHANNEL_POT_FREQUENCY
  { LAW_LINEAR, false, 1.0f, false, 0.10f },     // ADC_CHANNEL_POT_STRUCTURE
  // Poťáky z multiplexeru
  { LAW_LINEAR, false, 1.0f, false, 0.1f },      // ADC_CHANNEL_POT_BRIGHTNESS
  { LAW_LINEAR, false, 1.0f, false, 0.1f },      // ADC_CHANNEL_POT_DAMPING
  { LAW_LINEAR, false, 1.0f, false, 0.1f },      // ADC_CHANNEL_POT_POSITION
  // 5x Audio analýza přepnutá do vašeho módu RAW_ANALIZE
  { RAW_ANALIZE, false, 1.0f, false, 1.0f },     // ADC_CHANNEL_ANALYZE_AUDIO_1
  { RAW_ANALIZE, false, 1.0f, false, 1.0f },     // ADC_CHANNEL_ANALYZE_AUDIO_2
  { RAW_ANALIZE, false, 1.0f, false, 1.0f },     // ADC_CHANNEL_ANALYZE_AUDIO_3
  { RAW_ANALIZE, false, 1.0f, false, 1.0f },     // ADC_CHANNEL_ANALYZE_AUDIO_4
  { RAW_ANALIZE, false, 1.0f, false, 1.0f },     // ADC_CHANNEL_ANALYZE_AUDIO_5 (vyhlazovací koeficient 0.05f)     // ADC_CHANNEL_ANALYZE_AUDIO_5
};
//nemužou to delat ty filtry ???

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
  chord_ = 0; //nedelá to tohle ??? 
   
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
/*
#define ATTENUVERT(destination, NAME, min, max) \
  { \
    float value = adc_lp_[ADC_CHANNEL_CV_ ## NAME]; \
    value *= adc_lp_[ADC_CHANNEL_ATTENUVERTER_ ## NAME]; \
    value += adc_lp_[ADC_CHANNEL_POT_ ## NAME]; \
    CONSTRAIN(value, min, max) \
    destination = value; \
  }
*/   //nespřemotali ty kanáli ty attenuvertery odebrané poté přidané jako anliza 4 4 4 4 4 

void CvScaler::Read(Patch* patch, PerformanceState* performance_state) {
  
  // UNIVERZÁLNÍ SMYČKA PRO VŠECHNY KANÁLY
  for (size_t i = 0; i < ADC_CHANNEL_LAST; ++i) {
    const ChannelSettings& settings = channel_settings_[i];
    
    // A: VIRTUÁLNÍ ODPOJENÍ JACKU (Tady se uplatní true u kanálu 0, hardware se vůbec nečte)
    if (settings.virtual_disconnected) {
      adc_lp_[i] = 0.0f; 
      continue; 
    }
    //jestli ono to nahodou neodpojuje :cv_v_oct ?????

    // B: REŽIM PRO AUDIO ANALÝZU (bez float přepočtů) 
    //tenhle asi není připojen do pollservo v ui.cc  a mel by asi být , ale navíc bez smoothed filtru !!!
    if (settings.law == RAW_ANALIZE) {
        continue; 
    }
      
       // C: STANDARDNÍ ČTENÍ PRO VŠECHNY OSTATNÍ FUNKČNÍ JACKY A POTENCIOMETRY (16-bit!)
    float value = adc_.float_value(i);

    if (settings.remove_offset) {
      value = calibration_data_->offset[i] - value;
    }  //nepusobí tohl negativne na ty chc 1 a 7 ??? 

    value *= settings.scale; //taky nevin s timhle ??? 
    adc_lp_[i] += settings.lp_coefficient * (value - adc_lp_[i]);
  }


  // ROZPLETENÍ ZBYTKU JACKŮ V CV_SCALERU: //ne přiomnost tohoto nahovno pro ten posun ??
  patch->structure  = adc_lp_[ADC_CHANNEL_CV_STRUCTURE]  + adc_lp_[ADC_CHANNEL_POT_STRUCTURE];
  patch->brightness = adc_lp_[ADC_CHANNEL_CV_BRIGHTNESS] + adc_lp_[ADC_CHANNEL_POT_BRIGHTNESS];
  patch->damping    = adc_lp_[ADC_CHANNEL_CV_DAMPING] + adc_lp_[ADC_CHANNEL_POT_DAMPING];
  patch->position = adc_lp_[ADC_CHANNEL_CV_POSITION] + adc_lp_[ADC_CHANNEL_POT_POSITION];
  
  CONSTRAIN(patch->structure, 0.0f, 0.9995f); //jestlitu nemá být jen 0,5 ??? jako prostřed consten puvodně antenverter ,ale nyní pro analzu jen !!! 
  CONSTRAIN(patch->brightness, 0.0f, 1.0f); // taky jen pro analizu 
  CONSTRAIN(patch->damping, 0.0f, 1.0f); // -//--
  CONSTRAIN(patch->position, 0.0f, 1.0f); // ----//--

  // Váš kód pro vynulování FM a odeslání do performance_state
  float fm = 0.0f;
  fm_cv_ = fm;
  performance_state->fm = fm_cv_;
  
  float transpose = 60.0f * adc_lp_[ADC_CHANNEL_POT_FREQUENCY];
  float hysteresis = transpose - transpose_ > 0.0f ? -0.3f : +0.3f;
  transpose_ = static_cast<int32_t>(transpose + hysteresis + 0.5f);


  float note = calibration_data_->pitch_offset;
  note += adc_lp_[ADC_CHANNEL_CV_V_OCT] * calibration_data_->pitch_scale;
  
  // 2. V/OCT PŘÍMO ZE SVÉHO KANÁLU (index 5 = PA1):
  performance_state->note = note;
  performance_state->tonic = 12.0f + transpose_;
  
  DetectNormalization();

    // TOTO NATVRDO VYPNUTO (aby ti to nemazalo V/OCT na nulu):
 // performance_state->internal_note = false;
  

 // Strumming logika
  // Strumming / internal exciter triggering logic.
  bool internal_strum = normalization_detector_trigger_.normalized();
  bool internal_exciter = normalization_detector_exciter_.normalized();
  bool internal_note = normalization_detector_v_oct_.normalized();
  performance_state->internal_exciter = internal_exciter;
  performance_state->internal_strum = internal_strum;
  performance_state->internal_note = internal_note;
  performance_state->strum = trigger_input_.rising_edge();
  

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

}