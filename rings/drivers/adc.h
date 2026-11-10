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
// Driver for ADC1 - used for scanning CVs, and ADC2 - used for scanning pots.

#ifndef RINGS_DRIVERS_ADC_H_
#define RINGS_DRIVERS_ADC_H_

#include "stmlib/stmlib.h"
#include <stm32f4xx_conf.h>

namespace rings {

enum AdcChannel {
  ADC_CHANNEL_CV_FREQUENCY, //diskonectet virtual ... 
  ADC_CHANNEL_CV_STRUCTURE,
  ADC_CHANNEL_CV_BRIGHTNESS,
  ADC_CHANNEL_CV_DAMPING,
  ADC_CHANNEL_CV_POSITION,
  ADC_CHANNEL_CV_V_OCT,
  ADC_CHANNEL_POT_FREQUENCY, 
  ADC_CHANNEL_POT_STRUCTURE,
  //multiplex : 
  ADC_CHANNEL_POT_BRIGHTNESS,
  ADC_CHANNEL_POT_DAMPING,
  ADC_CHANNEL_POT_POSITION,
  // 5x opakovaný kanál pro analýzu audia 
  ADC_CHANNEL_ANALYZE_AUDIO_1,  
  ADC_CHANNEL_ANALYZE_AUDIO_2,
  ADC_CHANNEL_ANALYZE_AUDIO_3,
  ADC_CHANNEL_ANALYZE_AUDIO_4,
  ADC_CHANNEL_ANALYZE_AUDIO_5,
  ADC_CHANNEL_LAST,

  ADC_CHANNEL_LAST_DIRECT = ADC_CHANNEL_POT_STRUCTURE,
  ADC_CHANNEL_FIRST_MUXED = ADC_CHANNEL_POT_BRIGHTNESS,
  ADC_CHANNEL_NUM_DIRECT = ADC_CHANNEL_FIRST_MUXED,
  ADC_CHANNEL_NUM_MUXED = ADC_CHANNEL_LAST - ADC_CHANNEL_FIRST_MUXED,
  ADC_CHANNEL_NUM_OFFSETS = ADC_CHANNEL_CV_POSITION + 1
};

class Adc {
 public:
  Adc() { }
  ~Adc() { }
  ////////inicializuj funkci read ať to neměří s pa4 ,aledato virtual hodnotu nula ! aby to neovlivnilo nic .
  void Init();
  void DeInit();
  void Convert();

  inline const uint16_t* values() { 
        static uint16_t fake_values[ADC_CHANNEL_LAST];

    for (int i = 0; i < ADC_CHANNEL_LAST; i++) {
      fake_values[i] = values_[i];
    }
    // Vnitřní napětí čipu na indexu 0 zahodíme a podstrčíme audio enginu Virtual Jack nulu!
    fake_values[ADC_CHANNEL_CV_FREQUENCY] = 32768; 
    
    return &fake_values[0]; 
  }

  // Čistá softwarová izolace, která zachrání poťáky i analýzu, ale odřízne pískání
  inline int32_t value(int32_t channel) const {
    if (channel == ADC_CHANNEL_CV_FREQUENCY) {
      return 32768; // Tvůj Virtual Jack pro stabilní DAC (PA4)
    }
  
    // Všechny ostatní indexy (včetně poťáků z multiplexeru 8, 9, 10 a audio analýzy 11 až 15)
    // zůstávají 100% netknuté, takže tvé servo v ui.cc a lupa jedou na plný plyn dál!
    return static_cast<int32_t>(values_[channel]);
  }
  
  // Stejné pojištění pro float hodnoty, aby scaler v cv_scaler.cc utichl
  inline float float_value(int32_t index) const {
    if (index == ADC_CHANNEL_CV_FREQUENCY) {
      return 0.5f; 
    }

    return static_cast<float>(values_[index]) / 65536.0f;
  }

  inline int32_t last_read() const {
    return last_read_ + ADC_CHANNEL_FIRST_MUXED;
  }

  // Vrací plnou, čistou 16-bitovou hodnotu (0 až 65535) naměřenou na konci pátého chodu
  inline uint32_t raw_analyze_audio() const {
  return static_cast<uint32_t>(values_[ADC_CHANNEL_ANALYZE_AUDIO_5]);  
  }

 inline void write_dc_offset(uint16_t value_16bit) {
    uint16_t value_12bit = value_16bit >> 4;
    if (value_12bit > 4095) value_12bit = 4095;
    DAC_SetChannel1Data(DAC_Align_12b_R, value_12bit);
  
 }
 private:
  static uint8_t addresses_[ADC_CHANNEL_NUM_MUXED];
  
  uint16_t values_[ADC_CHANNEL_LAST];
  int32_t index_;
  int32_t last_read_;
  bool state_;
  
  DISALLOW_COPY_AND_ASSIGN(Adc);
};

}  // namespace rings

#endif  // RINGS_DRIVERS_ADC_H_
