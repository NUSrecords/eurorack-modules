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
// User interface.

#include "rings/ui.h"

#include <algorithm>
#include <stdint.h> 
#include "stmlib/system/system_clock.h"
#include "rings/cv_scaler.h"
#include "rings/dsp/part.h"
#include "rings/dsp/string_synth_part.h"

namespace rings {

const int32_t kLongPressDuration = 3000;

using namespace std;
using namespace stmlib;


void Ui::Init(
    Settings* settings,
    CvScaler* cv_scaler,
    Part* part,
    StringSynthPart* string_synth) {
  leds_.Init();
  switches_.Init();
  
  settings_ = settings;
  cv_scaler_ = cv_scaler;
  part_ = part;
  string_synth_ = string_synth;
  
  if (switches_.pressed_immediate(1)) {
    State* state = settings_->mutable_state();
    if (state->color_blind == 1) {
      state->color_blind = 0; 
    } else {
      state->color_blind = 1; 
    }
    settings_->Save();
  }
  
  part_->set_polyphony(settings_->state().polyphony);
  part_->set_model(static_cast<ResonatorModel>(settings_->state().model));
  string_synth_->set_polyphony(settings_->state().polyphony);
  string_synth_->set_fx(static_cast<FxType>(settings_->state().model));
  mode_ = settings_->state().easter_egg
      ? UI_MODE_EASTER_EGG_INTRO
      : UI_MODE_NORMAL;

        // Inicializace výstupního DAC na PA4 hned při startu UI na klidový střed
  cv_scaler_->adc().write_dc_offset(1001);    
  
}

void Ui::PollServo() {
    
    static const int16_t cal_offset_main = 9881; 
    static int32_t minuly_read = -1;

    // POUZE JEDEN SPOLEČNÝ VÝSTUPNÍ MOTOR PRO DAC
    static float motor_dac = 20742.0f; 
    static float speed_hard = 2.0f;
    static float peak_positive = 9881.0f;
    static float peak_negative = 9881.0f;
//spínej když je čerstvá hodnota z analizy (pote projede Pollservo funkce od začatku do konce )
    int32_t aktualni_read = cv_scaler_->adc().last_read();
    if (aktualni_read == ADC_CHANNEL_ANALYZE_AUDIO_5 && minuly_read != ADC_CHANNEL_ANALYZE_AUDIO_5) {
   
    uint32_t raw_sample = cv_scaler_->adc().raw_analyze_audio(); 
    float raw_p = static_cast<float>(raw_sample);
 
        // 1. PÍKOVÝ DETEKTOR
        if (raw_p > peak_positive) {
            peak_positive = raw_p;
        } else {
            peak_positive -= 0.05f * (peak_positive - raw_p);
        }
        if (raw_p < peak_negative) {
            peak_negative = raw_p;
        } else {
            peak_negative += 0.05f * (raw_p - peak_negative);
        }
        float ultra_filtered_adc = (peak_positive + peak_negative) * 0.5f; 

        bool realita_je_vysoko = (ultra_filtered_adc > static_cast<float>(cal_offset_main));
        float odchylka = realita_je_vysoko ? (ultra_filtered_adc - static_cast<float>(cal_offset_main)) 
                                           : (static_cast<float>(cal_offset_main) - ultra_filtered_adc);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
        static bool rezim_hruby = true;

        if (rezim_hruby && odchylka < 274.0) {
            rezim_hruby = false; // Nalítne rychle k cca 100 mV a vypne  
        } else if (!rezim_hruby && odchylka > 274.5) {
            rezim_hruby = true;  // Znovu zabere hru0bý, až když signál uteče nad ~250 mV
        }


        if (rezim_hruby) {  
            float cilova_rychlost_hard = odchylka * 0.002f;
            if (cilova_rychlost_hard > 64.0f) cilova_rychlost_hard = 64.0f;
            if (cilova_rychlost_hard < 2.0f)   cilova_rychlost_hard = 2.0f;

            if (speed_hard < cilova_rychlost_hard) {
                speed_hard *= 1.3f; 
                if (speed_hard > cilova_rychlost_hard) speed_hard = cilova_rychlost_hard;
            } else if (speed_hard > cilova_rychlost_hard) {
                speed_hard *= 0.5f; 
                if (speed_hard < cilova_rychlost_hard) speed_hard = cilova_rychlost_hard;
            }

            if (realita_je_vysoko) { 
                motor_dac -= speed_hard; 
            } else {
                motor_dac += speed_hard;
            }

        } else {  
            // ==========================================
            // REŽIM B: VAŠE LUPA S DITHEREM (jsme blízko)
            // ==========================================
            const uint32_t MAX_STUPNICE = 65523;
            const uint32_t POLOVINA_OKNA = (MAX_STUPNICE / 32) / 2; // cca 1023 dílků kolem středu 
            //POLOVINA_OKNA = (65523 / 32) / 2 = 1023 dílků

            uint32_t stred_bits  = cal_offset_main;
            uint32_t signal_bits = static_cast<uint32_t>(ultra_filtered_adc);
            uint32_t x2_spodni   = (stred_bits > POLOVINA_OKNA) ? (stred_bits - POLOVINA_OKNA) : 0;
            uint32_t x1_horni    = std::min(stred_bits + POLOVINA_OKNA, MAX_STUPNICE);
            uint32_t rozsah_okna = x1_horni - x2_spodni; //Spodní hranice (x2 x2): 9024−1023=8001  :  9024+1023=10047
      
            uint32_t virtual_signal_bits;
            uint32_t virtual_stred_bits;
static float speed_easy = 0.5f;
        if (signal_bits >= x2_spodni && signal_bits <= x1_horni) {
            // Čisté lineární roztažení okna 1/32 na plný rozsah 0-65523 (bez dither šumu)
            virtual_signal_bits = ((signal_bits - x2_spodni) * MAX_STUPNICE) / rozsah_okna;
            virtual_stred_bits  = ((stred_bits  - x2_spodni) * MAX_STUPNICE) / rozsah_okna;
        } else {
            virtual_signal_bits = signal_bits;
            virtual_stred_bits  = stred_bits;
        } 

           int32_t odchylka_lupy_int = static_cast<int32_t>(virtual_signal_bits) - static_cast<int32_t>(virtual_stred_bits);
           bool realita_v_lupe_vysoko = (odchylka_lupy_int > 0);
           uint32_t odchylka_lupy = (odchylka_lupy_int < 0) ? -odchylka_lupy_int : odchylka_lupy_int; //popiš co to přesne udelá matematicky se stupnicí a sparametry §
         
               speed_easy = static_cast<float>(odchylka_lupy) * 0.00005f;
            // Ochranné limity pružiny:
            if (speed_easy > 5.0f)  speed_easy = 5.0f;  // Maximální tah (aby nepřekmitla
            // Mrtvé pásmo (klid pružiny pod cca 12 mV):
            if (odchylka_lupy > 0.004) {
                if (realita_v_lupe_vysoko) {
                    motor_dac -= speed_easy;
                } else {
                    motor_dac += speed_easy;
                }
            }
        }

        if (motor_dac > 65535.0f) motor_dac = 65535.0f;
        if (motor_dac < 0.0f)     motor_dac = 0.0f;

        // Odeslání do DAC
        cv_scaler_->adc().write_dc_offset(static_cast<uint16_t>(motor_dac));
    }
    minuly_read = aktualni_read;     
}


void Ui::SaveState() {
  settings_->mutable_state()->polyphony = part_->polyphony();
  settings_->mutable_state()->model = part_->model();
  settings_->Save();
}

void Ui::AnimateEasterEggLeds() {
  mode_ = settings_->state().easter_egg
      ? UI_MODE_EASTER_EGG_INTRO
      : UI_MODE_EASTER_EGG_OUTRO;
}

void Ui::Poll() { //Sekundary funktions "NO NON-STOP !"
  // 1kHz.
  system_clock.Tick();
  switches_.Debounce();
  
  for (uint8_t i = 0; i < kNumSwitches; ++i) {
    if (switches_.just_pressed(i)) {
      queue_.AddEvent(CONTROL_SWITCH, i, 0);
      press_time_[i] = system_clock.milliseconds();
    }
    if (switches_.pressed(i) && press_time_[i] != 0) {
      int32_t pressed_time = system_clock.milliseconds() - press_time_[i];
      if (pressed_time > kLongPressDuration) {
        queue_.AddEvent(CONTROL_SWITCH, i, pressed_time);
        press_time_[i] = 0;
      }
    }
    if (switches_.released(i) && press_time_[i] != 0) {
      queue_.AddEvent(
          CONTROL_SWITCH,
          i,
          system_clock.milliseconds() - press_time_[i] + 1);
      press_time_[i] = 0;
    }
  }
  
  bool blink = (system_clock.milliseconds() & 127) > 64;
  bool slow_blink = (system_clock.milliseconds() & 255) > 128;
  switch (mode_) {
    case UI_MODE_NORMAL:
      {
        uint8_t pwm_counter = system_clock.milliseconds() & 15;
        uint8_t triangle = (system_clock.milliseconds() >> 5) & 31;
        triangle = triangle < 16 ? triangle : 31 - triangle;

        if (settings_->state().color_blind == 1) {
          uint8_t mode_red_brightness[] = {
            0, 15, 1,
            0, triangle, uint8_t(triangle >> 3)
          };
          uint8_t mode_green_brightness[] = {
            4, 15, 0, 
            uint8_t(triangle >> 1), triangle, 0,
          };
          
          uint8_t poly_counter = (system_clock.milliseconds() >> 7) % 12;
          uint8_t poly_brightness = (poly_counter >> 1) < part_->polyphony() &&
                (poly_counter & 1);
          uint8_t poly_red_brightness = part_->polyphony() >= 2
              ? 8 + 8 * poly_brightness
              : 0;
          uint8_t poly_green_brightness = part_->polyphony() <= 3
              ? 8 + 8 * poly_brightness
              : 0;
          if (part_->polyphony() == 1 || part_->polyphony() == 4) {
            poly_red_brightness >>= 3;
            poly_green_brightness >>= 2;
          }
          leds_.set(
              0,
              pwm_counter < poly_red_brightness,
              pwm_counter < poly_green_brightness);
          leds_.set(
              1,
              pwm_counter < mode_red_brightness[part_->model()],
              pwm_counter < mode_green_brightness[part_->model()]);
        } else {
          leds_.set(0, part_->polyphony() >= 2, part_->polyphony() <= 2);
          leds_.set(1, part_->model() >= 1, part_->model() <= 1);
          // Fancy modes!
          if (part_->polyphony() == 3) {
            leds_.set(0, true, pwm_counter < triangle);
          }
          if (part_->model() >= 3) {
            bool led_1 = part_->model() >= 4 && pwm_counter < triangle;
            bool led_2 = part_->model() <= 4 && pwm_counter < triangle;
            leds_.set(1, led_1, led_2);
          }
        }
        ++strumming_flag_interval_;
        if (strumming_flag_counter_) {
          --strumming_flag_counter_;
          leds_.set(0, false, false);
        }
      }
      break;
    
    case UI_MODE_CALIBRATION_C1:
      leds_.set(0, blink, blink);
      leds_.set(1, false, false);
      break;

    case UI_MODE_CALIBRATION_C3:
      leds_.set(0, false, false);
      leds_.set(1, blink, blink);
      break;

    case UI_MODE_CALIBRATION_LOW:
      leds_.set(0, slow_blink, 0);
      leds_.set(1, slow_blink, 0);
      break;

    case UI_MODE_CALIBRATION_HIGH:
      leds_.set(0, false, slow_blink);
      leds_.set(1, false, slow_blink);
      break;
    
    case UI_MODE_EASTER_EGG_INTRO:
      {
        uint8_t pwm_counter = system_clock.milliseconds() & 15;
        uint8_t triangle_1 = (system_clock.milliseconds() / 7) & 31;
        uint8_t triangle_2 = (system_clock.milliseconds() / 17) & 31;
        triangle_1 = triangle_1 < 16 ? triangle_1 : 31 - triangle_1;
        triangle_2 = triangle_2 < 16 ? triangle_2 : 31 - triangle_2;
        leds_.set(
            0,
            triangle_1 > pwm_counter,
            triangle_2 > pwm_counter);
        leds_.set(
            1,
            triangle_2 > pwm_counter,
            triangle_1 > pwm_counter);
      }
      break;

    case UI_MODE_EASTER_EGG_OUTRO:
      {
        uint8_t pwm_counter = 7;
        uint8_t triangle_1 = (system_clock.milliseconds() / 9) & 31;
        uint8_t triangle_2 = (system_clock.milliseconds() / 13) & 31;
        triangle_1 = triangle_1 < 16 ? triangle_1 : 31 - triangle_1;
        triangle_2 = triangle_2 < 16 ? triangle_2 : 31 - triangle_2;
        leds_.set(0, triangle_1 < pwm_counter, triangle_1 > pwm_counter);
        leds_.set(1, triangle_2 > pwm_counter, triangle_2 < pwm_counter);
      }
      break;
    
    case UI_MODE_PANIC:
      leds_.set(0, blink, false);
      leds_.set(1, blink, false);
      break;
  }
  leds_.Write();
}

void Ui::FlushEvents() {
  queue_.Flush();
}

void Ui::OnSwitchPressed(const Event& e) {

}

void Ui::OnSwitchReleased(const Event& e) {
  // Check if the other switch is still pressed.
  if (switches_.pressed(1 - e.control_id)) {
    if (mode_ == UI_MODE_CALIBRATION_C1) {
      StartNormalizationCalibration();
    } else {
      StartCalibration();
    }
    press_time_[0] = press_time_[1] = 0;
    return;
  }
  
  switch (e.control_id) {
    case 0:
      if (e.data >= kLongPressDuration) {
        if (cv_scaler_->easter_egg()) {
          settings_->ToggleEasterEgg();
          AnimateEasterEggLeds();
        } else {
          part_->set_polyphony(3);
          string_synth_->set_polyphony(3);
        }
        SaveState();
      } else {
        switch (mode_) {
          case UI_MODE_CALIBRATION_C1:
            CalibrateC1();
            break;
          case UI_MODE_CALIBRATION_C3:
            CalibrateC3();
            break;
          case UI_MODE_CALIBRATION_LOW:
            CalibrateLow();
            break;
          case UI_MODE_CALIBRATION_HIGH:
            CalibrateHigh();
            break;
          default:
            {
              int32_t polyphony = part_->polyphony();
              if (polyphony == 3) {
                polyphony = 2;
              }
              polyphony <<= 1;
              if (polyphony > 4) {
                polyphony = 1;
              }
              part_->set_polyphony(polyphony);
              string_synth_->set_polyphony(polyphony);
              SaveState();
            }
            break;
          }
      }
      break;
    
    case 1:
      if (e.data >= kLongPressDuration) {
        if (cv_scaler_->easter_egg()) {
          settings_->ToggleEasterEgg();
          AnimateEasterEggLeds();
        } else {
          int32_t model = part_->model();
          if (model >= 3) {
            model -= 3;
          } else {
            model += 3;
          }
          part_->set_model(static_cast<ResonatorModel>(model));
          string_synth_->set_fx(static_cast<FxType>(model));
        }
      } else {
        int32_t model = part_->model();
        if (model >= 3) {
          model -= 3;
        } else {
          model = (model + 1) % 3;
        }
        part_->set_model(static_cast<ResonatorModel>(model));
        string_synth_->set_fx(static_cast<FxType>(model));
      }
      SaveState();
      break;
    
    default:
      break;
  }
}

void Ui::StartCalibration() {
  mode_ = UI_MODE_CALIBRATION_C1;
}

void Ui::CalibrateC1() {
  cv_scaler_->CalibrateC1();
  cv_scaler_->CalibrateOffsets();
  mode_ = UI_MODE_CALIBRATION_C3;
}

void Ui::CalibrateC3() {
  bool success = cv_scaler_->CalibrateC3();
  if (success) {
    settings_->Save();
    mode_ = UI_MODE_NORMAL;
  } else {
    mode_ = UI_MODE_PANIC;
  }
}

void Ui::StartNormalizationCalibration() {
  cv_scaler_->StartNormalizationCalibration();
  mode_ = UI_MODE_CALIBRATION_LOW;
}

void Ui::CalibrateLow() {
  cv_scaler_->CalibrateLow();
  mode_ = UI_MODE_CALIBRATION_HIGH;
}

void Ui::CalibrateHigh() {
  bool success = cv_scaler_->CalibrateHigh();
  if (success) {
    settings_->Save();
    mode_ = UI_MODE_NORMAL;
  } else {
    mode_ = UI_MODE_PANIC;
  }
}

void Ui::DoEvents() {
  while (queue_.available()) {
    Event e = queue_.PullEvent();
    if (e.control_type == CONTROL_SWITCH) {
      if (e.data == 0) {
        OnSwitchPressed(e);
      } else {
        OnSwitchReleased(e);
      }
    }
  }
  if (queue_.idle_time() > 800 && mode_ == UI_MODE_PANIC) {
    mode_ = UI_MODE_NORMAL;
  }
  if (mode_ == UI_MODE_EASTER_EGG_INTRO || mode_ == UI_MODE_EASTER_EGG_OUTRO) {
    if (queue_.idle_time() > 3000) {
      mode_ = UI_MODE_NORMAL;
      queue_.Touch();
    }
  } else if (queue_.idle_time() > 1000) {
    queue_.Touch();
  }
}

uint8_t Ui::HandleFactoryTestingRequest(uint8_t command) {
  uint8_t argument = command & 0x1f;
  command = command >> 5;
  uint8_t reply = 0;
  switch (command) {
    case FACTORY_TESTING_READ_POT:
    case FACTORY_TESTING_READ_CV:
      reply = cv_scaler_->adc_value(argument);
      break;
    
    case FACTORY_TESTING_READ_NORMALIZATION:
      reply = cv_scaler_->normalization(argument);
      break;      
    
    case FACTORY_TESTING_READ_GATE:
      reply = argument == 2
          ? cv_scaler_->gate_value()
          : switches_.pressed(argument);
      break;
      
    case FACTORY_TESTING_SET_BYPASS:
      part_->set_bypass(argument);
      break;
      
    case FACTORY_TESTING_CALIBRATE:
      {
        switch (argument) {
          case 0:
            StartCalibration();
            break;
          
          case 1:
            CalibrateC1();
            break;
          
          case 2:
            CalibrateC3();
            break;
          
          case 3:
            StartNormalizationCalibration();
            break;

          case 4:
            CalibrateLow();
            break;
          
          case 5:
            CalibrateHigh();
            queue_.Touch();
            break;
        }
      }
      break;
  }
  return reply;
}

}  // namespace rings

