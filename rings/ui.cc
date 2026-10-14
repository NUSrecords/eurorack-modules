// Copyright 2015 Emilie Gillet.
//
// Author: Emilie Gillet 
// -----------------------------------------------------------------------------
//nejsem si jistý jestl je dobré mít smoothed_dac_main za lupou ,jestl by nebylo lepší ji dat pře lupu a neco za lupu ??? a nebo jdou už do lupy zruměrované data ... hlavně zjisti co zpusobilo tu nestabilitu ....obecně nedbej na drobné chby....

//static float auto_dac_main  -ještě nevím jak ho použijem ,zatím je to vystup z lupy asi ... 
//static float ultra_filtered_adc hodnota REEL offset filtrovaná z (adc_attenuverter_frequenci) 
//tahle hodnota je asi neustále proměná a odečítá se nebo přičítá podle stavu offsetu od pevné hodnoty : cal_offset_main 
// AUDIO OFFSET 0 DC (trashold audio DC outpute for MAIN and AUX)

//static int16_t cal_offset_main -- tato hodnota se dá pouze měnit ručním přepisem , nebo novým kalibračním režimem 
//static int16_t cal_offset_main --- HODNOTA ZUSTAVA JAKO MĚŘÍTKO NEMĚNÁ  IDEALNÍ HODNOTU VOLT VZTUPU DO ADC FREQUENCI KTERÁ JE NASTAVENA NA OFFSET 0V DC AUDIO OUT

//(starý kalibrační režim pro kalibraci tonu ze sequenceru je vyřazen a překonán novým online-vagrant ,který upraví podle zadání ladění celí firmware a zkompluje)
//-----------------------------
// Vagrant- only for OPTO-RINGS V4.0 : https://dubtechno.co/?page_id=682


#include "rings/ui.h"
#include <algorithm>
#include <stdint.h>
#include "stmlib/system/system_clock.h"
#include "stmlib/system/storage.h" // Nezávislá perzistentní paměť FLASH tohle bylo přidáno

#include "rings/cv_scaler.h"
#include "rings/dsp/part.h"
#include "rings/dsp/string_synth_part.h"

namespace rings {

const int32_t kLongPressDuration = 3000;

using namespace std;
using namespace stmlib;



// ============================================================================
//  STRUKTURY PRO PERZISTENTNÍ UKLÁDÁNÍ DO PAMĚTI FLASH
// ============================================================================
struct ServoStorageData {
  int16_t saved_cal_offset_main;
  uint16_t padding; // Držíme 4-bajtové zarovnání struktury
};


// Inicializace nezávislého úložiště mimo tónové settings
stmlib::Storage<2> servo_storage;
static ServoStorageData servo_flash_data;
static uint16_t servo_version_token = 0;


// Globální stavové proměnné pro perzistentní auto-ukládání serva
bool servo_data_changed = false; 
static int32_t save_timeout __attribute__((unused)) = 0;


// ============================================================================
//  DEFINICE PARAMETRŮ PRO SERVO AUDIO OFFSET CORCT
// ============================================================================

// --- SVĚT ADC (Měřítko vstupu) ---
// NEMĚNNÉ MĚŘÍTKO. Ideální hodnota voltů na vstupu do ADC, 
// která odpovídá stavu, kdy je na audio výstupu přesně 0V DC.
static int16_t cal_offset_main = 552;
// HODNOTA REEL OFFSET FILTROVANÁ Z ADC (Ultra-zprůměrovaný výstup z filtru).
// podle teplotního driftu JFETu a přičítá/odečítá se vůči cal_offset_main.
static float ultra_filtered_adc = 0.0f;
// / --- SVĚT DAC (Řízení serva) ---
//  Pevný, samostatný startovací bod, který při inicializaci vypálíme do DAC,
//   aby hardware držel stabilní DC stůl a neletěl do saturace, než najede systém.
//static float dac_start_voltage = 1001.0f; 
// Výstup z lupy (Vnitřní stav, plynulý posun bez kumulace chyb). 'DAC'
static float auto_dac_main = 1001.0f;   // 2048.0f;  napsali jste přednabít proč ?? navíc by to bylo : 1001,0f !! 
//static float smoothed_dac_start = 0.0f;   // Obyčejný startovní vyhlazovací registr 
float out4 = 1001.0f; 
// true = hardware drží startovací stůl, false = start je odpojen, jede čistá lupa
//static bool servo_startup_gate = true; 
//přichozí parametry ADC "horní a spodní špička"
//static float peak_positive = 552.0f;
//static float peak_negative = 552.0f;
// VNITŘNÍ MOTOR NAVIJÁKU: Pamatuje si aktuální plynulou rychlost přitahování.
// Tohle je ten klíč – rychlost se nemění skokem, ale plynule dotéká!
static float rychlost_navijaku = 1.3f;
// ============================================================================
//  PŘESNÉ KOEFICIENTY PRO 4x BIQUAD KASKÁDU (Celkové Fp = 1.4625Hz @ 48 dB/oct)
// ============================================================================
// Tyto koeficienty jsou upravené tak, aby po zařazení 4 stupňů za sebou
// byl výsledný pokles -3dB přesně na frekvenci 1.4625 Hz.
/*const float b0 = 0.000047f;
const float b1 = 0.000094f;
const float b2 = 0.000047f;
const float a1 = -1.980562f;
const float a2 = 0.980750f;
// --- PAMĚŤOVÉ REGISTRY HISTORIE PRO VŠECHNY 4 STUPNĚ ---
static float biquad1_x1 = 1001.0f, biquad1_x2 = 1001.0f, biquad1_y1 = 1001.0f, biquad1_y2 = 1001.0f;
static float biquad2_x1 = 1001.0f, biquad2_x2 = 1001.0f, biquad2_y1 = 1001.0f, biquad2_y2 = 1001.0f;
static float biquad3_x1 = 1001.0f, biquad3_x2 = 1001.0f, biquad3_y1 = 1001.0f, biquad3_y2 = 1001.0f;
static float biquad4_x1 = 1001.0f, biquad4_x2 = 1001.0f, biquad4_y1 = 1001.0f, biquad4_y2 = 1001.0f;

*/

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

InitServoHardware();
}


 void Ui::InitServoHardware() {
/*
     // Načteme data čistě z našeho nezávislého Sektoru 2
 // if (servo_storage.ParsimoniousLoad(&servo_flash_data, &servo_version_token)) {
 //   cal_offset_main = servo_flash_data.saved_cal_offset_main;
//  }
 // 1. Přednabijeme startovní registr na cílovou hodnotu
  smoothed_dac_start = dac_start_voltage;

  // Obyčejný jednoduchý vyhlazovací filtr pro plynulý náběh startu za studena
  smoothed_dac_start += 0.01f * (dac_start_voltage - smoothed_dac_start);
  // 3. Aktivujeme startovní bránu
  servo_startup_gate = true;
  // 4. Práskneme bezpečné vyhlazené startovní napětí rovnou ven, ať je JFET okamžitě chráněn
  uint16_t hardware_dac_register = static_cast<uint16_t>(smoothed_dac_start);
  cv_scaler_->adc_.set_value(ADC_CHANNEL_ATTENUVERTER_DAMPING, hardware_dac_register); 
  */
}


void Ui::PollServo() {
  cv_scaler_->adc_.ConvertServoMux(); ///??????????????????????????????????????? 
   // Načteme surovou 12-bitovou hodnotu serva z indexu 1
  float raw_sample = static_cast<float>(cv_scaler_->adc_.value(11)); 
  ultra_filtered_adc = raw_sample;

  // ==========================================================================
  // SPOLEČNÝ VÝPOČET ROZDÍLU OFFSETU PRO VŠECHNY LUPY""
  // ==========================================================================
   // ---  ADC (Měřítko vstupu) ---
    // 1. Zjistíme pouze směr odchylky (Zda jsme nad nebo pod)
    bool realita_je_vysoko = (ultra_filtered_adc > static_cast<float>(cal_offset_main));
    // 1. Výpočet absolutní odchylky (Zde vypadne čisté kladné číslo)
    float odchylka = realita_je_vysoko ? (ultra_filtered_adc - static_cast<float>(cal_offset_main)) 
                                   : (static_cast<float>(cal_offset_main) - ultra_filtered_adc);

// 2. Plynulé určení cílové rychlosti podle velikosti chyby
    float cilova_rychlost = odchylka * 0.1f;
    if (cilova_rychlost < 0.05f) {
    cilova_rychlost = 0.05f; // Pojistka proti záseku
    }

// 3. Exponenciální setrvačnost navijáku (Zrychlování a brždění)
    if (rychlost_navijaku < cilova_rychlost) {
    rychlost_navijaku *= 1.3f; 
    if (rychlost_navijaku > cilova_rychlost) rychlost_navijaku = cilova_rychlost;
    } else if (rychlost_navijaku > cilova_rychlost) {
    rychlost_navijaku *= 0.7f; 
    if (rychlost_navijaku < cilova_rychlost) rychlost_navijaku = cilova_rychlost;
    }
    // 4. PROVEDENÍ KROKU
    // Do DAC se přičítá/odčítá vyhlazená rychlost, nikoliv odchylka z ADC!
    if (odchylka > 0.2f) {
    if (realita_je_vysoko) {
        auto_dac_main -= rychlost_navijaku;
    } else {
        auto_dac_main += rychlost_navijaku;
    }
 }
    out4 = auto_dac_main;
    // 5. HARDWAROVÁ POJISTKA (Anti-windup)
    if (auto_dac_main > 4095.0f) auto_dac_main = 4095.0f;
    if (auto_dac_main < 0.0f)    auto_dac_main = 0.0f;

    

/*
  // ==========================================================================
  // --- KORIDOROVÝ PŘEPÍNAČ: KAM TEN HOTOVÝ PARAMETR POŠLEME ---
  // ==========================================================================
  //zoom = lupa a norma je normal bez lupy paralelní chod
  float filter_input_value = 0.0f; 
  if (!(auto_dac_main >= 2730.0f && auto_dac_main <= 2978.0f)) {
    // ------------------------------------------------------------------------
    // REŽIM A: NORMÁLNÍ CHOD (Hrubý rozsah 0 - 3.3V -> Plných 4095 bitů)
    // ------------------------------------------------------------------------
   filter_input_value = auto_dac_main;

    } else {  //a nebo to pošleme do lupy ... 

////přidal jsem proměné podívej snad nebudou potřeba na hoře difinovat nebo ano ?///

    // ------------------------------------------------------------------------
    // REŽIM B: EXTRÉMNÍ LUPA (Nové přesné okno 2.2V - 2.4V -> ZNOVU 4095 KROKŮ!)
    // ------------------------------------------------------------------------
    // Signál je stabilizovaný za tepla blízko nuly. 
    // Přichází tvůj upravený parametr a v plné float přesnosti ho zrcadlíme do okna:
    // Spodní stop (2.2V) = 2730.0f. Horní stop (2.4V) = 2978.0f. Rozsah = 248.0f.
    float exact_hardware_position = 2730.0f - ((auto_dac_main * 248.0f) / 4095.0f); 
    //float exact_hardware_position = 2730.0f + (((auto_dac_main - 2730.0f) * 4095.0f) / 248.0f); 
    
    // 1kHz hardwarové ditherování (přeměna času na brutální rozlišení  mikrovoltů na krok!)
    uint16_t base_step = static_cast<uint16_t>(exact_hardware_position);
    float fractional_part = exact_hardware_position - static_cast<float>(base_step);

    static uint32_t dither_seed = 0x12345678;
    dither_seed = dither_seed * 1103515245 + 12345;
    float random_threshold = static_cast<float>(dither_seed & 0xFFFF) / 65535.0f;

    // Horní pojistka je teď omezena tvým novým stropem 2978 (2.4V)
    if (fractional_part > random_threshold && base_step < 2978) {
      filter_input_value = base_step + 1; // Pulzně dýcháme o 48?? mikrovoltů výš
    } else {
      filter_input_value = static_cast<float>(base_step);       // Držíme krok
    }
  }
  
 // ==========================================================================
  //  SPOLEČNÝ 4-STUPEŇ BESSEL KASKÁDY (48 dB/oct @ Fp = 1.4625Hz) - HLAVNÍ ŠTÍT
  // ==========================================================================
  // Hodnota z přepínače prochází kaskádou 4 biquadů za sebou.
  // Odstraní VF zvlnění a veškerý přechodový šum těsně před výstupem z multiplexeru.

  // 1. STUPEŇ
  float out1 = b0 * filter_input_value + b1 * biquad1_x1 + b2 * biquad1_x2 - a1 * biquad1_y1 - a2 * biquad1_y2;
  biquad1_x2 = biquad1_x1; biquad1_x1 = filter_input_value; biquad1_y2 = biquad1_y1; biquad1_y1 = out1;

  // 2. STUPEŇ
  float out2 = b0 * out1 + b1 * biquad2_x1 + b2 * biquad2_x2 - a1 * biquad2_y1 - a2 * biquad2_y2;
  biquad2_x2 = biquad2_x1; biquad2_x1 = out1; biquad2_y2 = biquad2_y1; biquad2_y1 = out2;

  // 3. STUPEŇ
  float out3 = b0 * out2 + b1 * biquad3_x1 + b2 * biquad3_x2 - a1 * biquad3_y1 - a2 * biquad3_y2;
  biquad3_x2 = biquad3_x1; biquad3_x1 = out2; biquad3_y2 = biquad3_y1; biquad3_y1 = out3;

  // 4. STUPEŇ (Konečný, dokonale čistý stejnosměrný parametr)
  float out4 = b0 * out3 + b1 * biquad4_x1 + b2 * biquad4_x2 - a1 * biquad4_y1 - a2 * biquad4_y2;
  biquad4_x2 = biquad4_x1; biquad4_x1 = out3; biquad4_y2 = biquad4_y1; biquad4_y1 = out4;
*/
  // Bezpečnostní oříznutí finálního parametru na hardwarový rozsah 12-bit DAC
  if (out4 > 4095.0f) out4 = 4095.0f;
  if (out4 < 0.0f)    out4 = 0.0f;
///////////////////Společný výstupní filter pro lupu a normal posílání "The END"//////////////////

  // ==========================================================================
  // JEDINÝ SPOLEČNÝ VYSÍLAČ NA HARDWARE (Zápis čistého DC na pin damping 0 0 0 0)
  cv_scaler_->adc_.set_value(ADC_CHANNEL_ATTENUVERTER_DAMPING, static_cast<uint16_t>(out4));
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

void Ui::Poll() {
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
  (void)slow_blink;
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

          // OPRAVENO: V originálu blikaly LEDky oranžově, takže pouštíme červenou i zelenou naráz!
    case UI_MODE_CALIBRATION_SERVO:
      leds_.set(0, blink, blink); // LED 0 bliká oranžově (červená + zelená)
      leds_.set(1, blink, blink); // LED 1 bliká oranžově (červená + zelená)
      break;

      case UI_MODE_CALIBRATION_C1:
  break;

    case UI_MODE_CALIBRATION_C3:
      leds_.set(0, false, false);
      leds_.set(1, false, false);
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
     // PŘIDÁNO SEM: Prázdné větve pro původní kalibraci jacků + TVŮJ NOVÝ REŽIM SERVA.
    // Tím kompletně zmizely ty dvě chyby "enumeration value not handled in switch"!
    case UI_MODE_CALIBRATION_LOW:
    case UI_MODE_CALIBRATION_HIGH:
      break;
      }
      leds_.Write();
      PollServo(); 
  }

void Ui::FlushEvents() {
  queue_.Flush();
}

void Ui::OnSwitchPressed(const Event& e) {
  // --- VSTUPNÍ BRÁNA ZA PROVOZU: Stiskem obou tlačítek naráz let意的 do kalibrace C1
  if (mode_ == UI_MODE_NORMAL && switches_.pressed(1 - e.control_id)) {
    mode_ = UI_MODE_CALIBRATION_SERVO; // Aktivujeme režim kalibrace, kde fungují tlačítka
    press_time_[0] = press_time_[1] = 0;
    queue_.Touch();
    return;
  }
}

  
void Ui::OnSwitchReleased(const Event& e) {
  // UKONČENÍ A OKAMŽITÝ ZÁPIS: Pokud jsme v kalibraci serva a stiskneme OBĚ tlačítka naráz,
  // kód bezpečně zapíše novou naklikanou nulu do izolovaného Sektoru 2 a skočíme zpět do provozu!
  if (mode_ == UI_MODE_CALIBRATION_SERVO && switches_.pressed(1 - e.control_id)) {
    servo_flash_data.saved_cal_offset_main = cal_offset_main;
    servo_storage.ParsimoniousSave(servo_flash_data, &servo_version_token); // Izolovaný zápis bez zamrznutí
    
    //  OPRAVENO: Správná syntaxe přetypování na float!
    auto_dac_main = static_cast<float>(cal_offset_main);
    
    mode_ = UI_MODE_NORMAL;
    press_time_[0] = press_time_[1] = 0;
    queue_.Touch();
    return;
  }
  
  // MANUÁLNÍ KLIKÁNÍ THRESHOLDU PO JEDNOM NEJMENŠÍM KROKU (ZCELA ODPOJENO OD TÓNŮ A DSP.H)
  if (mode_ == UI_MODE_CALIBRATION_SERVO) {
    if (e.control_id == 0) {
      cal_offset_main += 1; // Tlačítko 0 ladí o krok +1 nahoru
      if (cal_offset_main > 4095) cal_offset_main = 4095;
    } else if (e.control_id == 1) {
      cal_offset_main -= 1; // Tlačítko 1 ladí o krok -1 dolů
      if (cal_offset_main < 0) cal_offset_main = 0;
    }
    return; // Vyskočíme, ať se neovlivní polyfonie níže
  }

  // ORIGINÁLNÍ ZACHOVANÁ KALIBRACE DETEKCE NORMALIZACE JACKŮ (DRŽENÍ PRAVÉHO TLAČÍTKA PŘI BOOTU)
  if (switches_.pressed(1 - e.control_id)) {
    if (mode_ == UI_MODE_CALIBRATION_LOW) {
      cv_scaler_->CalibrateLow();
      mode_ = UI_MODE_CALIBRATION_HIGH;
    } else if (mode_ == UI_MODE_CALIBRATION_HIGH) {
      bool success = cv_scaler_->CalibrateHigh();
      if (success) {
        settings_->Save(); // Zápis parametrů jacků do settings Sektor 1
        mode_ = UI_MODE_NORMAL;
      } else {
        mode_ = UI_MODE_PANIC;
      }
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
        // OPRAVENO: Vráceno čisté a funkční točení polyfonie za běžného chodu modulu
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
        SaveState();
      } else {
        int32_t model = part_->model();
        if (model >= 3) {
          model -= 3;
        } else {
          model = (model + 1) % 3;
        }
        part_->set_model(static_cast<ResonatorModel>(model));
        string_synth_->set_fx(static_cast<FxType>(model));
        SaveState();
      }
      break;
    
    default:
      break;
     }
   }
 

void Ui::StartCalibration() { }

void Ui::CalibrateC1() { }

void Ui::CalibrateC3() { }

void Ui::StartNormalizationCalibration() { }

void Ui::CalibrateLow() { }

void Ui::CalibrateHigh() { }

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

