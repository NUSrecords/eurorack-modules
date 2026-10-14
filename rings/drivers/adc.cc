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

#include "rings/drivers/adc.h"

#include <stm32f4xx_conf.h>

namespace rings {

/* static */
#if defined(NO_APPLICATION)
// OPRAVA PRO BOOTLOADER:
uint8_t Adc::addresses_[ADC_CHANNEL_NUM_MUXED] = {
  7, // ADC_CHANNEL_POT_BRIGHTNESS
  5, // ADC_CHANNEL_POT_DAMPING
  3  // ADC_CHANNEL_POT_POSITION
};
#else
// OPRAVENO: Pozice 4, 5, 6, 7 jsou nahrazeny hardwarovou zacpávkou analyzátoru FREQUENCY (adresa 4).
// Multiplexer se na tomto kritickém pinu zdrží nejdelší čas, což zajistí maximální přesnost 
// a stabilitu pro real-DC servo měření teplotního driftu!
uint8_t Adc::addresses_[ADC_CHANNEL_NUM_MUXED] = {
  7, // ADC_CHANNEL_POT_BRIGHTNESS,         (index_ = 0 -> NETKNUTO)
  5, // ADC_CHANNEL_POT_DAMPING,            (index_ = 1 -> NETKNUTO)
  3, // ADC_CHANNEL_POT_POSITION,           (index_ = 2 -> NETKNUTO)
  4, // ADC_CHANNEL_ATTENUVERTER_FREQUENCY, (index_ = 3 -> tvůj AD analyzátor)
  0, // ADC_CHANNEL_ATTENUVERTER_DAMPING,
  0, // ADC_CHANNEL_ATTENUVERTER_DAMPING,
  0, // ADC_CHANNEL_ATTENUVERTER_DAMPING,
  0, // ADC_CHANNEL_ATTENUVERTER_DAMPING,
};
#endif
 static uint8_t servo_index = 0;  
void Adc::Init() {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
  
  DMA_InitTypeDef dma_init;
  ADC_CommonInitTypeDef adc_common_init;
  ADC_InitTypeDef adc_init;
  GPIO_InitTypeDef gpio_init;
  
  gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  gpio_init.GPIO_Pin |= GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_init.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOA, &gpio_init);

  gpio_init.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_init.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOC, &gpio_init);
  
  // Configure the address lines for the MUX.
  gpio_init.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  gpio_init.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &gpio_init);
  GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
  
  // Use DMA to automatically copy ADC data register to values_ buffer.
  dma_init.DMA_Channel = DMA_Channel_0;
  dma_init.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; 
  dma_init.DMA_Memory0BaseAddr = (uint32_t)&values_; 
  dma_init.DMA_DIR = DMA_DIR_PeripheralToMemory;
  dma_init.DMA_BufferSize = ADC_CHANNEL_NUM_DIRECT; 
  dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
  dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable; 
  dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
  dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
  dma_init.DMA_Mode = DMA_Mode_Circular; 
  dma_init.DMA_Priority = DMA_Priority_High;
  dma_init.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  dma_init.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  dma_init.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  dma_init.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &dma_init);
  DMA_Cmd(DMA2_Stream0, ENABLE);
  
  adc_common_init.ADC_Mode = ADC_Mode_Independent;
  adc_common_init.ADC_Prescaler = ADC_Prescaler_Div8;
  adc_common_init.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  adc_common_init.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&adc_common_init);
  // Nastavení ADC1 (Stará funkce zachována)
  adc_init.ADC_Resolution = ADC_Resolution_12b;
  adc_init.ADC_ScanConvMode = ENABLE;
  adc_init.ADC_ContinuousConvMode = DISABLE;
  adc_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  adc_init.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  adc_init.ADC_DataAlign = ADC_DataAlign_Left; // Zarovnání vlevo vyžaduje posun >> 4 pro zpracování
  adc_init.ADC_NbrOfConversion = ADC_CHANNEL_NUM_DIRECT;
  ADC_Init(ADC1, &adc_init);
  // Nastavení ADC2 (Izolováno pro tvůj kanál 11)
  adc_init.ADC_Resolution = ADC_Resolution_12b;
  adc_init.ADC_ScanConvMode = DISABLE;
  adc_init.ADC_ContinuousConvMode = DISABLE;
  adc_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  adc_init.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  adc_init.ADC_DataAlign = ADC_DataAlign_Left; // Zde je zarovnání vlevo
  adc_init.ADC_NbrOfConversion = 1;
  ADC_Init(ADC2, &adc_init);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_480Cycles); 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_480Cycles); 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_480Cycles); 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 5, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 6, ADC_SampleTime_480Cycles); 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 7, ADC_SampleTime_480Cycles); 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 8, ADC_SampleTime_480Cycles);

  ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_480Cycles);
  
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
   
  index_ = ADC_CHANNEL_NUM_MUXED - 1; //???????????????????????????????????????????????
  last_read_ = 0;
  state_ = false;
  
  Convert(); // První tovární start konverze
}

void Adc::DeInit() {
  DMA_Cmd(DMA2_Stream0, DISABLE);
  ADC_DMARequestAfterLastTransferCmd(ADC1, DISABLE);
  ADC_Cmd(ADC1, DISABLE);
  ADC_Cmd(ADC2, DISABLE);
  ADC_DMACmd(ADC1, DISABLE);
  ADC_DeInit();
}
/*

//////puvodní originální software ringsv3/////////

void Adc::Convert() {
  ADC_SoftwareStartConv(ADC1);
  
  if (state_) {
    // Read the value from the previous conversion.
    values_[ADC_CHANNEL_POT_BRIGHTNESS + index_] = ADC2->DR;
    last_read_ = index_;
    ++index_;
    if (index_ >= ADC_CHANNEL_NUM_MUXED) {
      index_ = 0;
    }
  
    uint8_t address = addresses_[index_];
  
    // Write the mux address.
    GPIO_WriteBit(GPIOB, GPIO_Pin_7, static_cast<BitAction>(address & 1));
    GPIO_WriteBit(GPIOB, GPIO_Pin_6, static_cast<BitAction>(address & 2));
    GPIO_WriteBit(GPIOB, GPIO_Pin_5, static_cast<BitAction>(address & 4));
  } else {
    ADC_SoftwareStartConv(ADC2);
  }
  
  state_ = !state_;
}
*/

// 1. STARÁ TOVÁRNÍ FUNKCE (Zůstává čistá a bezpečná pro Audio a CV přes DMA)
void Adc::Convert() {
  ADC_SoftwareStartConv(ADC1);
}

void Adc::ConvertServoMux() {
  uint8_t current_channel = 11 + servo_index;
  
  // 1. NEJDŘÍVE NAČTEME ADRESU (Čtení NAD posunem indexu)
  uint8_t address = addresses_[4 + servo_index];

  // 2. OKAMŽITĚ PŘEPNEME FYZICKÉ PINY MULTIPLEXERU NA DESCE
  GPIO_WriteBit(GPIOB, GPIO_Pin_7, static_cast<BitAction>(address & 1));
  GPIO_WriteBit(GPIOB, GPIO_Pin_6, static_cast<BitAction>(address & 2));
  GPIO_WriteBit(GPIOB, GPIO_Pin_5, static_cast<BitAction>(address & 4));

  // 3. HARDWAROVÁ PAUZA: Necháme analogové cesty na PCB i v chipu kompletně vyrovnat
  asm volatile("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");

  // 4. MĚŘENÍ REALITY NA KANÁLE 11 (S FILTRACÍ INTERNÍHO CROSSTALKU)
  if (current_channel == 11) {
    
    // --- 1. VZOREK: Spustíme konverzi pouze pro vybití parazitního náboje ze STRUCTURE ---
    ADC_SoftwareStartConv(ADC2);
    uint32_t timeout1 = 0xFFFF;
    while (ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET && timeout1--);
    
    // Čtení registru naprázdno - hodnotu vyhodíme do koše, je kontaminovaná!
    volatile uint16_t dummy_read = ADC2->DR; 
    (void)dummy_read; // Prevence warningu o nepoužité proměnné

    // Malá kratičká mezipauza pro stabilizaci vnitřního VREF
    asm volatile("nop; nop; nop; nop;");

    // --- 2. VZOREK: Teď je vnitřní sběrnice 100% čistá a usazená na napětí serva ---
    ADC_SoftwareStartConv(ADC2);
    uint32_t timeout2 = 0xFFFF;
    while (ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET && timeout2--);
    
    uint16_t servo_raw_value = ADC2->DR; 

    // Srovnání dat z Left Alignment na čistých 0-4095
    servo_raw_value = servo_raw_value >> 4; 

    // Uložení stoprocentně čisté hodnoty do paměti RAM
    values_[current_channel] = servo_raw_value;
  }

  // 5. POSUNEME INDEX PRO PŘÍŠTÍ KOLO
  ++servo_index;
  if (servo_index >= 5) {
    servo_index = 0;
  }
}

} // namespace rings