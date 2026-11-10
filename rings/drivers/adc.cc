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
uint8_t Adc::addresses_[ADC_CHANNEL_NUM_MUXED] = {
  7, // ADC_CHANNEL_POT_BRIGHTNESS,
  5, // ADC_CHANNEL_POT_DAMPING,
  3, // ADC_CHANNEL_POT_POSITION,

  4, //ADC_CHANNEL_ANALYZE_AUDIO_1, 
  4, //ADC_CHANNEL_ANALYZE_AUDIO_2, 
  4, //ADC_CHANNEL_ANALYZE_AUDIO_3, 
  4, //ADC_CHANNEL_ANALYZE_AUDIO_4, 
  4  //ADC_CHANNEL_ANALYZE_AUDIO_5,
   //1najíždí -2-3-4-stabilizace signálu 5 měření konec ,následuje další krok.
};

  
void Adc::Init() {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE); // nema tu být :DAC1 ?? JE TO NA PA4 ... 

  DMA_InitTypeDef dma_init;
  ADC_CommonInitTypeDef adc_common_init;
  ADC_InitTypeDef adc_init;
  GPIO_InitTypeDef gpio_init;
  
  //gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  //gpio_init.GPIO_Pin |= GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
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
  dma_init.DMA_Channel = DMA_Channel_0; //asi BRIGTNESS_CV PIN 14 neznamena ! , nastav se bna adc1 okruh na multiplex 
  dma_init.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //čeká na dokončení prvního kanálu asi / 1*
  dma_init.DMA_Memory0BaseAddr = (uint32_t)&values_[0]; 
  dma_init.DMA_DIR = DMA_DIR_PeripheralToMemory;
  dma_init.DMA_BufferSize = ADC_CHANNEL_NUM_DIRECT; 
  dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
  dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable; 
  dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
  dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
  dma_init.DMA_Mode = DMA_Mode_Circular; 
  dma_init.DMA_Priority = DMA_Priority_VeryHigh;
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

  ADC->CCR |= ADC_CCR_TSVREFE;    // Hardwarové zapnutí VREF a teplotního čidla

  
  adc_init.ADC_Resolution = ADC_Resolution_12b;
  adc_init.ADC_ScanConvMode = ENABLE;  //  1* nezustan na jedné adrese proměřuj podle seznamu zadaní :
  adc_init.ADC_ContinuousConvMode = DISABLE;
  adc_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  adc_init.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  adc_init.ADC_DataAlign = ADC_DataAlign_Left;
  // SPRÁVNĚ (přesně odpovídá 8 nakonfigurovaným kanálům ADC1):
  adc_init.ADC_NbrOfConversion = ADC_CHANNEL_NUM_DIRECT;       // tady je tet osum nakonfigurovaných kanalu pro loop""

  ADC_Init(ADC1, &adc_init);
  //multiplex 
  adc_init.ADC_Resolution = ADC_Resolution_12b;
  adc_init.ADC_ScanConvMode = DISABLE; //netočí vnitřní křemík ,jetam jen jeden adc ! 
  adc_init.ADC_ContinuousConvMode = DISABLE;
  adc_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  adc_init.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  adc_init.ADC_DataAlign = ADC_DataAlign_Left;
  adc_init.ADC_NbrOfConversion = 1;
  ADC_Init(ADC2, &adc_init);
  //(potřebuje to nejkaký čas pro naměření všech parametru .. to ma ok )


  // 168M / 2 / 8 / (9 x (480 + 5) + 0 x (144 + 5)) = 2.4kHz.
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles);// PA4 4...1 zachovano pro osuk kanalu na muliplex, ale co zmenit :  adc_init.ADC_NbrOfConversion = ADC_CHANNEL_NUM_DIRECT; a nebo to vyplnit ničim tt tam je vypl virtual jack , jinak je to už jinde použité jako dac -PA4 pro jinou funkci !
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_480Cycles); //Rank 2 (values_[1]) PA6 -- IN22 --- ADC_CHANNEL_CV_STRUCTURE (gemetry_cv)
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_480Cycles); //Rank 3 (values_[2]) PA0 --PIN14 --- ADC_CHANNEL_CV_BRIGHTNESS
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_480Cycles); //Rank 4 (values_[3]) PA5 --- PIN21 ---ADC_CHANNEL_CV_DAMPING
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 5, ADC_SampleTime_480Cycles); // Rank 5 (values_[4]) --- 23PIN = cv_Posittion PA7
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 6, ADC_SampleTime_480Cycles); // Rank 6 (values_[5]) --- PIN15 = cv_V_oct  PA1
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 7, ADC_SampleTime_480Cycles); // Rank 7 (values_(6)) --- PC3 PIN11 = ADC_CHANNEL_POT_FREQUENCY,
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 8, ADC_SampleTime_480Cycles); //Rank 8 (values_[7]) --- PC2 PIN10 = ADC_CHANNEL_POT_STRUCTURE nebo "GEOMETRY_pot""

  ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_480Cycles); // Rank 1 (valus_1)  ADC2 --- MUXED_POTS (pro mltiplex !! (tam je i analiza audio offst DC))
  
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
  
  index_ = ADC_CHANNEL_NUM_MUXED - 1;
  last_read_ = 0;
  state_ = false;
  // ==========================================================================
  // INICIALIZACE SVOBODNÉHO PINU PA4 JAKO HARDWAROVÉHO DAC KANÁLU 1
  // ==========================================================================
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE); // Zapnutí hodin pro DAC

  gpio_init.GPIO_Pin = GPIO_Pin_4;      ///nema to jit vejš ?? 
  gpio_init.GPIO_Mode = GPIO_Mode_AN;      // Analogový mód výstupu
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;  // Bez rezistorů
  GPIO_Init(GPIOA, &gpio_init);

  DAC_InitTypeDef dac_init;
  dac_init.DAC_Trigger = DAC_Trigger_None;               // Změna napětí nastane ihned po zápisu
  dac_init.DAC_WaveGeneration = DAC_WaveGeneration_None; // Bez interních generátorů vln
  dac_init.DAC_OutputBuffer = DAC_OutputBuffer_Enable;   // Zapnutý buffer pro silný a stabilní výstup
  
  DAC_Init(DAC_Channel_1, &dac_init);
  DAC_Cmd(DAC_Channel_1, ENABLE);

  ADC_SoftwareStartConv(ADC1); 

  Convert();
}

void Adc::DeInit() {
  DMA_Cmd(DMA2_Stream0, DISABLE);
  DAC_Cmd(DAC_Channel_1, DISABLE);
  ADC_DMARequestAfterLastTransferCmd(ADC1, DISABLE);
  ADC_Cmd(ADC1, DISABLE);
  ADC_Cmd(ADC2, DISABLE);
  ADC_DMACmd(ADC1, DISABLE);
  ADC_DeInit();
}

void Adc::Convert() {

  ADC_SoftwareStartConv(ADC1);
  
  if (state_) {
    // 2. BEZPEČNÉ NAČTENÍ HODNOTY Z MINULÉHO CYKLU ADC2
    int32_t target_channel = ADC_CHANNEL_POT_BRIGHTNESS + index_;
    // STRIGTNÍ INDEXOVÝ ZÁMEK: Hodnota z multiplexeru se zapíše POUZE do poťáků!
    // NIKDY nesmí přetéct a přepsat kanály ADC_CHANNEL_ANALYZE_AUDIO_1 až 5.
    if (target_channel < ADC_CHANNEL_LAST) {
      values_[target_channel] = ADC2->DR; // Zde teče čistý 16-bit (Left-Aligned)
    }
    
    last_read_ = index_;
    ++index_;
    if (index_ >= ADC_CHANNEL_NUM_MUXED) {
      index_ = 0;
    }
    /*
    // Read the value from the previous conversion.
    values_[ADC_CHANNEL_POT_BRIGHTNESS + index_] = ADC2->DR;
    last_read_ = index_;
    ++index_;
    if (index_ >= ADC_CHANNEL_NUM_MUXED) {
      index_ = 0;
    }
  */
    uint8_t address = addresses_[index_];
  
    // Write the mux address.
    GPIO_WriteBit(GPIOB, GPIO_Pin_7, static_cast<BitAction>(address & 1));
    GPIO_WriteBit(GPIOB, GPIO_Pin_6, static_cast<BitAction>(address & 2));
    GPIO_WriteBit(GPIOB, GPIO_Pin_5, static_cast<BitAction>(address & 4));

      // Krátká hardwarová pauza (Settle time) pro náboj kondenzátoru po přepnutí adresy
    __NOP(); __NOP(); __NOP(); __NOP();
  } else {
    ADC_SoftwareStartConv(ADC2);
  }
  
  state_ = !state_;
}


}  // namespace rings
