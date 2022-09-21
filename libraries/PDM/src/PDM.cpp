/*
  PDM.cpp - library to interface with nRF52840 PDM peripheral
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2019 Arduino SA

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#include "PDM.h"
#include <hal/nrf_pdm.h>

#define DEFAULT_PDM_GAIN     80
#define PDM_IRQ_PRIORITY     7

//because nordic clk divider is not continuous, so there is some error range between user's and provided
#define NRF_PDM_FREQ_62_5K  (nrf_pdm_freq_t)(0x00800000UL)               ///< PDM_CLK= 62.5  KHz (32 MHz / 512) 
#define NRF_PDM_FREQ_80K    (nrf_pdm_freq_t)(0x00A3A000UL)               ///< PDM_CLK= 80    KHz (32 MHz / 400)
#define NRF_PDM_FREQ_125K   (nrf_pdm_freq_t)(0x01000000UL)               ///< PDM_CLK= 125   KHz (32 MHz / 256)
#define NRF_PDM_FREQ_160K   (nrf_pdm_freq_t)(0x01474000UL)               ///< PDM_CLK= 160   KHz (32 MHz / 200)
#define NRF_PDM_FREQ_250K   (nrf_pdm_freq_t)(0x02000000UL)               ///< PDM_CLK= 250   KHz (32 MHz / 128)
#define NRF_PDM_FREQ_320K   (nrf_pdm_freq_t)(0x028E8000UL)               ///< PDM_CLK= 320   KHz (32 MHz / 100)
#define NRF_PDM_FREQ_500K   (nrf_pdm_freq_t)(0x04000000UL)               ///< PDM_CLK= 500   KHz (32 MHz / 64) 
#define NRF_PDM_FREQ_640K   (nrf_pdm_freq_t)(0x051D0000UL)               ///< PDM_CLK= 640   KHz (32 MHz / 50) 
#define NRF_PDM_FREQ_1000K  (nrf_pdm_freq_t)(0x08000000UL)               ///< PDM_CLK= 1.000 MHz (32 MHz / 32) 
#define NRF_PDM_FREQ_1032K  (nrf_pdm_freq_t)(0x08400000UL)               ///< PDM_CLK= 1.032 MHz (32 MHz / 31) 
#define NRF_PDM_FREQ_1067K  (nrf_pdm_freq_t)(0x08800000UL)               ///< PDM_CLK= 1.067 MHz (32 MHz / 30) 
#define NRF_PDM_FREQ_1231K  (nrf_pdm_freq_t)(0x09800000UL)               ///< PDM_CLK= 1.231 MHz (32 MHz / 26) 
#define NRF_PDM_FREQ_1280K  (nrf_pdm_freq_t)(0x0A000000UL)               ///< PDM_CLK= 1.280 MHz (32 MHz / 25) 
#define NRF_PDM_FREQ_1785K  (nrf_pdm_freq_t)(0x0D8A0000UL)               ///< PDM_CLK= 1.785 MHz (32 MHz / 18)
#define NRF_PDM_FREQ_1785K  (nrf_pdm_freq_t)(0x0D8A0000UL)               ///< PDM_CLK= 1.785 MHz (32 MHz / 18)
#define NRF_PDM_FREQ_2667K  (nrf_pdm_freq_t)(0x15000000UL)               ///< PDM_CLK= 2.667 MHz (32 MHz / 12) 
#define NRF_PDM_FREQ_3200K  (nrf_pdm_freq_t)(0x19000000UL)               ///< PDM_CLK= 3.200 MHz (32 MHz / 10) 
#define NRF_PDM_FREQ_3571K  (nrf_pdm_freq_t)(0x1B140000UL)               ///< PDM_CLK= 3.571 MHz (32 MHz / 9 ) 
#define NRF_PDM_FREQ_4000K  (nrf_pdm_freq_t)(0x20000000UL)               ///< PDM_CLK= 4.000 MHz (32 MHz /  8) 
#define NRF_PDM_FREQ_8000K  (nrf_pdm_freq_t)(0x40000000UL)               ///< PDM_CLK= 8.000 MHz (32 MHz /  4) 
#define NRF_PDM_FREQ_10666K (nrf_pdm_freq_t)(0x54000000UL)               ///< PDM_CLK= 10.666 MHz (32 MHz /  3) 
#define NRF_PDM_FREQ_16000K (nrf_pdm_freq_t)(0x80000000UL)               ///< PDM_CLK= 16.000 MHz (32 MHz /  2) 


#define RAW_BUFFER_SIZE 256 
int16_t rawBuffer0[RAW_BUFFER_SIZE];
int16_t rawBuffer1[RAW_BUFFER_SIZE];
int16_t* rawBuffer[2] = {rawBuffer0, rawBuffer1};
volatile int rawBufferIndex = 0; 

PDMClass::PDMClass(int dinPin, int clkPin, int pwrPin) :
  _dinPin(dinPin),
  _clkPin(clkPin),
  _pwrPin(pwrPin),
  _onReceive(NULL)
{
}

PDMClass::~PDMClass()
{
}

void PDMClass::setPins(int dataPin, int clkPin, int pwrPin)
{
  _dinPin = dataPin;
  _clkPin = clkPin;
  _pwrPin = pwrPin;
}

int PDMClass::begin(int channels, PCM_FREQUENCE sampleRate)
{
  _channels = channels;

  // Enable high frequency oscillator if not already enabled
  if (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) { }
  }
  NRF_PDM->RATIO = ((PDM_RATIO_RATIO_Ratio80 << PDM_RATIO_RATIO_Pos) & PDM_RATIO_RATIO_Msk);
  // configure the sample rate and channels
  switch (sampleRate) {
    case PCM_1000:
      nrf_pdm_clock_set(NRF_PDM, NRF_PDM_FREQ_80K);
      break;
    case PCM_2000:
      nrf_pdm_clock_set(NRF_PDM, NRF_PDM_FREQ_160K);
      break;
    case PCM_4000:
      nrf_pdm_clock_set(NRF_PDM, NRF_PDM_FREQ_320K);
      break;
    case PCM_8000:
      nrf_pdm_clock_set(NRF_PDM, NRF_PDM_FREQ_640K);
      break;
    case PCM_16000:
      nrf_pdm_clock_set(NRF_PDM, NRF_PDM_FREQ_1280K);
      break;
    case PCM_22050:      //22.31K, error is 1.1%
      nrf_pdm_clock_set(NRF_PDM, NRF_PDM_FREQ_1785K);
      break;
    case PCM_44100:      //44.64K, error is 2.2%
      nrf_pdm_clock_set(NRF_PDM, NRF_PDM_FREQ_3571K);
      break;
    case PCM_48000:      //50K, error is 4.1%
      nrf_pdm_clock_set(NRF_PDM, NRF_PDM_FREQ_4000K);
      break;
    case PCM_100000:      //100K
      nrf_pdm_clock_set(NRF_PDM, NRF_PDM_FREQ_8000K);
      break;
    case PCM_200000:      //200K
      nrf_pdm_clock_set(NRF_PDM, NRF_PDM_FREQ_16000K);
      break;
    default:
      return 0; // unsupported
  }

  switch (channels) {
    case 2:
      nrf_pdm_mode_set(NRF_PDM, NRF_PDM_MODE_STEREO, NRF_PDM_EDGE_LEFTRISING);
      break;

    case 1:
      nrf_pdm_mode_set(NRF_PDM, NRF_PDM_MODE_MONO, NRF_PDM_EDGE_LEFTFALLING);
      break;

    default:
      return 0; // unsupported
  }

  setGain(DEFAULT_PDM_GAIN);  

  // configure the I/O and mux
  pinMode(_clkPin, OUTPUT);
  digitalWrite(_clkPin, LOW);

  pinMode(_dinPin, INPUT);

  nrf_pdm_psel_connect(NRF_PDM, digitalPinToPinName(_clkPin), digitalPinToPinName(_dinPin));

  // clear events and enable PDM interrupts
  nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_STARTED);
  nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_END);
  nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_STOPPED);
  nrf_pdm_int_enable(NRF_PDM, NRF_PDM_INT_STARTED | NRF_PDM_INT_STOPPED | NRF_PDM_INT_END);

  if (_pwrPin > -1) {
    // power the mic on
    pinMode(_pwrPin, OUTPUT);
    digitalWrite(_pwrPin, HIGH);
  }

  // clear the buffer
  _doubleBuffer.reset();

  // set the PDM IRQ priority and enable
  NVIC_SetPriority(PDM_IRQn, PDM_IRQ_PRIORITY);
  NVIC_ClearPendingIRQ(PDM_IRQn);
  NVIC_EnableIRQ(PDM_IRQn);

  // set the buffer for transfer
  //nrf_pdm_buffer_set(NRF_PDM, (uint32_t*)_doubleBuffer.data(), _doubleBuffer.availableForWrite() / (sizeof(int16_t) * _channels));
  //_doubleBuffer.swap(0);
  nrf_pdm_buffer_set(NRF_PDM, (uint32_t*)rawBuffer[rawBufferIndex], RAW_BUFFER_SIZE);


  // enable and trigger start task
  nrf_pdm_enable(NRF_PDM);
  nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_STARTED);
  nrf_pdm_task_trigger(NRF_PDM, NRF_PDM_TASK_START);

  return 1;
}

void PDMClass::end()
{
  // disable PDM and IRQ
  nrf_pdm_disable(NRF_PDM);

  NVIC_DisableIRQ(PDM_IRQn);

  if (_pwrPin > -1) {
    // power the mic off
    digitalWrite(_pwrPin, LOW);
    pinMode(_pwrPin, INPUT);
  }

  // Don't disable high frequency oscillator since it could be in use by RADIO

  // unconfigure the I/O and un-mux
  nrf_pdm_psel_disconnect(NRF_PDM);

  pinMode(_clkPin, INPUT);
}

int PDMClass::available()
{
  NVIC_DisableIRQ(PDM_IRQn);

  //size_t avail = _doubleBuffer.available();

  NVIC_EnableIRQ(PDM_IRQn);

  return 0;
}

int PDMClass::read(void* buffer, size_t size)
{
  static int cutSamples = 100;
  NVIC_DisableIRQ(PDM_IRQn);

  if (cutSamples) {
    memset((uint8_t*)rawBuffer[rawBufferIndex], 0, cutSamples);
    cutSamples = 0;
  }
  memcpy(buffer, rawBuffer[rawBufferIndex], RAW_BUFFER_SIZE*2);

  NVIC_EnableIRQ(PDM_IRQn);

  return 0;
}

void PDMClass::onReceive(void(*function)(void))
{
  _onReceive = function;
}

void PDMClass::setGain(int gain)
{
  nrf_pdm_gain_set(NRF_PDM, gain, gain);
}

void PDMClass::setBufferSize(int bufferSize)
{
  //_doubleBuffer.setSize(bufferSize);
}

void PDMClass::IrqHandler()
{
  if (nrf_pdm_event_check(NRF_PDM, NRF_PDM_EVENT_STARTED)) {
    nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_STARTED);
    
    //Swap buffer.
    if (nrf_pdm_buffer_get(NRF_PDM) == (uint32_t*) rawBuffer[0]) {
        nrf_pdm_buffer_set(NRF_PDM, (uint32_t*)rawBuffer[1], RAW_BUFFER_SIZE);
    } else {
        nrf_pdm_buffer_set(NRF_PDM, (uint32_t*)rawBuffer[0], RAW_BUFFER_SIZE);
    }

    // if (_doubleBuffer.available() == 0) {
    //   // switch to the next buffer
    //   nrf_pdm_buffer_set(NRF_PDM, (uint32_t*)_doubleBuffer.data(), _doubleBuffer.availableForWrite() / (sizeof(int16_t) * _channels));

    //   // make the current one available for reading
    //   _doubleBuffer.swap(_doubleBuffer.availableForWrite());

    //   if (cutSamples) {
    //      memset((uint8_t*)_doubleBuffer.data(), 0, cutSamples);
    //      cutSamples = 0;
    //   }
    //   // call receive callback if provided
    //   if (_onReceive) {
    //     _onReceive();
    //   }
    // } else {
    //   // buffer overflow, stop
    //   nrf_pdm_disable(NRF_PDM);
    // }
  } else if (nrf_pdm_event_check(NRF_PDM, NRF_PDM_EVENT_STOPPED)) {
    nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_STOPPED);
  } else if (nrf_pdm_event_check(NRF_PDM, NRF_PDM_EVENT_END)) {
    nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_END);
    //Buffer is ready to process.
    if (nrf_pdm_buffer_get(NRF_PDM) == (uint32_t*) rawBuffer[0]) {
      rawBufferIndex = 1;
    } else {
      rawBufferIndex = 0;
    }
    // call receive callback if provided
    if (_onReceive) {
      _onReceive();
    }
  }
}

extern "C"
{
  void PDM_IRQHandler(void)
  {
    PDM.IrqHandler();
  }
}

#ifndef PIN_PDM_DIN
#define PIN_PDM_DIN -1
#endif

#ifndef PIN_PDM_CLK
#define PIN_PDM_CLK -1
#endif

#ifndef PIN_PDM_PWR
#define PIN_PDM_PWR -1
#endif

PDMClass PDM(PIN_PDM_DIN, PIN_PDM_CLK, PIN_PDM_PWR);
