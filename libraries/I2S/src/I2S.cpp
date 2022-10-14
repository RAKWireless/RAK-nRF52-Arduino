#include "I2S.h"
#include <wiring_private.h>
#include <assert.h>
#include <hal/nrf_i2s.h>
#include "nrf.h"
#include "nrfx_i2s.h"
#include "nrf_i2s.h"
#include "I2SDoubleBuffer.h"

#define I2S_IRQ_PRIORITY     7

I2SClass::I2SClass():
  _onTxIRQ(NULL),
  _onRxIRQ(NULL)
{
  //this->config = I2S_DEFAULT_CONFIG;
}

I2SClass::~I2SClass()
{

}

void I2SClass::irqHander(nrfx_i2s_buffers_t const * p_released)
{
  if (p_released->p_rx_buffer != NULL)
  {
    if (_onRxIRQ)
    {
      // NRF_I2S->EVENTS_RXPTRUPD = 0;
      _readBuffer.swap();
      NRF_I2S->RXD.PTR = (uint32_t)_readBuffer.data();
      _onRxIRQ();
    }

  }

  if (p_released->p_tx_buffer != NULL)
  {
    if (_onTxIRQ)
    {
      // NRF_I2S->EVENTS_TXPTRUPD = 0;
      _writeBuffer.swap();
      NRF_I2S->TXD.PTR = (uint32_t)_writeBuffer.data();
      _onTxIRQ();
    }

  }
}
extern "C"
{
  static void data_handler(nrfx_i2s_buffers_t const * p_released, uint32_t  status)
  {
    if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
    {
      return;
    }
    I2S.irqHander(p_released);
  }

}
uint8_t I2SClass::setMode(i2s_mode_t i2s_mode)
{
  _i2s_config.mode = (nrf_i2s_mode_t)i2s_mode;
  return _i2s_config.mode;
}
void I2SClass::setPin( uint8_t sck_pin, uint8_t lrck_pin, uint8_t mck_pin,
                       uint8_t sdout_pin, uint8_t sdin_pin)
{
  _i2s_config.sck_pin = sck_pin;
  _i2s_config.lrck_pin = lrck_pin;
  _i2s_config.mck_pin = mck_pin;
  _i2s_config.sdout_pin = sdout_pin;
  _i2s_config.sdin_pin = sdin_pin;

}
uint8_t I2SClass::setFrameFormat(i2s_format_t format)
{
  _i2s_config.format = (nrf_i2s_format_t)format;
  return _i2s_config.format;
}
uint8_t I2SClass::setAlignment(i2s_align_t align)
{
  _i2s_config.alignment = (nrf_i2s_align_t)align;
  return _i2s_config.alignment;
}
uint8_t I2SClass::setSampleWidth(uint8_t width)
{
  _sampleWidth = width;
  switch (width)
  {
    case 8:
      _i2s_config.sample_width = (nrf_i2s_swidth_t)NRF_I2S_SWIDTH_8BIT;
      break;
    case 16:
      _i2s_config.sample_width = (nrf_i2s_swidth_t)NRF_I2S_SWIDTH_16BIT;
      break;
    case 24:
      _i2s_config.sample_width = (nrf_i2s_swidth_t)NRF_I2S_SWIDTH_24BIT;
      break;
    default:
      _i2s_config.sample_width = (nrf_i2s_swidth_t)NRF_I2S_SWIDTH_16BIT;
      break;
  }
  return _i2s_config.sample_width;
}
uint8_t I2SClass::setChannels(i2s_channels_t channel)
{
  _i2s_config.channels = (nrf_i2s_channels_t)channel;
  return _i2s_config.channels;
}

bool I2SClass::setFs(uint32_t frequency)
{
  //8,000 Hz - the sampling rate used by the phone, enough for human speech
  //11,025 Hz - the sample rate used for AM radio
  //22,050 Hz and 24,000 Hz - sample rates used by FM radio
  //32,000 Hz - sample rate used by miniDV digital video camcorder, DAT (LP mode)
  //44,100 Hz - Audio CD, also commonly used sample rate used by MPEG-1 audio (VCD, SVCD, MP3)
  //47,250 Hz - sample rate used by commercial PCM recorders
  //48,000 Hz - sample rate for digital sound used by miniDV, digital TV, DVD, DAT, movies and professional audio

  //_i2s_config.mck_setup = NRF_I2S_MCK_32MDIV8; ///< 32 MHz / 11 = 16.0 MHz.
  //_i2s_config.ratio = NRF_I2S_RATIO_32X;  //LRCK = MCK / 64.

  //MCK = 32MHz/mck_setup
  //LCK = MCK/ratio
  if (frequency >= 125000)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV8; ///< 32 MHz / 8 =  MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_32X;  //LRCK = MCK / 32
  }
  else if (frequency >= 96000)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV10; ///< 32 MHz / 32 = MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_32X;  //LRCK = MCK / 32.
  }
  else if (frequency >= 48000)
  {
    //   _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV21; ///< 32 MHz / 10 = 3.2 MHz.
    // _i2s_config.ratio = NRF_I2S_RATIO_32X;  //LRCK = MCK / 64.

    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV10; ///< 32 MHz / 10 = 3.2 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_64X;  //LRCK = MCK / 64.
  }
  else if (frequency >= 44100)
  {
    //    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV23; ///< 32 MHz / 11 =  MHz.
    // _i2s_config.ratio = NRF_I2S_RATIO_32X;  //LRCK = MCK / 64.
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV15; ///< 32 MHz / 15 =  MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_48X;  //LRCK = MCK / 64.
  }
  else if (frequency >= 38400)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV23; ///< 32 MHz / 23 =  MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_32X;  //LRCK = MCK / 32.
  }
  else if (frequency >= 32000)
  {
    //  _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV21; ///< 32 MHz / 31 =  MHz.
    // _i2s_config.ratio = NRF_I2S_RATIO_48X;  //LRCK = MCK / 32.

    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV31; ///< 32 MHz / 31 =  MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_32X;  //LRCK = MCK / 32.
  }
  else if (frequency >= 22050)
  { 
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV15; ///< 32 MHz / 15 =  MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_96X;  //LRCK = MCK / 96.
  }
  else if (frequency >= 16000)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV31; ///< 32 MHz / 63 =  MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_64X;  //LRCK = MCK / 32.
  }
  else if (frequency >= 11025)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV30; ///< 32 MHz / 30 = 16.0 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_96X;  //LRCK = MCK / 96.
  }
  else if (frequency >= 8000)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV125; ///< 32 MHz / 21 = 16.0 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_32X;  //LRCK = MCK / 96.
  }
  else if (frequency >= 5000)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV125; ///< 32 MHz / 21 = 16.0 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_48X;  //LRCK = MCK / 96.
  }
  else if (frequency >= 4000)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV125; ///< 32 MHz / 21 = 16.0 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_64X;  //LRCK = MCK / 96.
  }
  else
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV125; ///< 32 MHz / 32 = 1MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_256X;  //LRCK = MCK / 256
  }
  return true;
}

void I2SClass::setSampleRate(int freq)
{
  int64_t sys = 32000000;
  int fs = freq;
  uint8_t px = 0, py = 0;
  uint8_t pi = 0, pj = 0;
  int temp = 0;
  int delt = 0;
  int min = sys;
  int64_t max = -32000000;
  int div[13] = {8, 10, 11, 15, 16, 21, 23, 30, 31, 32, 42, 63, 125};
  int ratio[9] = {32, 48, 64, 96, 128, 192, 256, 384, 512};
  int value1[9][13] = {{0}};
  for (int i = 0; i < 9; i++)
  {
    for (int j = 0; j < 13; j++)
    {
      value1[i][j] = ratio[i] * div[j];
      temp = fs * value1[i][j];
      delt = sys - temp;
      if (delt >= 0)
      {
        if (delt < min)
        {
          int temp = _sampleWidth *2; 
          float ftemp = ((float)ratio[i])/temp;
          int integer = (int)(ftemp*1000);
          // Serial.printf("temp:%d  ftemp:%3.3f  integer:%d\r\n",temp,ftemp,integer);  
          if((integer%1000 == 0) && (ratio[i] >= temp))
          {
            px = i;
            py = j;
            min = delt;
          }
          
        }
      }
      else
      {
        if (delt > max)
        {
          pi = i;
          pj = j;
          max = delt;
        }
      }
      //      Serial.printf("%d\t", value1[i][j]);
    }
    //    Serial.println();
  }
  // Serial.printf("px:%d,py:%d  min:%d\r\n", px, py, min);
  uint64_t n = sys;
  temp = div[py] * ratio[px];
  temp = (int) n / temp;
  // Serial.printf("ratio:%d,div:%d  value:%d  fs:%d\r\n", ratio[px], div[py], (ratio[px]*div[py]*fs), temp);

  //  Serial.printf("pi:%d,pj:%d  min:%d\r\n", pi, pj, max);
  //  temp = div[pi] * ratio[pj];
  //  temp = (int) n / temp;
  //  Serial.printf("ratio:%d,div:%d  value:%d  fs:%d\r\n", ratio[pi], div[pj], (ratio[pi]*div[pj]*fs), temp);

  switch (div[py])
  {
    case 8:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV8;
      break;
    case 10:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV10;
      break;
    case 11:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV11;
      break;
    case 15:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV15;
      break;
    case 16:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV16;
      break;
    case 21:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV21;
      break;
    case 23:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV23;
      break;
    case 30:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV30;
      break;
    case 31:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV31;
      break;
    case 32:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV32;
      break;
    case 42:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV42;
      break;
    case 63:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV63;
      break;
    case 125:
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV125;
      break;
    default :
      _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV125;
      break;
  }

  switch (ratio[px])
  {
    case 32:
      _i2s_config.ratio = NRF_I2S_RATIO_32X;
      break;
    case 48:
      _i2s_config.ratio = NRF_I2S_RATIO_48X;
      break;
    case 64:
      _i2s_config.ratio = NRF_I2S_RATIO_64X;
      break;
    case 96:
      _i2s_config.ratio = NRF_I2S_RATIO_96X;
      break;
    case 128:
      _i2s_config.ratio = NRF_I2S_RATIO_128X;
      break;
    case 192:
      _i2s_config.ratio = NRF_I2S_RATIO_192X;
      break;
    case 256:
      _i2s_config.ratio = NRF_I2S_RATIO_256X;
      break;
    case 384:
      _i2s_config.ratio = NRF_I2S_RATIO_384X;
      break;
    case 512:
      _i2s_config.ratio = NRF_I2S_RATIO_512X;
      break;
    default :
      _i2s_config.ratio = NRF_I2S_RATIO_512X;
      break;
  }

}

uint32_t I2SClass::begin(i2s_mode_t mode, i2s_channels_t channel, uint8_t sampleWidth, uint32_t fs)
{
  _i2s_config.irq_priority = I2S_IRQ_PRIORITY;
  setMode(mode);
  setPin(I2S_SCK_PIN, I2S_LRCK_PIN, I2S_MCK_PIN, I2S_SDOUT_PIN, I2S_SDIN_PIN);
  setFrameFormat(FORMAT_I2S);
  setAlignment(ALIGN_Left);
  setSampleWidth(sampleWidth);
  setChannels(channel);
  setSampleRate(fs); //setFs(fs);
  //  _readBuffer.reset();
  //  _writeBuffer.reset();
  return (nrfx_i2s_init(&_i2s_config, data_handler));
}
uint32_t I2SClass::begin(i2s_mode_t mode)
{
  _i2s_config.irq_priority = I2S_IRQ_PRIORITY;
  setMode(mode);
  setPin(I2S_SCK_PIN, I2S_LRCK_PIN, I2S_MCK_PIN, I2S_SDOUT_PIN, I2S_SDIN_PIN);
  setFrameFormat(FORMAT_I2S);

  //  _readBuffer.reset();
  //  _writeBuffer.reset();
  return (nrfx_i2s_init(&_i2s_config, data_handler));
}
uint32_t I2SClass::begin(i2s_channels_t channel, uint32_t fs, uint8_t sampleWidth)
{
  _i2s_config.irq_priority = I2S_IRQ_PRIORITY;
  setMode(I2S_Master);
  setPin(I2S_SCK_PIN, I2S_LRCK_PIN, I2S_MCK_PIN, I2S_SDOUT_PIN, I2S_SDIN_PIN);
  setFrameFormat(FORMAT_I2S );  //FORMAT_ALIGNED  FORMAT_I2S
  setAlignment(ALIGN_Left);  //ALIGN_Right ALIGN_Left
  setSampleWidth(sampleWidth);
  setChannels(channel);
  // setFs(fs);
  setSampleRate(fs);
  //  _readBuffer.reset();
  //  _writeBuffer.reset();
  return (nrfx_i2s_init(&_i2s_config, data_handler));
}
void I2SClass::setSize(int size)
{
  _readBuffer.setSize(size);
  _writeBuffer.setSize(size);
}
void I2SClass::start(void)
{
  // int DMAsize = _writeBuffer.size()/sizeof(uint32_t);
  int DMAsize = _writeBuffer.size();
  _readBuffer.reset();
  _writeBuffer.reset();
  i2s_buffers_t const initial_buffers = {
    .p_rx_buffer = (uint32_t*)_readBuffer.data(),
    .p_tx_buffer = (uint32_t*)_writeBuffer.data()
  };

  nrfx_i2s_start(&initial_buffers, DMAsize, 0);
}

void I2SClass::stop(void)
{
  nrfx_i2s_stop();
}
void I2SClass::end()
{
  nrfx_i2s_uninit();
}

int I2SClass::read(void* buffer, size_t size)
{
  NVIC_DisableIRQ(I2S_IRQn);
  int readByte = _readBuffer.size();
  _readBuffer.read(buffer, size);
  NVIC_EnableIRQ(I2S_IRQn);

  return readByte;
}
int I2SClass::write(void* buffer, size_t size)
{
  NVIC_DisableIRQ(I2S_IRQn);
  _writeBuffer.write(buffer, size);
  NVIC_EnableIRQ(I2S_IRQn);
  return size;
}
void I2SClass::TxIRQCallBack(void(*function)(void))
{
  _onTxIRQ = function;
}
void I2SClass::RxIRQCallBack(void(*function)(void))
{
  _onRxIRQ = function;
}


I2SClass I2S;
