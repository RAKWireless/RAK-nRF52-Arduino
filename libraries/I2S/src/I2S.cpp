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
    if(p_released->p_rx_buffer!=NULL)
    {               
       if (_onRxIRQ) 
       { 
        _onRxIRQ();  
         _readBuffer.swap();
         NRF_I2S->RXD.PTR = (uint32_t)_readBuffer.data();  
        
      }           
    }
     
    if(p_released->p_tx_buffer!=NULL)
    {            
      if (_onTxIRQ)
       { 
        _onTxIRQ();   
        _writeBuffer.swap();  
        NRF_I2S->TXD.PTR = (uint32_t)_writeBuffer.data();     
       }  
       
    }   
}
extern "C"
{  
static void data_handler(nrfx_i2s_buffers_t const * p_released,uint32_t  status)
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
void I2SClass::setPin( uint8_t sck_pin,uint8_t lrck_pin,uint8_t mck_pin,
             uint8_t sdout_pin,uint8_t sdin_pin)
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
 switch(width)
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
  if(frequency>=48000)
  {
//    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV21; ///< 32 MHz / 2 = 16.0 MHz.
//    _i2s_config.ratio = NRF_I2S_RATIO_32X;  //LRCK = MCK / 32.
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV11; ///< 32 MHz / 11 = 16.0 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_64X;  //LRCK = MCK / 64.
  }
  else if(frequency>=47250)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV21; ///< 32 MHz / 21 = 16.0 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_32X;  //LRCK = MCK / 32.
  }
    else if(frequency>=44100)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV11; ///< 32 MHz / 11 = 16.0 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_64X;  //LRCK = MCK / 64.
  }
  else if(frequency>=38400)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV23; ///< 32 MHz / 31 = 16.0 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_32X;  //LRCK = MCK / 32.
  }
  else if(frequency>=32000)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV31; ///< 32 MHz / 31 = 16.0 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_32X;  //LRCK = MCK / 32.
  }
  else if(frequency>=22050)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV15; ///< 32 MHz / 15 =  MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_96X;  //LRCK = MCK / 96.
  }
  else if(frequency>=1600)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV31; ///< 32 MHz / 331 =  MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_64X;  //LRCK = MCK / 64.
  }
    else if(frequency>=11025)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV21; ///< 32 MHz / 2 = 16.0 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_96X;  //LRCK = MCK / 32.
  }
  else if(frequency>=8000)
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV21; ///< 32 MHz / 2 = 16.0 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_96X;  //LRCK = MCK / 32.
  }
  else
  {
    _i2s_config.mck_setup = NRF_I2S_MCK_32MDIV21; ///< 32 MHz / 2 = 16.0 MHz.
    _i2s_config.ratio = NRF_I2S_RATIO_96X;  //LRCK = MCK / 32.
  }
  return true;
}
uint32_t I2SClass::begin(i2s_mode_t mode,i2s_channels_t channel,uint8_t sampleWidth,uint32_t fs)
{
   _i2s_config.irq_priority = I2S_IRQ_PRIORITY;
   setMode(mode);
//   setPin(I2S_SCK_PIN,I2S_LRCK_PIN,I2S_MCK_PIN,I2S_SDOUT_PIN,I2S_SDIN_PIN);
   setFrameFormat(FORMAT_I2S);
   setAlignment(ALIGN_Left);
   setSampleWidth(sampleWidth);
   setChannels(channel);
   setFs(fs);
   _readBuffer.reset();
   _writeBuffer.reset();
   if(_readBuffer.size()>_writeBuffer.size())
   {
    _bufferlength = _readBuffer.size();
   }
   else
   {
    _bufferlength = _writeBuffer.size();
   }  
   return(nrfx_i2s_init(&_i2s_config, data_handler));
}
uint32_t I2SClass::begin(i2s_mode_t mode)
{
   _i2s_config.irq_priority = I2S_IRQ_PRIORITY;
   setMode(mode);
   setPin(I2S_SCK_PIN,I2S_LRCK_PIN,I2S_MCK_PIN,I2S_SDOUT_PIN,I2S_SDIN_PIN);
//   setFrameFormat(FORMAT_I2S);

   _readBuffer.reset();
   _writeBuffer.reset();

   if(_readBuffer.size()>_writeBuffer.size())
   {
    _bufferlength = _readBuffer.size();
   }
   else
   {
    _bufferlength = _writeBuffer.size();
   }   
   
   return(nrfx_i2s_init(&_i2s_config, data_handler));
}
uint32_t I2SClass::begin(i2s_channels_t channel,uint32_t fs,uint8_t sampleWidth)
{
   _i2s_config.irq_priority = I2S_IRQ_PRIORITY;
   setMode(I2S_Master);
   setPin(I2S_SCK_PIN,I2S_LRCK_PIN,I2S_MCK_PIN,I2S_SDOUT_PIN,I2S_SDIN_PIN);
   setFrameFormat(FORMAT_I2S);
   setAlignment(ALIGN_Left);
   setSampleWidth(sampleWidth);
   setChannels(channel);
   setFs(fs);
   _readBuffer.reset();
   _writeBuffer.reset();
   if(_readBuffer.size()>_writeBuffer.size())
   {
    _bufferlength = _readBuffer.size();
   }
   else
   {
    _bufferlength = _writeBuffer.size();
   } 
   return(nrfx_i2s_init(&_i2s_config, data_handler));
}
void I2SClass::start(void)
{ 
  i2s_buffers_t const initial_buffers = {
              .p_rx_buffer = (uint32_t*)_readBuffer.data(),
              .p_tx_buffer = (uint32_t*)_writeBuffer.data()
            };  
  nrfx_i2s_start(&initial_buffers,_bufferlength,0);
}

void I2SClass::stop(void)
{
  nrfx_i2s_stop();
}
void I2SClass::end()
{
  // _readBuffer.end();
  // _writeBuffer.end();
  nrfx_i2s_uninit();
}

int I2SClass::read(void* buffer, size_t size)
{
  NVIC_DisableIRQ(I2S_IRQn);  
  int readByte = _readBuffer.size();  
  _readBuffer.read(buffer,size);
  NVIC_EnableIRQ(I2S_IRQn);

  return readByte;
}
int I2SClass::write(void* buffer, size_t size)
{
  NVIC_DisableIRQ(I2S_IRQn);
  int writeByte = _writeBuffer.size();
  _writeBuffer.write(buffer,size);
  NVIC_EnableIRQ(I2S_IRQn);
  return writeByte;
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
