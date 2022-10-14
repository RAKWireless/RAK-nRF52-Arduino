#ifndef I2S_H
#define I2S_H

#include "nrf.h"
#include "nrfx_i2s.h"
#include "nrf_i2s.h"
#include "I2SDoubleBuffer.h"

#include <stdio.h>
#include "Arduino.h"
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif


#define I2S_SCK_PIN   WB_IO6
#define I2S_LRCK_PIN  WB_IO5
#define I2S_MCK_PIN   -1
#define I2S_SDOUT_PIN WB_I2C2_SDA //24
#define I2S_SDIN_PIN  WB_I2C2_SCL //25


/**
 * @brief I2S modes of operation.
 */
typedef enum
{
    I2S_Master = NRF_I2S_MODE_MASTER, ///< Master mode.
    I2S_Slave = NRF_I2S_MODE_SLAVE   ///< Slave mode.
} i2s_mode_t;
/**
 * @brief I2S frame formats.
 */
typedef enum
{
    FORMAT_I2S = NRF_I2S_FORMAT_I2S,    ///< Original I2S format.
    FORMAT_ALIGNED = NRF_I2S_FORMAT_ALIGNED ///< Alternate (left- or right-aligned) format.
} i2s_format_t;
/**
 * @brief I2S enabled channels.
 */
typedef enum
{
    Stereo = NRF_I2S_CHANNELS_STEREO, ///< Stereo.
    Left = NRF_I2S_CHANNELS_LEFT,   ///< Left only.
    Right = NRF_I2S_CHANNELS_RIGHT   ///< Right only.
} i2s_channels_t;

typedef enum
{
    StereoMode = 2, ///< Stereo.
    MonoMode = 1   ///< Left only.   
} channels_t;
/**
 * @brief I2S alignments of sample within a frame.
 */
typedef enum
{
    ALIGN_Left = NRF_I2S_ALIGN_LEFT, ///< Left-aligned.
    ALIGN_Right = NRF_I2S_ALIGN_RIGHT ///< Right-aligned.
} i2s_align_t;

typedef nrfx_i2s_buffers_t i2s_buffers_t;

class I2SClass
{
public:
  I2SClass(void);
  virtual ~I2SClass();

  uint8_t setMode(i2s_mode_t i2s_mode);
  void setPin( uint8_t sck_pin,uint8_t lrck_pin,uint8_t mck_pin,
             uint8_t sdout_pin,uint8_t sdin_pin);
  uint8_t setFrameFormat(i2s_format_t format);
  uint8_t setAlignment(i2s_align_t align);
  uint8_t setSampleWidth(uint8_t width);
  uint8_t setChannels(i2s_channels_t channel);
  void setSampleRate(int freq);
  bool setFs(uint32_t frequency);
  uint32_t begin(i2s_mode_t mode,i2s_channels_t channel,uint8_t sampleWidth,uint32_t fs);
  uint32_t begin(i2s_mode_t mode);
  uint32_t begin(i2s_channels_t channel,uint32_t fs,uint8_t sampleWidth);
  void stop(void);
  void end();
  void start(void);
  void setSize(int size);
  int write(void* buffer, size_t size);
  int read(void* buffer, size_t size);
  void irqHander(nrfx_i2s_buffers_t const * p_released);
  void TxIRQCallBack(void(*function)(void));
  void RxIRQCallBack(void(*function)(void));

  private:
    int _sampleWidth = 16;
 
    nrfx_i2s_config_t _i2s_config;  
    I2SDoubleBuffer _readBuffer;
    I2SDoubleBuffer _writeBuffer;  
   
     void (*_onTxIRQ)(void);
     void (*_onRxIRQ)(void);
};

extern I2SClass I2S;

#endif
