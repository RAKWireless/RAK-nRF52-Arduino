#ifndef _I2SDOUBLEBUFFER_H
#define _I2SDOUBLEBUFFER_H

#include <stddef.h>
#include <stdint.h>

#define DEFAULT_I2S_BUFFER_SIZE 512

class I2SDoubleBuffer
{
public:
  I2SDoubleBuffer();
  virtual ~I2SDoubleBuffer();

  void setSize(int size);
  void reset();
  size_t write(const void *buffer, size_t size);
  size_t read(void *buffer, size_t size);
  void* data();
  void* data(uint8_t index);
  int swap(void);
  int size(void);
  void end(void);
private:
  uint32_t* _buffer[2];
  int _size;
  volatile int _index; 
};

#endif
