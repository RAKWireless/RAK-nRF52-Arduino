#include <stdlib.h>
#include <string.h>
#include "arduino.h"
#include "I2SDoubleBuffer.h"


static uint32_t buffer[2][DEFAULT_I2S_BUFFER_SIZE];

I2SDoubleBuffer::I2SDoubleBuffer() :
  _size(DEFAULT_I2S_BUFFER_SIZE)
{
  reset();
}

I2SDoubleBuffer::~I2SDoubleBuffer()
{
}

void I2SDoubleBuffer::setSize(int size)
{
  _size = size;
}
int I2SDoubleBuffer::size(void)
{
  int size = _size;
  return size;
}
void I2SDoubleBuffer::reset()
{
  // _buffer[0] = (uint32_t*)realloc(_buffer[0], _size);
  // _buffer[1] = (uint32_t*)realloc(_buffer[1], _size);  

  _buffer[0] = buffer[0];
  _buffer[1] = buffer[1];

  memset(_buffer[0], 0x00, _size);
  memset(_buffer[1], 0x00, _size);
  _index = 0;
}
size_t I2SDoubleBuffer::write(const void *buffer, size_t size)
{
  memcpy(_buffer[_index], buffer, size);  
  return size;
}

size_t I2SDoubleBuffer::read(void *buffer, size_t size)
{
  memcpy(buffer, _buffer[_index], size);
  return size;
}

void* I2SDoubleBuffer::data()
{
  return (void*)_buffer[_index];  
}

void* I2SDoubleBuffer::data(uint8_t index)
{
  return (void*)_buffer[index];  
}

int I2SDoubleBuffer::swap(void)
{  
  int num = 0;
  if (_index == 0) {
    _index = 1;
  } else {
    _index = 0;
  }
  num = _index;
  return num;
}

void I2SDoubleBuffer::end(void)
{
  free(_buffer);
}
