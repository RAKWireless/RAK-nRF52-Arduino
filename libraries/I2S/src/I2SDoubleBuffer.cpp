#include <stdlib.h>
#include <string.h>
#include "arduino.h"
#include "I2SDoubleBuffer.h"

I2SDoubleBuffer::I2SDoubleBuffer() :
  _size(DEFAULT_I2S_BUFFER_SIZE)
{
  reset();
}

I2SDoubleBuffer::~I2SDoubleBuffer()
{
  free(_buffer);
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
// void I2SDoubleBuffer::reset()
// {
//   if(!_buffer)
//   {
//     free(_buffer);
//   }
//   _buffer[0] = (uint8_t*)realloc(_buffer[0], _size);
//   _buffer[1] = (uint8_t*)realloc(_buffer[1], _size);

//   memset(_buffer[0], 0x00, _size);
//   memset(_buffer[1], 0x00, _size);
//   _index = 0;
// }
void I2SDoubleBuffer::reset()
{
  if(!_buffer)
  {
    free(_buffer);
  }
  _buffer[0] = (uint32_t*)realloc(_buffer[0], _size*sizeof(uint32_t));
  _buffer[1] = (uint32_t*)realloc(_buffer[1], _size*sizeof(uint32_t));

  memset(_buffer[0], 0x00, _size*sizeof(uint32_t));
  memset(_buffer[1], 0x00, _size*sizeof(uint32_t));
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
  _index = !_index; 
  return _index;
}

void I2SDoubleBuffer::end(void)
{
  free(_buffer);
}
