/**
 * @file LookBack.ino
 * @author rakwireless.com
 * @brief This example is used to test I2S read and write, you need to connect the I2S read and write pins together.
 * @version 0.1
 * @date 2022-06-20
 * @copyright Copyright (c) 2022
 */
#include "audio.h"
#include "I2S.h"
#include "Arduino.h"

#define I2S_DATA_BLOCK_WORDS    512

i2s_channels_t channels = Left ;//Stereo ;// Right // 
int frequency = 16000;  // sample rate in Hz
int sampleBit = 16;

uint32_t readbuff[512]= {0}; 
uint32_t writebuff[512]= {0};

uint16_t leftChannel[512] = {0};
uint16_t rightChannel[512] = {0};

volatile uint8_t rx_flag = 0;  
volatile uint8_t tx_flag = 0; 

void i2s_config();
void tx_irq(); 
void rx_irq(); 
void setup()
{   
  pinMode(WB_IO2,OUTPUT);
  digitalWrite(WB_IO2,HIGH);   
  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 3000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  } 
  for(int i=0;i<512;i++)
  {
    writebuff[i] = i+1;
  }
    
  i2s_config();
  delay(200);
  I2S.write(&writebuff,sizeof(writebuff));  
  Serial.println("=====================================");  
}

void loop()
{  
  if(rx_flag==1)
  {    
    rx_flag = 0;
    Serial.println("I2S read---------------------------------------");
    I2S.read(&readbuff,512);
    for(int i=0;i<512;i++)
    {
       uint32_t const * p_word = &readbuff[i];
       leftChannel[i] = ((uint16_t const *)p_word)[0];
       rightChannel[i] = ((uint16_t const *)p_word)[1];     
     
      Serial.printf("%08X\t\tL:%04X\tR:%04X\t\r\n",readbuff[i],leftChannel[i],rightChannel[i]);
    }
    Serial.println("I2S write ########################################");  
    if(tx_flag == 1)
    {
      I2S.write(&writebuff,sizeof(writebuff)); 
      tx_flag = 0; 
    } 
  }
}

void rx_irq()   ///< Pointer to the buffer for received data.
{   
  rx_flag = 1;  
//  I2S.read(&readbuff,sizeof(readbuff));     
}
void tx_irq()  ///< Pointer to the buffer with data to be sent.
{ 
  tx_flag = 1;   
}
void i2s_config()
{
  I2S.TxIRQCallBack(tx_irq);
  I2S.RxIRQCallBack(rx_irq);
  I2S.begin(channels,frequency,sampleBit);
  I2S.start();  
}

 
