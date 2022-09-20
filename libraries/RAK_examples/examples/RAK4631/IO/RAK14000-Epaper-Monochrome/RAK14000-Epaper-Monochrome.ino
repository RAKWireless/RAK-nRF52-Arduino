/**
   @file RAK14000-Epaper-Monochrome.ino.ino
   @author taylor.lee (taylor.lee@rakwireless.com)
   @brief display image and text on DEPG0213BNS800F41HP/DEPG0213BNS800F42HP epaper,
          and control by botton
   @version 0.1
   @date 2021-02-18

   @copyright Copyright (c) 2021

*/
#include <Arduino.h>

#include <Adafruit_GFX.h>
#include <Adafruit_EPD.h>

#include "images.h"

#define POWER_ENABLE   WB_IO2
#define EPD_MOSI       MOSI  
#define EPD_MISO       -1     // not use
#define EPD_SCK        SCK   
#define EPD_CS         SS    
#define EPD_DC         WB_IO1
#define SRAM_CS        -1     // not use
#define EPD_RESET      -1     // not use
#define EPD_BUSY       WB_IO4

#define LEFT_BUTTON    WB_IO3
#define MIDDLE_BUTTON  WB_IO5
#define RIGHT_BUTTON   WB_IO6

/*****************************************************/
/*****************************************************/
/* Select the correct display from the defines below */
/* The display type is printed on the back of the    */
/* display.                                          */
/* If you use a different display than the one      */
/* provided by RAKwireless you have to adjust the    */
/* the values matching with your custom display      */
/*****************************************************/
/*****************************************************/
typedef struct  DEPG {
   int  width;
   int  height;
   int  position1_x;
   int  position1_y;  
   int  position2_x;
   int  position2_y;  
   int  position3_x;
   int  position3_y; 
   int  position4_x;
   int  position4_y;   
} DEPG;

DEPG  DEPG_HP = {250,122,40,20,40,30,40,50,90,40};  //use DEPG0213BNS800F41HP as default
//DEPG  DEPG_HP = {212,104,30,15,30,25,30,45,80,30};  //  this is for DEPG0213BNS800F42HP



uint8_t gKeyNum = 0; // which button is pressed

// left button interrupt handle function
void interruptHandle1()
{
  if(gKeyNum == 0)
  {
    gKeyNum = 1;
  }
}

// middle button interrupt handle function
void interruptHandle2()
{
  if(gKeyNum == 0)
  {
    gKeyNum = 2;
  }
}

// right button interrupt handle function
void interruptHandle3()
{
  if(gKeyNum == 0)
  {
    gKeyNum = 3;
  }
}

// 2.13" EPD with SSD1680
Adafruit_SSD1680 display(DEPG_HP.width, DEPG_HP.height, EPD_MOSI,
                         EPD_SCK, EPD_DC, EPD_RESET,
                         EPD_CS, SRAM_CS, EPD_MISO,
                         EPD_BUSY);

void testdrawtext(int16_t x, int16_t y, char *text, uint16_t color);

/**
   @brief Arduino Setup function

*/
void setup(void)
{
  pinMode(POWER_ENABLE, INPUT_PULLUP);
  digitalWrite(POWER_ENABLE, HIGH);

  Serial.begin(115200);
  time_t timeout = millis();
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }

  // set left button interrupt
  pinMode(LEFT_BUTTON, INPUT);
  attachInterrupt(LEFT_BUTTON, interruptHandle1, FALLING);

  // set middle button interrupt
  pinMode(MIDDLE_BUTTON, INPUT);
  attachInterrupt(MIDDLE_BUTTON, interruptHandle2, FALLING);

  // set right button interrupt
  pinMode(RIGHT_BUTTON, INPUT);
  attachInterrupt(RIGHT_BUTTON, interruptHandle3, FALLING);


  Serial.println("EPD Epaper-DEPG0213BNS800F4xHP test");

  display.begin();

  // large block of text
  display.clearBuffer();

  display.drawBitmap(DEPG_HP.position1_x, DEPG_HP.position1_y, rak_img, 150, 56, EPD_BLACK);

  testdrawtext(DEPG_HP.position1_x, DEPG_HP.position1_y+50, "IoT Made Easy", (uint16_t)EPD_BLACK, 2);

  display.display(true);
}

/**
   @brief Arduino Loop function

*/
void loop()
{
  if(gKeyNum == 1)
  {
    Serial.println("Left button pressed");
    display.clearBuffer();
    display.drawBitmap(DEPG_HP.position2_x, DEPG_HP.position2_y, rak_img, 150, 56, EPD_BLACK);
    display.display(true);
    gKeyNum = 0;
  }

  if(gKeyNum == 2)
  {
    Serial.println("Middle button pressed");
    display.clearBuffer();
    testdrawtext(DEPG_HP.position3_x, DEPG_HP.position3_y, "IoT Made Easy", (uint16_t)EPD_BLACK, 2);
    display.display(true);
    gKeyNum = 0;
  }

  if(gKeyNum == 3)
  {
    Serial.println("Right button pressed");
    display.clearBuffer();
    display.drawBitmap(DEPG_HP.position4_x, DEPG_HP.position4_y, lora_img, 60, 40, EPD_BLACK);
    display.display(true);
    gKeyNum = 0;
  }
}

/**
   @brief Write a text on the display

   @param x x position to start
   @param y y position to start
   @param text text to write
   @param text_color color of text
   @param text_size size of text
*/
void testdrawtext(int16_t x, int16_t y, char *text, uint16_t text_color, uint32_t text_size)
{
  display.setCursor(x, y);
  display.setTextColor(text_color);
  display.setTextSize(text_size);
  display.setTextWrap(true);
  display.print(text);
}
