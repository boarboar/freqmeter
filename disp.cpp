#include <MapleFreeRTOS821.h>
#include <Adafruit_GFX_AS.h>    // Core graphics library, with extra fonts.
#include "Adafruit_ILI9341_STM_1.h" // STM32 DMA Hardware-specific library
#include <SPI.h>
#include "log.h"
#include "disp.h"

#define DISPLAY_LED PA2 

// GPIO DISPLAY_LED
#define DISPLAY_CS  PA3
#define DISPLAY_DC  PA1
#define DISPLAY_RST PA4

#define DISPLAY_LEN_S4  16
#define DISPLAY_H_SZ  240
#define DISPLAY_V_SZ  320

extern ComLogger xLogger;

Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM( DISPLAY_CS, DISPLAY_DC, DISPLAY_RST);       // Invoke custom library

void Display::Init() {
 

    pinMode(DISPLAY_LED, PWM);
    pwmWrite(DISPLAY_LED, 16000);

    tft.begin();    
    tft.setRotation(2);
    tft.fillScreen(ILI9341_BLACK);  
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);

    /*
    tft.drawCentreString("HELLO",160,48,2); // Next size up font 2

    vSemaphoreCreateBinary(xDispFree);

    //tft.drawString("SEM OK",20,20,4);
       
    xDispQueue = xQueueCreate( CLOG_Q_SZ, sizeof( struct AMessage ) );

    if( xDispQueue == NULL )
    {
        // Queue was not created and must not be used. 
        //Serial.println("Couldn't create LQ");
        //  tft.drawString("Q FAIL",20,20,4);
        return;
    }

    //tft.drawString("Q OK",20,20,4);

    txMessage.ucMessageID=0;
    txMessage.ucData[0]=0;
    */

    //tft.drawString("DISP OK",160,48,2);

    ShowStatus("0123456789abcdef");
    
    int8_t w=240/(32);
    for(int i=0; i<32; i++) {
      int16_t h=i*5+2;
      tft.fillRect(i*(w), DISPLAY_V_SZ-h-1, w-1, h, (i%10==0 ? ILI9341_GREEN : ILI9341_RED));
    }

}


void Display::ShowStatus(const char *msg) {    
    strncpy(out_buf, msg, DISPLAY_LEN_S4);
    out_buf[DISPLAY_LEN_S4-1] = 0;
    //BufLen();
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.fillRect(0, 20, 240, 20, ILI9341_BLACK);
    tft.drawString(out_buf,0,20,4);


}

void Display::ShowData3(const int16_t d[3], int row) {
    const int D_FONT_SZ = 4;
    const int D_FONT_H = 26;
    itoa(d[0], out_buf);
    strcat(out_buf, " ");
    itoa_cat(d[1], out_buf);
    strcat(out_buf, " ");
    itoa_cat(d[2], out_buf);
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.fillRect(0, row*D_FONT_H, DISPLAY_H_SZ, D_FONT_H, ILI9341_BLACK);
    tft.drawString(out_buf,0,row*D_FONT_H, D_FONT_SZ);
}

/*
static int c=0;
void Display::Process() { 
   strcpy(out_buf, "PROC : ");          
   itoa_cat(c++, out_buf);
   tft.drawCentreString(out_buf,160,48,2); // Next size up font 2
}
*/

void Display::BufLen() {
  int len = strlen(out_buf);
  while(len<DISPLAY_LEN_S4-1) {
    out_buf[len]=' ';
    len++;    
  }
  out_buf[len] = 0;
}
