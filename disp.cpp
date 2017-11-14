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

#define D_FONT_SZ   4
#define D_FONT_H    26

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
    /*
    int8_t w=240/(32);
    for(int i=0; i<32; i++) {
      int16_t h=i*5+2;
      tft.fillRect(i*(w), DISPLAY_V_SZ-h-1, w-1, h, (i%10==0 ? ILI9341_GREEN : ILI9341_RED));
    }
    */
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
    itoa(d[0], out_buf);
    strcat(out_buf, " ");
    itoa_cat(d[1], out_buf);
    strcat(out_buf, " ");
    itoa_cat(d[2], out_buf);
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.fillRect(0, row*D_FONT_H, DISPLAY_H_SZ, D_FONT_H, ILI9341_BLACK);
    tft.drawString(out_buf,0,row*D_FONT_H, D_FONT_SZ);
}

void Display::ShowChart(const double *pdVals, int16_t nvals, int16_t y, int16_t h) {
    int16_t vmax=-32768, vmin=32767, vmed;
    int16_t v;
    int8_t w, i;
    if(!pdVals || nvals<=0 || h<=0) return;
    for(i=0; i<nvals; i++) {
        v=(int16_t)pdVals[i];
        if(v<vmin) vmin=v;
        if(v>vmax) vmax=v;
    }
    vmed=(vmax-vmin)/2;
    //xLogger.vAddLogMsg("VMIN", vmin, "VMAX", vmax);
    // scale = h/(vmax-vmin+1)
    w=240/(nvals);
    tft.fillRect(0, DISPLAY_V_SZ-h, DISPLAY_H_SZ-1, DISPLAY_V_SZ-1, ILI9341_DARKGREY);
    for(i=0; i<nvals; i++) {
      //int16_t h=i*5+2;
      v=(int16_t)( ((int32_t)pdVals[i]-vmed)*h / ((int32_t)vmax-vmin+1) );
      if(v>=0)
        tft.fillRect(i*(w), y-v, w-1, v, ILI9341_RED);
      else  
        tft.fillRect(i*(w), y, w-1, -v,ILI9341_GREEN);
    }
    tft.drawFastHLine(0, y, DISPLAY_H_SZ-1, ILI9341_BLUE);
    itoa(vmax, out_buf);
    tft.drawString(out_buf,0,DISPLAY_V_SZ-h, D_FONT_SZ);
    itoa(vmed, out_buf);
    tft.drawString(out_buf,0,DISPLAY_V_SZ-h/2, D_FONT_SZ);
    itoa(vmin, out_buf);
    tft.drawString(out_buf,0,DISPLAY_V_SZ-D_FONT_H, D_FONT_SZ);
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
