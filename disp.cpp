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


extern ComLogger xLogger;

Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM( DISPLAY_CS, DISPLAY_DC, DISPLAY_RST);       // Invoke custom library

void Display::Init() {
 

    //pinMode(DISPLAY_LED, PWM);
    //pwmWrite(DISPLAY_LED, 16000);
    pinMode(DISPLAY_LED, OUTPUT);
    digitalWrite(DISPLAY_LED, HIGH);

    tft.begin();    
    //tft.setRotation(2);
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


void Display::ShowStatus(char *msg) {    
    //strncpy(out_buf, msg, DISPLAY_LEN_S4);
    out_buf[DISPLAY_LEN_S4-1] = 0;
    //BufLen();
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.fillRect(0, 0, 240, 20, ILI9341_BLACK);
    //tft.drawString(out_buf,0,20,4);
    tft.drawString(msg,0,0,4);


}

void Display::ShowData(const int16_t d[3], int ndata, int row) {
    if(!ndata) return;
    if(ndata>6) ndata=6;
    *out_buf = 0;
    for(int i=0; i<ndata; i++) {
        itoa_cat(d[i], out_buf);
        if(i<ndata-1) strcat(out_buf, " ");
    }
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    //tft.fillRect(0, row*D_FONT_H, DISPLAY_H_SZ, D_FONT_H, ILI9341_BLACK);
    int16_t dx = tft.drawString(out_buf,0,row*D_FONT_H, D_FONT_SZ);
    tft.fillRect(dx, row*D_FONT_H, DISPLAY_H_SZ-dx, D_FONT_H, ILI9341_BLACK);
}

/*
void Display::ShowChart(const double *pdVals, int16_t nvals, 
        int16_t y, int16_t h, int16_t h0, int16_t xlab) {
    int16_t vmax=-32768, vmin=32767, vmed;
    int16_t v, y0=y+h0, xp;
    int8_t w, i;
    if(!pdVals || nvals<=0 || h<=0) return;
    for(i=0; i<nvals; i++) {
        v=(int16_t)pdVals[i];
        if(v<vmin) vmin=v;
        if(v>vmax) vmax=v;
    }
    vmed=(vmax+vmin)/2;
    // scale = h/(vmax-vmin+1)
    w=DISPLAY_H_SZ/(nvals);
    tft.fillRect(0, y, DISPLAY_H_SZ-1, h, ILI9341_BLACK);
    for(i=0; i<nvals; i++) {
      v=(int16_t)( ((int32_t)pdVals[i]-vmed)*h / ((int32_t)vmax-vmin+1) );
      xp=i*(w);
      if(v>=0) 
        tft.fillRect(xp, y0-v, w-1, v, ILI9341_RED);
      else  
        tft.fillRect(xp, y0, w-1, -v, ILI9341_GREEN);
        
    }
    tft.drawFastHLine(0, y0, DISPLAY_H_SZ-1, ILI9341_BLUE);
    //tft.setTextColor(ILI9341_YELLOW); // transparent
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    itoa(vmax, out_buf);
    tft.drawString(out_buf,0, y, D_FONT_S_SZ);
    itoa(vmed, out_buf);
    tft.drawString(out_buf,0, y0, D_FONT_S_SZ);
    itoa(vmin, out_buf);
    tft.drawString(out_buf, 0, y+h-D_FONT_S_H, D_FONT_S_SZ);
    itoa(xlab, out_buf);
    tft.drawRightString(out_buf, DISPLAY_H_SZ, y0, D_FONT_S_SZ);
}
*/

void Display::ShowChart0(const int16_t *pdVals, int16_t nvals, 
        int16_t y, int16_t h, int16_t xlab, int16_t noise) {
    int16_t vmax=0;
    int16_t v, y0=y+h/2, xp;
    uint16_t i;
    int8_t w, wc;
    if(!pdVals || nvals<=0 || h<=0) return;
    for(i=0; i<nvals; i++) {
        v=(int16_t)pdVals[i];
        v=abs(v);
        if(v>vmax) vmax=v;
    }
    w=DISPLAY_H_SZ/(nvals);
    if(nvals*w*6<DISPLAY_H_SZ*5) w++; // if eff width<5/6   
    wc=w<=1 ? 1 : w-1;    
    tft.fillRect(0, y, DISPLAY_H_SZ-1, h, ILI9341_BLACK);
    // vmzax is always>=0 due to abs 
    xp=0;
    for(i=0; i<nvals && xp+wc<DISPLAY_H_SZ; i++) {
        v=(int16_t)pdVals[i];  
        if(abs(v)<noise) continue;
        v=(int16_t)( ((int32_t)v)*h / ((int32_t)vmax+vmax+1) );
        if(v>=0) 
            tft.fillRect(xp, y0-v, wc, v, ILI9341_RED);
        else  
            tft.fillRect(xp, y0, wc, -v, ILI9341_GREEN);
        xp+=w;
    }
    tft.drawFastHLine(0, y0, DISPLAY_H_SZ-1, ILI9341_BLUE);
    tft.setTextColor(ILI9341_YELLOW); // transparent
    //tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    itoa(vmax, out_buf);
    tft.drawString(out_buf,0, y, D_FONT_S_SZ);
    itoa(0, out_buf);
    tft.drawString(out_buf,0, y0, D_FONT_S_SZ);
    itoa(-vmax, out_buf);
    tft.drawString(out_buf, 0, y+h-D_FONT_S_H, D_FONT_S_SZ);
    itoa(xlab, out_buf);
    tft.drawRightString(out_buf, DISPLAY_H_SZ, y0, D_FONT_S_SZ);
}
/*
void Display::ShowChartPlus(const double *pdVals, int16_t nvals, 
        int16_t y, int16_t h, int16_t xlab, int16_t noise) {
    int16_t vmax=-32768;
    int16_t v, y0=y+h, xp;
    int8_t w, i;
    if(!pdVals || nvals<=0 || h<=0) return;
    for(i=0; i<nvals; i++) {
        v=(int16_t)pdVals[i];
        if(v>vmax) vmax=v;
    }
    w=DISPLAY_H_SZ/(nvals);
    tft.fillRect(0, y, DISPLAY_H_SZ-1, h, ILI9341_BLACK);
    if(vmax<0) return;
    for(i=0; i<nvals; i++) {
        v=(int16_t)pdVals[i];  
        if(abs(v)<noise) continue;
        v=(int16_t)( ((int32_t)v)*h / ((int32_t)vmax+1) );
        xp=i*(w);
        tft.fillRect(xp, y0-v, w-1, v, ILI9341_RED);
    }
    tft.setTextColor(ILI9341_YELLOW); // transparent
    //tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    itoa(vmax, out_buf);
    tft.drawString(out_buf, 0, y, D_FONT_S_SZ);
    itoa(xlab, out_buf);
    tft.drawRightString(out_buf, DISPLAY_H_SZ, y0, D_FONT_S_SZ);
    itoa(xlab/2, out_buf);
    tft.drawRightString(out_buf, DISPLAY_H_SZ/2, y0, D_FONT_S_SZ);
}
*/

void Display::ShowChartPlusMax(const int16_t *pdVals, int16_t nvals, 
        int16_t y, int16_t h, int16_t xlab, int16_t vmax) {
    int16_t v, y0=y+h, xp;
    uint16_t i, color;
    uint8_t w, wc;
    w=DISPLAY_H_SZ/(nvals);
    if(w<=1) return;

    if(nvals*w*6<DISPLAY_H_SZ*5) w++; // if eff width<5/6   
    wc=w<=1 ? 1 : w-1;    

    tft.fillRect(0, y, DISPLAY_H_SZ-1, h, ILI9341_BLACK);
    //if(vmax<=0) return;
    xp=0;
    for(i=0; i<nvals && xp+wc<DISPLAY_H_SZ; i++) {
        v=(int16_t)pdVals[i];      
        v=(int16_t)( ((uint32_t)v)*h / ((int32_t)vmax) );
        //xp=i*(w);
        xp+=w;
        /*
        if(v<h)
            tft.fillRect(xp, y0-v, wc, v, ILI9341_GREEN);
        else    
            tft.fillRect(xp, y0-h, wc, h, ILI9341_RED);
            */
        color = v<h ? ILI9341_GREEN : ILI9341_RED;
        if(v>h) v=h;
        tft.fillRect(xp, y0-v, wc, v, color);

    }
    tft.setTextColor(ILI9341_YELLOW); // transparent
    //tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    itoa(vmax, out_buf);
    tft.drawString(out_buf, 0, y, D_FONT_S_SZ);
    itoa(xlab, out_buf);
    tft.drawRightString(out_buf, DISPLAY_H_SZ, y0, D_FONT_S_SZ);
    itoa(xlab/2, out_buf);
    tft.drawRightString(out_buf, DISPLAY_H_SZ/2, y0, D_FONT_S_SZ);
}

void Display::ShowCellChart(const int16_t *pdVals, int16_t nvals, 
      int16_t y, int16_t h, int16_t xlab, int16_t ncells, int16_t vmax) {
    int16_t v, y0=y+h, xp, ip;
    uint16_t i, color;
    uint8_t w, wc, hc, nc, ic;

    w=DISPLAY_H_SZ/(nvals);
    if(w<=1) return;
        
    if(nvals*w*6<DISPLAY_H_SZ*5) w++; // if eff width<5/6   
    wc=w<=1 ? 1 : w-1;    

    hc=h/ncells;
    if(hc<2) return;

    //tft.fillRect(0, y, DISPLAY_H_SZ-1, h, ILI9341_BLACK);
    xp=0;
    for(i=0; i<nvals && xp+wc<DISPLAY_H_SZ; i++) {
        v=(int16_t)pdVals[i];      
        v=(int16_t)( ((uint32_t)v)*h / ((int32_t)vmax) );
        xp+=w;
        /*
        if(v<h)
            tft.fillRect(xp, y0-v, wc, v, ILI9341_GREEN);
        else    
            tft.fillRect(xp, y0-h, wc, h, ILI9341_RED);
            */
        nc=v/hc;
        if(nc>ncells) nc=ncells;
        ip=y0-hc;
        for(ic=0; ic<ncells; ic++) {
            color = ic<nc ? ILI9341_GREEN : ILI9341_DARKGREY;
            tft.fillRect(xp, ip, wc, hc-1, color);
            ip-=hc;
        }

    }
    tft.setTextColor(ILI9341_YELLOW); // transparent
    //tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    itoa(vmax, out_buf);
    tft.drawString(out_buf, 0, y, D_FONT_S_SZ);
    itoa(xlab, out_buf);
    tft.drawRightString(out_buf, DISPLAY_H_SZ, y0, D_FONT_S_SZ);
    itoa(xlab/2, out_buf);
    tft.drawRightString(out_buf, DISPLAY_H_SZ/2, y0, D_FONT_S_SZ);
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
