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

Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM( DISPLAY_CS, DISPLAY_DC, DISPLAY_RST);       // Invoke custom library

void Display::Init() {
 

    pinMode(DISPLAY_LED, PWM);
    pwmWrite(DISPLAY_LED, 16000);

    tft.begin();    
    tft.setRotation(2);
    tft.fillScreen(ILI9341_BLACK);  
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
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
    
    tft.drawString("DISP OK",20,20,4);
}

/*
void ComLogger::vAddLogMsg(const char *pucMsg) {  
  if ( xSemaphoreTake( xLogFree, ( portTickType ) 10 ) == pdTRUE )
    {
      txMessage.ucMessageID++;     
      if(pucMsg) 
        strncpy(txMessage.ucData, pucMsg, CLOG_MSG_SZ);          
      else *txMessage.ucData=0;  
      xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
      xSemaphoreGive( xLogFree );
    }
}

void ComLogger::vAddLogMsg(const char *pucMsg, int16_t i) {  
   if ( xSemaphoreTake( xLogFree, ( portTickType ) 10 ) == pdTRUE )
    {
      txMessage.ucMessageID++;     
      if(pucMsg) 
        strncpy(txMessage.ucData, pucMsg, CLOG_MSG_SZ);          
      else *txMessage.ucData=0;        
      strncat(txMessage.ucData, ":", CLOG_MSG_SZ);          
      itoa_cat(i, txMessage.ucData);
      xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
      xSemaphoreGive( xLogFree );
    }
}

void ComLogger::vAddLogMsg(const char *pucMsg1, int16_t i1, const char *pucMsg2, int16_t i2) {  
   if ( xSemaphoreTake( xLogFree, ( portTickType ) 10 ) == pdTRUE )
    {
      txMessage.ucMessageID++;     
      if(pucMsg1) 
        strncpy(txMessage.ucData, pucMsg1, CLOG_MSG_SZ);          
      else *txMessage.ucData=0;  
      strncat(txMessage.ucData, ":", CLOG_MSG_SZ);          
      itoa_cat(i1, txMessage.ucData);
      if(pucMsg2) {
        strncat(txMessage.ucData, ",", CLOG_MSG_SZ);                
        strncat(txMessage.ucData, pucMsg2, CLOG_MSG_SZ);          
      }
      strncat(txMessage.ucData, ":", CLOG_MSG_SZ);          
      itoa_cat(i2, txMessage.ucData);
      
      xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
      xSemaphoreGive( xLogFree );
    }
}


void ComLogger::vAddLogMsg(const char *pucMsg1, int32_t i1, int32_t i2, int32_t i3) {
   if ( xSemaphoreTake( xLogFree, ( portTickType ) 10 ) == pdTRUE )
    {
      txMessage.ucMessageID++;     
      if(pucMsg1) 
        strncpy(txMessage.ucData, pucMsg1, CLOG_MSG_SZ);          
      else *txMessage.ucData=0;  
      strncat(txMessage.ucData, ":", CLOG_MSG_SZ);          
      ltoa_cat(i1, txMessage.ucData);
      strncat(txMessage.ucData, ",", CLOG_MSG_SZ);          
      ltoa_cat(i2, txMessage.ucData);
      strncat(txMessage.ucData, ",", CLOG_MSG_SZ);          
      ltoa_cat(i3, txMessage.ucData);
      
      xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
      xSemaphoreGive( xLogFree );
    }
}

void ComLogger::vAddLogMsg(const char *pucMsg1, int32_t i1, int32_t i2, int32_t i3, int32_t i4) {
   if ( xSemaphoreTake( xLogFree, ( portTickType ) 10 ) == pdTRUE )
    {
      txMessage.ucMessageID++;     
      if(pucMsg1) 
        strncpy(txMessage.ucData, pucMsg1, CLOG_MSG_SZ);          
      else *txMessage.ucData=0;  
      strncat(txMessage.ucData, ":", CLOG_MSG_SZ);          
      ltoa_cat(i1, txMessage.ucData);
      strncat(txMessage.ucData, ",", CLOG_MSG_SZ);          
      ltoa_cat(i2, txMessage.ucData);
      strncat(txMessage.ucData, ",", CLOG_MSG_SZ);          
      ltoa_cat(i3, txMessage.ucData);
      strncat(txMessage.ucData, ",", CLOG_MSG_SZ);          
      ltoa_cat(i4, txMessage.ucData);      
      xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
      xSemaphoreGive( xLogFree );
    }
}

*/

static int c=0;
void Display::Process() { 
  
  /*

  if( xQueueReceive( xLogQueue, &rxMessage, ( TickType_t ) 10 ) )
  {
    Serial.print((int)rxMessage.ucMessageID);
    Serial.print(" : ");
    Serial.println(rxMessage.ucData);
    vTaskDelay(100);         
   }       
   */
   strcpy(out_buf, "PROC : ");          
   itoa_cat(c++, out_buf);
   tft.drawCentreString(out_buf,160,48,2); // Next size up font 2
}


