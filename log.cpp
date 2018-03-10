#include <MapleFreeRTOS821.h>
#include "log.h"


void ComLogger::Init() {
    vSemaphoreCreateBinary(xLogFree);
    xLogQueue = xQueueCreate( CLOG_Q_SZ, sizeof( struct AMessage ) );

    if( xLogQueue == NULL )
    {
        /* Queue was not created and must not be used. */
        Serial.println("Couldn't create LQ");
        return;
    }

    txMessage.ucMessageID=0;
    txMessage.ucData[0]=0;
    nFails = nIn = nOut = 0;
    xLastWakeTime = xTaskGetTickCount();

    Serial.println("Logger OK");
}

void ComLogger::vAddLogMsg(const char *pucMsg) {  
  if ( xSemaphoreTake( xLogFree, ( portTickType ) 10 ) == pdTRUE )
    {
      txMessage.ucMessageID++;     
      if(pucMsg) 
        strncpy(txMessage.ucData, pucMsg, CLOG_MSG_SZ);          
      else *txMessage.ucData=0;  
      if(xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 ) != pdPASS ) {
        nFails++;
      } else nIn++;
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
      if(xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 ) != pdPASS ) {
        nFails++;
      } else nIn++;
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
      
      if(xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 ) != pdPASS ) {
        nFails++;
      } else nIn++;
      xSemaphoreGive( xLogFree );
    }
}

/*
void ComLogger::vAddLogMsg(const char *pucMsg1, int16_t i1, int16_t i2, int16_t i3) {
   if ( xSemaphoreTake( xLogFree, ( portTickType ) 10 ) == pdTRUE )
    {
      txMessage.ucMessageID++;     
      if(pucMsg1) 
        strncpy(txMessage.ucData, pucMsg1, CLOG_MSG_SZ);          
      else *txMessage.ucData=0;  
      strncat(txMessage.ucData, ":", CLOG_MSG_SZ);          
      itoa_cat(i1, txMessage.ucData);
      strncat(txMessage.ucData, ",", CLOG_MSG_SZ);          
      itoa_cat(i2, txMessage.ucData);
      strncat(txMessage.ucData, ",", CLOG_MSG_SZ);          
      itoa_cat(i3, txMessage.ucData);
      
      xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
      xSemaphoreGive( xLogFree );
    }
}
*/

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
      
      if(xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 ) != pdPASS ) {
        nFails++;
      } else nIn++;
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
      if(xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 ) != pdPASS ) {
        nFails++;
      } else nIn++;
      xSemaphoreGive( xLogFree );
    }
}
void ComLogger::Process() {  
  if( xQueueReceive( xLogQueue, &rxMessage, ( TickType_t ) 10 ) )
  {
    Serial.print((int)rxMessage.ucMessageID);
    Serial.print(" : ");
    Serial.println(rxMessage.ucData);
    nOut++;
    vTaskDelay(5);         
   }        
  if ((int16_t)(xTaskGetTickCount()-xLastWakeTime)>5000) {
    Serial.print("* SS: ");
    Serial.print(nIn);
    Serial.print(", ");
    Serial.print(nOut);
    Serial.print(", ");
    Serial.println(nFails);
    xLastWakeTime = xTaskGetTickCount();
  }
}



/* reverse:  reverse string s in place */
 void reverse(char s[])
 {
     int i, j;
     char c;

     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
}  

/* itoa:  convert n to characters in s */
void itoa(int n, char s[])
 {
     int i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}  


/* itoa:  convert n to characters in s */

void ltoa(int32_t n, char s[])
 {
     int32_t i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}  

