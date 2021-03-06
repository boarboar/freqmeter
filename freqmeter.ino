
#include <MapleFreeRTOS821.h>
#include <Wire.h>
#include <Adafruit_GFX_AS.h>    // Core graphics library, with extra fonts.
#include "Adafruit_ILI9341_STM_1.h" // STM32 DMA Hardware-specific library
#include <SPI.h>
#include "arduinoFFT.h"

#include "fix_fft.h"

#include "disp.h"
#include "log.h"
#include "mpu.h"

/*
Adafruit_GFX_AS : Load_fonts.h to be fixed:
//#define LOAD_GLCD // Standard Adafruit font needs ~1792 bytes in FLASH

#define LOAD_FONT2 // Small font, needs ~3092 bytes in FLASH
#define LOAD_FONT4 // Medium font, needs ~8126 bytes in FLASH
//#define LOAD_FONT6 // Large font, needs ~4404 bytes in FLASH
//#define LOAD_FONT7 // 7 segment font, needs ~3652 bytes in FLASH
*/

/*
 * To enable timers in FreeRTOS
 * <MapleFreeRTOS821.h>
#include "utility/timers.h"
 * 
 * FreeRTOSConfig.h
 * 
#define configUSE_TIMERS       1
#define configTIMER_TASK_PRIORITY   (tskIDLE_PRIORITY + 3)
#define configTIMER_QUEUE_LENGTH    4
#define configTIMER_TASK_STACK_DEPTH  (configMINIMAL_STACK_SIZE)
 */

/*
 * vacant ports
 * ===Side 1
 * PA0
 * PA7  PWM
 * PB0  PWM
 * PB1  PWM
 * PC14 low_current not use as a constant current source eg LED
 * PC15 low_current not use as a constant current source eg LED
 * ===Side 2
 * PA9  PWM FT
 * PA10 PWM FT
 * PA11 PWM FT
 * PA12     FT * PA15 PWM FT
 * PB3  PWM FT
 * PB4  PWM FT
 * PB5  PWM
 */

#define BOARD_LED_PIN PC13

/*
// PWM ports (check!)
#define DISPLAY_LED PA2 

// GPIO DISPLAY_LED
#define DISPLAY_CS  PA3
#define DISPLAY_DC  PA1
#define DISPLAY_RST PA4

*/

#define TASK_DELAY_LOG 10
#define TASK_DELAY_DISP 10
//#define TASK_DELAY_MPU 1   // 1kHz
#define TASK_DELAY_MPU 2   // 500Hz
//#define TASK_DELAY_MPU 3   // 333Hz
//#define TASK_DELAY_MPU 4   // 250Hz
//#define TASK_DELAY_MPU 8

Display xDisplay;
ComLogger xLogger;

arduinoFFT FFT; /* Create FFT object */

boolean fMPUReady=false;
boolean fSAMPReady=false;

TimerHandle_t xIMU_TimerSMP=0;
TimerHandle_t xIMU_TimerInit=0;

// FFT

#define NOISE_CUT_OFF   2

// https://github.com/kosme/arduinoFFT
//static const uint16_t FFT_SAMPLES = 64;//This value MUST ALWAYS be a power of 2

static const uint16_t FFT_SAMPLES = N_WAVE;//This value MUST ALWAYS be a power of 2

// with sampling at 1000 Hz, we get width 1000/2 = 500 Hz
// discrete of (1000/2) / (64/2) = 500/32 = 15 Hz
//double vSamp[FFT_SAMPLES];
//double vReal[FFT_SAMPLES];
//double vImag[FFT_SAMPLES];  

int16_t  vSamp[FFT_SAMPLES]={0};
int16_t  vReal[FFT_SAMPLES]={0};
int16_t  vImag[FFT_SAMPLES]={0};  


static void vSerialOutTask(void *pvParameters) {
    Serial.println("Serial Out Task started.");
    for (;;) {
       xLogger.Process();
       vTaskDelay(TASK_DELAY_LOG);
    }
}

static void vDispOutTask(void *pvParameters) {
    //tft.drawString("Task started!",20,20,4);
    xLogger.vAddLogMsg("Disp task started.");    
    int16_t a[6]={0,0,0,0,0,0};
    int16_t i;
    //int16_t iOverTimeCount1=0;
    /*
    {
        // test log
        int16_t x[4]={2, 5, 7, 12};
        xLogger.vAddLogMsg("X", x[0], x[1], x[2], x[3]);
        fix_fft_log(x, 4);
        xLogger.vAddLogMsg("L", x[0], x[1], x[2], x[3]);
    }
    */
    TestChart(20); // 20Hz
    vTaskDelay(5000);
    TestChart(50); // 50Hz
    vTaskDelay(5000);
    TestChart(100); // 100Hz
    vTaskDelay(5000);
    TestChart(250); // 250Hz
    
    boolean bSampReady=false;
    for (;;) {
        if(fMPUReady) {
            bSampReady=false;
            if(MpuDrv::Mpu.Acquire()) {               
                bSampReady=fSAMPReady;
            
                if(bSampReady) {
                    //xLogger.vAddLogMsg("GET!!!"); 
                    for(i=0; i<FFT_SAMPLES; i++) vReal[i]=vSamp[i];                
                }
                /*
                a[0] = MpuDrv::Mpu.FFT_GetDataSampCount();
                a[1] = MpuDrv::Mpu.FFT_GetOverTimeCount1();
                a[2] = MpuDrv::Mpu.FFT_GetDataMissCount();
                a[3] = MpuDrv::Mpu.FFT_GetFIFOOvflCount();
                a[4] = MpuDrv::Mpu.FFT_GetFIFOXcsCount();
                a[5] = MpuDrv::Mpu.FFT_GetSampleTime();
                */
                a[0] = MpuDrv::Mpu.FFT_GetSampleTime();
                a[1] = MpuDrv::Mpu.FFT_GetDataMissCount();
                a[2] = MpuDrv::Mpu.FFT_GetOverTimeCount1();                
                MpuDrv::Mpu.Release();
           } 
           
            if(bSampReady) {             
                //xLogger.vAddLogMsg("DRAW+++");
                xDisplay.ShowData(a, 3);
                //FFT_StartSampling();                
                FFT_Do(vReal, vImag, false);
                FFT_StartSampling();          
                //xLogger.vAddLogMsg("DRAW---");      
            }
        }
        else {                        
            xDisplay.ShowStatus("Warm up...");
        }
       vTaskDelay(TASK_DELAY_DISP);
    }
}

void vIMU_TimerCallbackInit( TimerHandle_t xTimer )
 {
    int16_t mpu_res = -1;
    if(MpuDrv::Mpu.Acquire()) {
        mpu_res = MpuDrv::Mpu.cycle(0);     
        MpuDrv::Mpu.Release();
      } else return;
    if(mpu_res==2) {
        // IMU settled
        fMPUReady=true;
        if(xTimerStop( xIMU_TimerInit, 10 )==pdFAIL) {
            xLogger.vAddLogMsg("Fail to stop!");    
        }
        xLogger.vAddLogMsg("MPU Ready!");
        xDisplay.ShowStatus("Ready");
        vTaskDelay(2);
        FFT_StartSampling();
    }
 }

void vIMU_TimerCallbackSample( TimerHandle_t xTimer )
 {
    if(!fMPUReady || fSAMPReady) return;
    boolean bSampReady=false;     
    if(MpuDrv::Mpu.Acquire()) {
        MpuDrv::Mpu.cycle(0);       
        bSampReady = MpuDrv::Mpu.FFT_SamplingReady();
        MpuDrv::Mpu.Release();
      } else return;

    if(bSampReady) {
        if(xTimerStop( xIMU_TimerSMP, 10 )==pdFAIL) {
            xLogger.vAddLogMsg("Fail to stop!");    
        }
        digitalWrite(BOARD_LED_PIN, HIGH);         
        //xLogger.vAddLogMsg("Sample Ready!");
        //xDisplay.ShowStatus("Sample Ready");
        if(MpuDrv::Mpu.Acquire()) {
          if(!fSAMPReady) { 
            //xLogger.vAddLogMsg("SMP!!!");  
            fSAMPReady=true;
            }
          MpuDrv::Mpu.Release();
        }
    }
 }

void setup() {
    digitalWrite(BOARD_LED_PIN, LOW);
    pinMode(BOARD_LED_PIN, OUTPUT);
    
    delay(5000);
    //disableDebugPorts(); // disable JTAG debug, enable PB3,4 for usage
    // NEVER do this!!!

    xDisplay.Init();
    Serial.begin(115200); 
    Serial.print("Tick = ");
    Serial.println(portTICK_PERIOD_MS);
        
    xLogger.Init();

    Serial.println("Init Wire...");
    //Wire.begin(SCL_PIN, SDA_PIN);
    Wire.begin();
    MpuDrv::Mpu.init();
    MpuDrv::Mpu.FFT_SetSampling(vSamp, FFT_SAMPLES);
    //MpuDrv::Mpu.FFT_SetSampling(vReal, FFT_SAMPLES);

    //xDisplay.ShowCellChart(vReal, i, 320-128-D_FONT_S_H, 128, ((1000/TASK_DELAY_MPU)>>1), 8, 25);  // init chart bg

    Serial.println("Starting...");
    digitalWrite(BOARD_LED_PIN, HIGH);
       
    xTaskCreate(vSerialOutTask,
                "TaskSO",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1, // low
                NULL);

    xTaskCreate(vDispOutTask,
                "TaskDISP",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1, // low
                NULL);
                

    xIMU_TimerSMP = xTimerCreate( 
                     "IMU_Timer_SMP",
                     TASK_DELAY_MPU,                   
                     pdTRUE,
                     ( void * ) 0,
                     vIMU_TimerCallbackSample
                   );

    if( xIMU_TimerSMP == NULL )
     {
         /* The timer was not created. */
         Serial.println("IMU_Timer SMP - failed to create");
     }


    xIMU_TimerInit = xTimerCreate( 
                     "IMU_Timer_INIT",
                     8,                     
                     pdTRUE,
                     ( void * ) 0,
                     vIMU_TimerCallbackInit
                   );

    if( xIMU_TimerInit == NULL )
     {
         /* The timer was not created. */
         Serial.println("IMU_Timer - failed to create");
     }
    else
     {
         /* Start the timer.  No block time is specified, and
         even if one was it would be ignored because the RTOS
         scheduler has not yet been started. */
         if( xTimerStart( xIMU_TimerInit, 0 ) != pdPASS )
         {
             Serial.println("IMU_Timer - failed to start");
         }
     }                        
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}

void TestChart(double signalFrequency) {
    // test Display
    /* Build raw data */
    const double samplingFrequency = 1000;
    //const uint8_t amplitude = 100;
    const uint16_t amplitude = 1000;
    double cycles = (((FFT_SAMPLES-1) * signalFrequency) / samplingFrequency); //Number of signal cycles that the sampling will read
    for (uint16_t i = 0; i < FFT_SAMPLES; i++)
    {
        vReal[i] = (int16_t)( (amplitude * (sin((i * (PI*2 * cycles)) / FFT_SAMPLES))) / 2.0);/* Build data with positive and negative values*/        
    }
    FFT_Do(vReal, vImag, true);
}

void  FFT_StartSampling() {    
    digitalWrite(BOARD_LED_PIN, LOW); 
    fSAMPReady=false;       
    if(MpuDrv::Mpu.Acquire()) {
        MpuDrv::Mpu.FFT_StartSampling();               
        MpuDrv::Mpu.Release();
    }

    if( xIMU_TimerSMP != NULL )
     {
         /* Start the timer.  No block time is specified, and
         even if one was it would be ignored because the RTOS
         scheduler has not yet been started. */
         if( xTimerStart( xIMU_TimerSMP, 0 ) != pdPASS )
         {
             /* The timer could not be set into the Active
             state. */
             xLogger.vAddLogMsg("SMP FAIL STRT");             
         } else {
             //xLogger.vAddLogMsg("SMP STRT OK");             
         }
     }      
}


void  FFT_Do(int16_t *vReal, int16_t *vImag, boolean doLogTiming) {    
    //xLogger.vAddLogMsg("DO");  
    uint16_t i;
    uint32_t xRunTime=xTaskGetTickCount();
    
    fix_fft_debias(vReal, FFT_SAMPLES);

    //fix_fft_wnd(vReal, FFT_SAMPLES); //!!! just for test

    xDisplay.ShowChart0(vReal, FFT_SAMPLES, 320-256-D_FONT_S_H*2, 128, TASK_DELAY_MPU*FFT_SAMPLES);
    if(doLogTiming)
        xLogger.vAddLogMsg("CH0", (int16_t)(xTaskGetTickCount()-xRunTime));

    xRunTime=xTaskGetTickCount();    
    /*
    FFT.Windowing(vReal, FFT_SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);	// Weigh data
    if(doLogTiming)
        xLogger.vAddLogMsg("WGT", (int16_t)(xTaskGetTickCount()-xRunTime));
      unTime=xTaskGetTickCount();
    */

    for (i = 0; i < FFT_SAMPLES; i++) vImag[i] = 0;
    i = FFT_ComputeMagnitudeFix(vReal, vImag); 

    //FFT_ComputeMagnitude(vReal, vImag, FFT_SAMPLES); 


/*
    FFT_Log(vReal, (FFT_SAMPLES>>1));
    if(doLogTiming)
        xLogger.vAddLogMsg("LOG", (int16_t)(xTaskGetTickCount()-xRunTime));
        
    xRunTime=xTaskGetTickCount();
    xDisplay.ShowChartPlusMax(vReal, (FFT_SAMPLES>>1), 320-128-D_FONT_S_H, 128, ((1000/TASK_DELAY_MPU)>>1), 100, NOISE_CUT_OFF);    
    */

    if(doLogTiming)
        xLogger.vAddLogMsg("CMP", (int16_t)(xTaskGetTickCount()-xRunTime));
    xRunTime=xTaskGetTickCount();

    //xDisplay.ShowChart0(vReal, (FFT_SAMPLES>>1), 320-128-D_FONT_S_H, 128, ((1000/TASK_DELAY_MPU)>>1));    

    //xDisplay.ShowChartPlusMax(vReal, (FFT_SAMPLES>>1), 320-128-D_FONT_S_H, 128, ((1000/TASK_DELAY_MPU)>>1), 100, NOISE_CUT_OFF);    
    //xDisplay.ShowChartPlusMax(vReal, (FFT_SAMPLES>>1), 320-128-D_FONT_S_H, 128, ((1000/TASK_DELAY_MPU)>>1), 25);    
    xDisplay.ShowCellChart(vReal, i, 320-128-D_FONT_S_H, 128, ((1000/TASK_DELAY_MPU)>>1), 8, 25); 

    
    if(doLogTiming)
        xLogger.vAddLogMsg("CH1", (int16_t)(xTaskGetTickCount()-xRunTime));  
                 
}


uint16_t  FFT_ComputeMagnitudeFix(int16_t *vReal, int16_t *vImag) {
    uint16_t n2=FFT_SAMPLES>>1;
    
    //fix_fft_wnd(vReal, FFT_SAMPLES);

    fix_fft(vReal, vImag, LOG2_N_WAVE, 0);  
    fix_fft_cp2m(vReal, vImag, n2);
    vReal[0]/=2; //DC
    fix_fft_sq2(vReal, n2);
    n2>>=1;
    fix_fft_sq2(vReal, n2);
    n2>>=1;
    fix_fft_denoise(vReal, n2, NOISE_CUT_OFF);
    fix_fft_log(vReal, n2);
    return n2;
  }

/*
void  FFT_DeBias(double *pdSamples, int8_t n) {
    double mean = 0;
    uint16_t i;
    for(i=0; i<n; i++) {
        mean+=pdSamples[i];
    }
    mean/=n;
    for(i=0; i<n; i++) {
        pdSamples[i]-=mean;
    }
  }
  
  void  FFT_ComputeMagnitude(double *vReal, double *vImag, int8_t n) {
    FFT.Compute(vReal, vImag, n, FFT_FORWARD); // Compute FFT 
    FFT.ComplexToMagnitude(vReal, vImag, n/2); // Compute magnitudes 
    vReal[0]/=n; // DC
    uint16_t i, n2=n>>1;
    for(i=1; i<n2; i++) {
        vReal[i]/=n2;
    }    
  }

void  FFT_Log(double *pdSamples, int8_t n) {
    uint16_t i;
    for(i=0; i<n; i++) {
        if(pdSamples[i]>=1.0)
            pdSamples[i]=20.0*log10(pdSamples[i]);
        else     
            pdSamples[i]=0.0;
    }
  }

*/