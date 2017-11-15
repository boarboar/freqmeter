
#include <MapleFreeRTOS821.h>
#include <Wire.h>
#include <Adafruit_GFX_AS.h>    // Core graphics library, with extra fonts.
#include "Adafruit_ILI9341_STM_1.h" // STM32 DMA Hardware-specific library
#include <SPI.h>

#include "disp.h"
#include "log.h"
#include "mpu.h"


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
 * PA12     FT
 * PA15 PWM FT
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
#define TASK_DELAY_DISP 100
//#define TASK_DELAY_DISP 10000
#define TASK_DELAY_MPU 1   // 1kHz

Display xDisplay;
ComLogger xLogger;

boolean fMPUReady=false;

//int16_t fft_buf[64];

// FFT
// https://github.com/kosme/arduinoFFT
static const uint16_t FFT_SAMPLES = 64; //This value MUST ALWAYS be a power of 2
// with sampling at 1000 Hz, we get width 1000/2 = 500 Hz
// discrete of (1000/2) / (64/2) = 500/32 = 15 Hz
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];  

static void vSerialOutTask(void *pvParameters) {
    Serial.println("Serial Out Task started.");
    for (;;) {
       xLogger.Process();
       vTaskDelay(TASK_DELAY_LOG);
    }
}

static void vDispOutTask(void *pvParameters) {
    //tft.drawString("Task started!",20,20,4);
    //int16_t a[3];
    
    TestChart(20);
    vTaskDelay(1000);
    TestChart(50);
    vTaskDelay(1000);
    TestChart(100);

    boolean bSampReady=false;
    for (;;) {
        if(fMPUReady) {
          
            if(MpuDrv::Mpu.Acquire()) {
                bSampReady = MpuDrv::Mpu.FFT_SamplingReady();
                //MpuDrv::Mpu.getRawAccel(a);                
                MpuDrv::Mpu.Release();
                //xDisplay.ShowData3(a, 0);
            } 
            /*
            vTaskDelay(1);
            if(MpuDrv::Mpu.Acquire()) {
                MpuDrv::Mpu.getAccel(a);                
                MpuDrv::Mpu.Release();
                xDisplay.ShowData3(a, 1);
            } 
            */
            /*
            vTaskDelay(1);
            if(MpuDrv::Mpu.Acquire()) {
                MpuDrv::Mpu.getRawAccelMax(a);                
                MpuDrv::Mpu.Release();
                xDisplay.ShowData3(a, 2);
            } 
            */
       
            if(bSampReady) {
                // there is no sampling at the miment, so we can use the buffer for FFT
                xLogger.vAddLogMsg("Sampling ready:", FFT_SAMPLES);
                /*
                for(int i=0; i<FFT_SAMPLES; i++) {
                    xLogger.vAddLogMsg("S", i, "V", (int16_t)vReal[i]);
                    vTaskDelay(10);                              
                }*/
                uint32_t xRunTime=xTaskGetTickCount();
                xDisplay.ShowChart(vReal, FFT_SAMPLES, 320-256, 256, 128);    
                xLogger.vAddLogMsg("DT", (int16_t)(xTaskGetTickCount()-xRunTime));
                
                if(MpuDrv::Mpu.Acquire()) {
                    MpuDrv::Mpu.FFT_StartSampling();
                    MpuDrv::Mpu.Release();                
                }  
                
            }
        }
        else {                        
            xDisplay.ShowStatus("Warm up...");
        }
       vTaskDelay(TASK_DELAY_DISP);
    }
}

static void vIMU_Task(void *pvParameters) {
    int16_t mpu_res=0;    
    xLogger.vAddLogMsg("IMU Task started.");
    for (;;) { 
      vTaskDelay(TASK_DELAY_MPU); 
      if(MpuDrv::Mpu.Acquire()) {
        mpu_res = MpuDrv::Mpu.cycle_dt();               
        MpuDrv::Mpu.Release();
      } else continue;
      if(mpu_res==2) {
        // IMU settled
        fMPUReady=true;
        if(MpuDrv::Mpu.Acquire()) {
            MpuDrv::Mpu.FFT_StartSampling();               
            MpuDrv::Mpu.Release();
          }
        xLogger.vAddLogMsg("MPU Ready!");
        xDisplay.ShowStatus("Ready");
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
    /*
    TestChart(20);
    //delay(1000);
    TestChart(50);
    //delay(1000);
    TestChart(100);
    */

    Serial.println("Init Wire...");
    //Wire.begin(SCL_PIN, SDA_PIN);
    Wire.begin();
    MpuDrv::Mpu.init();
    MpuDrv::Mpu.FFT_SetSampling(vReal, FFT_SAMPLES);

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
                
    xTaskCreate(vIMU_Task,
                "TaskIMU",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3, // max
                NULL);
                            
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}

void TestChart(double signalFrequency) {
    // test Display
    /* Build raw data */
    //const double signalFrequency = 1000;
    //const double samplingFrequency = 5000;
    const double samplingFrequency = 1000;
    const uint8_t amplitude = 100;
    double cycles = (((FFT_SAMPLES-1) * signalFrequency) / samplingFrequency); //Number of signal cycles that the sampling will read
    for (uint16_t i = 0; i < FFT_SAMPLES; i++)
    {
        vReal[i] = int8_t((amplitude * (sin((i * (PI*2 * cycles)) / FFT_SAMPLES))) / 2.0);/* Build data with positive and negative values*/
        //vReal[i] = uint8_t((amplitude * (sin((i * (twoPi * cycles)) / samples) + 1.0)) / 2.0);/* Build data displaced on the Y axis to include only positive values*/
        //vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
    }
    xDisplay.ShowChart(vReal, FFT_SAMPLES, 320-256, 256, 128);
}
