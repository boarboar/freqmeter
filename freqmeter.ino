
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

// PWM ports (check!)
#define DISPLAY_LED PA2 

// GPIO DISPLAY_LED
#define DISPLAY_CS  PA3
#define DISPLAY_DC  PA1
#define DISPLAY_RST PA4


#define TASK_DELAY_LOG 20
#define TASK_DELAY_DISP 200
#define TASK_DELAY_MPU 1

/*
#define WHITE 0xFFFF
#define BLACK 0x0000
#define RED 0xF800
*/

Display xDisplay;
ComLogger xLogger;
//Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM( DISPLAY_CS, DISPLAY_DC, DISPLAY_RST);  

static void vSerialOutTask(void *pvParameters) {
    Serial.println("Serial Out Task started.");
    for (;;) {
       xLogger.Process();
       vTaskDelay(TASK_DELAY_LOG);
    }
}

static void vDispOutTask(void *pvParameters) {
    //tft.drawString("Task started!",20,20,4);
    for (;;) {
       xDisplay.Process();
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
        xLogger.vAddLogMsg("MPU Ready!");
      }
    }
}


void setup() {
    digitalWrite(BOARD_LED_PIN, LOW);
    pinMode(BOARD_LED_PIN, OUTPUT);
    
    delay(5000);
    //disableDebugPorts(); // disable JTAG debug, enable PB3,4 for usage
    // NEVER do this!!!
/*
    pinMode(DISPLAY_LED, PWM);
    pwmWrite(DISPLAY_LED, 16000);

    tft.begin();
    tft.fillScreen(WHITE);
    tft.setTextColor(BLACK);  
    tft.setTextSize(1);
    tft.setTextColor(BLACK, WHITE);
    tft.drawString("Starting...",20,20,4);
    */

    xDisplay.Init();
    Serial.begin(115200); 
    Serial.print("Tick = ");
    Serial.println(portTICK_PERIOD_MS);
    
    xLogger.Init();
      
    Serial.println("Init Wire...");
    //Wire.begin(SCL_PIN, SDA_PIN);
    Wire.begin();
    MpuDrv::Mpu.init();
     
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


