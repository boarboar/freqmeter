#ifndef _UMP_MPU_H_
#define _UMP_MPU_H_

#include "MPU6050.h"
#include "helper_3dmath.h"

#define MPU_FAIL_CNT_SZ 4

class MpuDrv {
  public:
  static const int8_t ST_0=0;
  static const int8_t ST_FAIL=-10;
  static const int8_t ST_WUP=-12;
  static const int8_t ST_NOTCONV=-13;
  static const int8_t ST_READY=1;
  //static const int16_t QUAT_INIT_TOL=10; 
  //static const int16_t ACC_INIT_TOL=10;
  static const int16_t QUAT_INIT_TOL=16; 
  static const int16_t ACC_INIT_TOL=20;
  static const int16_t INIT_PERIOD_MIN=20;
  static const int16_t INIT_PERIOD_MAX=60;
  //enum FailReason {MPU_FAIL_NONE=0, MPU_FAIL_INIT=1, MPU_FAIL_NODATA=2, MPU_FAIL_FIFOOVFL=3, MPU_FAIL_FIFOTMO=4, MPU_FAIL_FIFOEXCESS=5, MPU_FAIL_CONVTMO=6, MPU_FAIL_INIT_OK=128};
  enum FailReason {MPU_FAIL_NONE=0, MPU_FAIL_INIT=1,  MPU_FAIL_CONVTMO=2, MPU_FAIL_CYCLE=3, MPU_EVENT_CONV_PROG=16, MPU_FAIL_INIT_OK=128};
  enum FailCountIdx { MPU_FAIL_NODATA_IDX=0, MPU_FAIL_FIFOOVFL_IDX=1, MPU_FAIL_FIFOTMO_IDX=2, MPU_FAIL_FIFOEXCESS_IDX=3 };
  // FFT
  //static const uint16_t FFT_SAMPLES = 64; //This value MUST ALWAYS be a power of 2
  // with sampling at 1000 Hz, we get width 1000/2 = 500 Hz
  // discrete of (1000/2) / (64/2) = 500/32 = 15 Hz
public:
  static MpuDrv Mpu; // singleton  
  //int16_t init(/*uint16_t sda, uint16_t sdl,*/ uint16_t intr);
  int16_t init();

  bool Acquire();
  void Release();
  int16_t cycle_dt();
  int16_t cycle(uint16_t dt);
  int8_t getStatus();
  uint8_t isDataReady();
  uint8_t isNeedReset();
  void needReset();
  void getAll(float* ypr, float* af, float* vf);  
  //void resetIntegrator();
  //void process();
  void copyAlarms();
  void flushAlarms();
  //float getYaw();
  void  getRawAccel(int16_t a[3]);
  //void  getRawAccelMax(int16_t a[3]);
  void  getAccel(int16_t a[3]);

  void FFT_SetSampling(double *dSamples, int8_t n);
  void FFT_StartSampling(); 
  boolean FFT_SamplingReady();
  int16_t FFT_GetOverTimeCount1();
  int16_t FFT_GetDataMissCount();
  int16_t FFT_GetDataSampCount();
  int16_t FFT_GetFIFOOvflCount();
  int16_t FFT_GetFIFOXcsCount();
  int16_t FFT_GetSampleTime();
  
protected:  
  MpuDrv();
  // MPU control/status vars
  MPU6050 mpu;
  //uint32_t start;
  int8_t dmpStatus; 
  uint8_t data_ready;
  uint8_t need_reset;
  uint8_t fail_cnt[MPU_FAIL_CNT_SZ];
  uint8_t fail_cnt_buf[MPU_FAIL_CNT_SZ];
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  uint32_t count;
  uint16_t conv_count;
  // measurements
  int16_t q16[4];         // [w, x, y, z]         quaternion container (int 16)
  VectorInt16 aa16;          // [x, y, z]            accel sensor measurements
  int16_t q16_0[4];         // [w, x, y, z]         quaternion container (int 16) - prev/base
  VectorInt16 aa16_0;          // [x, y, z]            accel sensor measurements - prev/base
  //VectorInt16 aa16_max;          // [x, y, z]            accel sensor measurements - max // test only
  //VectorInt16 daa16;          // [x, y, z]
  /*
  VectorFloat a0; // base world accel
  VectorFloat a;
  VectorFloat v;
  float ypr[3];
  */
  // FFT
  //double vReal[FFT_SAMPLES];
  //double vImag[FFT_SAMPLES];

  int8_t nSample, iSample;
  int16_t iOverTimeCount1;
  int16_t iDataMissCount;
  int16_t iFIFOOvflCount;
  int16_t iFIFOXcsCount;
  int16_t nSampleTime;
  
  double *pdSample;
  
  xSemaphoreHandle xIMUFree;
  TickType_t xLastWakeTime, xStart, xStartSample;
};

#endif /* _UMP_MPU_H_ */
