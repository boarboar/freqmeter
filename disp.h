
/*
#define CLOG_MSG_SZ 80
#define CLOG_Q_SZ 16
*/

#define DISPLAY_LEN_S4  16
#define DISPLAY_H_SZ  240
#define DISPLAY_V_SZ  320

#define D_FONT_SZ   4
#define D_FONT_H    26

#define D_FONT_S_SZ 2
#define D_FONT_S_H  16

class Display {  
  public:
    void Init();    
    /*
    void vAddLogMsg(const char *pucMsg=NULL);
    void vAddLogMsg(const char *pucMsg, int16_t i);
    void vAddLogMsg(const char *pucMsg1, int16_t i1, const char *pucMsg2, int16_t i2);
    //void vAddLogMsg(const char *pucMsg1, int16_t i1, int16_t i2, int16_t i3);
    void vAddLogMsg(const char *pucMsg1, int32_t i1, int32_t i2, int32_t i3);
    void vAddLogMsg(const char *pucMsg1, int32_t i1, int32_t i2, int32_t i3, int32_t i4);
    */
    //void Process();
    void ShowStatus(char *msg);
    void ShowData3(const int16_t d[3], int row=0);
    void ShowChart(const double *pdVals, int16_t nvals, 
      int16_t y=(320-256), int16_t h=256, int16_t h0=128, int16_t xlab=0);
    void ShowChart0(const double *pdVals, int16_t nvals, 
      int16_t y=(320-256), int16_t h=256, int16_t xlab=0);  
    void ShowChartPlus(const double *pdVals, int16_t nvals, 
      int16_t y=(320-256), int16_t h=256, int16_t xlab=0);
  protected:
    void BufLen();
    char out_buf[64];
    /*
  struct AMessage
  {
    char ucMessageID;
    char ucData[ CLOG_MSG_SZ ];
  };
  struct AMessage txMessage;
  struct AMessage rxMessage;
  QueueHandle_t xDispQueue;
  xSemaphoreHandle xDispFree;
  */
};


