
#define CLOG_MSG_SZ 80
#define CLOG_Q_SZ 16

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
    void Process();
  protected:
    char out_buf[256];
  struct AMessage
  {
    char ucMessageID;
    char ucData[ CLOG_MSG_SZ ];
  };
  struct AMessage txMessage;
  struct AMessage rxMessage;
  QueueHandle_t xDispQueue;
  xSemaphoreHandle xDispFree;
};


