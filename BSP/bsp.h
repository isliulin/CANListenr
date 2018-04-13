#ifndef __BSP_H_
#define __BSP_H_

#define BSP_COM_PC                  USART1
#define BSP_COM_PCIRQn              USART1_IRQn
#define BSP_COM_PCIrqHandle         USART1_IRQHandler

#define BSP_COM_PCTxPin             GPIO_Pin_6
#define BSP_COM_PCTxPinPort         GPIOB
#define BSP_COM_PCTxPinPortClk      RCC_APB2Periph_GPIOB

#define BSP_COM_PCRxPin             GPIO_Pin_7
#define BSP_COM_PCRxPinPort         GPIOB
#define BSP_COM_PCRxPinPortClk      RCC_APB2Periph_GPIOB

#define BSP_COM_PCClkEn()           RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

#define DataToPcBuffSize    4096
#define MAXPCRXBUF      1024
#define MINPCRXFRAME    4
#define MAXPCBYTETIME   50

#define FrameHead       0x7D
#define FrameTail       0x7E
#define DecodeKeyWord   0x7F    //×ªÒå×Ö·û
#define DecodeValue_7D  0x00
#define DecodeValue_7E  0x01
#define DecodeValue_7F  0x02

#define LedOn()     GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define LedOff()    GPIO_ResetBits(GPIOA, GPIO_Pin_15)

#define CAN_B500K	0x00070206
#define CAN_B250K	0x000D0208
#define CAN_B125K	0x000D0210
#define CAN_B100K	0x000F0212
#define CAN_B1000K	0x00040204
#define CAN_B83_3K	0x000a0510
#define CAN_B50K	0x00090420

//#define CAN_B500K           0x00090406
//#define CAN_B250K           0x0009040C
//#define CAN_B125K           0x00090418
//#define CAN_B100K           0x00090432
//#define CAN_B83_3K          0x000A0520
//#define CAN_B50K            0x00090340
//#define CAN_B33_3K          0x000A0550


extern bool g_bComPcDmaBusy;
extern DMA_InitTypeDef DMA_InitStructure;
extern uint8_t  g_u8DataToPC[];
extern uint32_t g_u32DataToPcSave;
extern uint32_t g_u32DataToPcGet;
extern uint32_t g_u32SendLen;

extern uint8_t *pPcRx;
extern uint8_t PcRxBuf[][MAXPCRXBUF];
extern uint8_t MKCMDbuf[];
extern uint16_t pcrxcnt,pcbufput,pcbufget;


extern const uint32_t g_u32BaudList[];


void BSP_COM_PCCheckSendData(void);

void BSP_COM_PCSendBuff(uint8_t *Buff, uint32_t Len);
void SendFrameToPc(uint8_t *pBuff, uint16_t len);

void BSP_DelayMs(uint32_t ms);
void BSP_Init(void);

void BSP_CANInit(uint32_t Baud);
void SDK_OBD_CANFilterInit(uint8_t FilterNum, uint32_t ID, uint32_t Mask);
void BSP_CAN_SendFrame(CanTxMsg *Msg);

void BSP_LEDOn(void);

uint16_t GetFrameLen(uint8_t *pBuff);


#endif
