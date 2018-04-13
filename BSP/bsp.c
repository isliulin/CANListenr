#include "includes.h"

bool g_bComPcDmaBusy = false;       //DMAæ��־������DMA��������ж�ʱ�˱�־�ü�
DMA_InitTypeDef DMA_InitStructure;
uint8_t  g_u8DataToPC[DataToPcBuffSize] = {0};
uint32_t g_u32DataToPcSave = 0;
uint32_t g_u32DataToPcGet = 0;
uint32_t g_u32SendLen = 0;          //����һ��DMA����ʱ���ô�ֵ��������ɺ󸽼ӵ�ȡָ��g_u32DataToPcGet

uint32_t g_u32SysTime = 0;

uint8_t MKCMDbuf[MAXPCRXBUF];
uint8_t PcRxBuf[4][MAXPCRXBUF];
uint8_t *pPcRx;
uint16_t pcrxcnt=0,pcbufput,pcbufget;

const uint32_t g_u32BaudList[] = 
{
    CAN_B500K, CAN_B250K,  CAN_B125K,
    CAN_B100K, CAN_B1000K,  CAN_B83_3K,
    CAN_B50K,  10400,      9600
};


/*
****************************************************************************************************
 * ��    �ܣ���PCͨ�����ô���DMA����
 * �����������
 * �� �� ֵ����
 * ˵    ������
****************************************************************************************************
*/
void BSP_COM_PCDMACfg(void)
{
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1,ENABLE);

    /* USARTy_Tx_DMA_Channel (triggered by USARTy Tx event) Config */
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);//0x40013804;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC , ENABLE);
}

/*
****************************************************************************************************
 * ��    �ܣ���ʼ����PCͨ�����ô���
 * �����������
 * �� �� ֵ����
 * ˵    ������
 *           �����ʷ����֣�һ���Ǳ�׼�Ĵ��ڲ����ʣ�����921600��Լ����
 *           9216000,460800,230400��Ҳ����˵���������ܹ���921600����
****************************************************************************************************
*/
void BSP_COM_PCInit(void)
{
    uint32_t i = 0;
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    pcbufput=0;
    pcbufget=0;
    pPcRx=PcRxBuf[pcbufput];

    RCC_APB2PeriphClockCmd(BSP_COM_PCTxPinPortClk | BSP_COM_PCRxPinPortClk
                          |RCC_APB2Periph_AFIO, ENABLE);
    BSP_COM_PCClkEn();

    GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = BSP_COM_PCTxPin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BSP_COM_PCTxPinPort, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = BSP_COM_PCRxPin;
    GPIO_Init(BSP_COM_PCRxPinPort, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 1500000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(BSP_COM_PC, &USART_InitStructure);

    USART_ITConfig(BSP_COM_PC, USART_IT_RXNE, ENABLE);

    BSP_COM_PCDMACfg();
    USART_DMACmd(BSP_COM_PC, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(BSP_COM_PC, ENABLE);
    USART_ClearFlag(BSP_COM_PC, USART_FLAG_TC);

    for (i=0; i<0x10000; i++);
}

/*
****************************************************************************************************
 * ��    �ܣ�PC��������һ��DMA����
 * ���������Buff---->Buffer Addr
 *          Len----->Buffer Length
 * �� �� ֵ����
 * ˵    ������
****************************************************************************************************
*/
void BSP_COM_PCDmaSend(uint8_t *Buff, uint32_t Len)
{
    if (!g_bComPcDmaBusy)
    {
        g_bComPcDmaBusy = true;
        g_u32SendLen = Len;
        DMA_DeInit(DMA1_Channel4);
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Buff;
        DMA_InitStructure.DMA_BufferSize = Len;
        DMA_Init(DMA1_Channel4, &DMA_InitStructure);
        DMA_ITConfig(DMA1_Channel4, DMA_IT_TC , ENABLE);
        DMA_Cmd(DMA1_Channel4, ENABLE);
    }
}

/*
****************************************************************************************************
 * ��    �ܣ�PC���ڼ��һ���Ƿ���DMA��������
 * �����������
 * �� �� ֵ����
 * ˵    ������
****************************************************************************************************
*/
void BSP_COM_PCCheckSendData(void)
{
    uint32_t l_u32DataLen = 0;

    __disable_fault_irq();

    if (g_bComPcDmaBusy)
    {
        __enable_fault_irq();

        return;
    }

    if (g_u32DataToPcSave >= g_u32DataToPcGet)
    {
        l_u32DataLen = g_u32DataToPcSave - g_u32DataToPcGet;
    }
    else
    {
        l_u32DataLen = DataToPcBuffSize - (g_u32DataToPcGet - g_u32DataToPcSave);
    }

    if (l_u32DataLen > 0)
    {
        if (l_u32DataLen >= DataToPcBuffSize-g_u32DataToPcGet)
        {
            l_u32DataLen = DataToPcBuffSize-g_u32DataToPcGet;
        }

        BSP_COM_PCDmaSend(&g_u8DataToPC[g_u32DataToPcGet], l_u32DataLen);
    }

    __enable_fault_irq();
}

/*
****************************************************************************************************
 * ��    �ܣ���PC��������
 * ���������Buff---->Ҫ���͵�����
 *          Len----->Ҫ���͵����ݳ���
 * �� �� ֵ����
 * ˵    ������
****************************************************************************************************
*/
void BSP_COM_PCSendBuff(uint8_t *Buff, uint32_t Len)
{
    uint32_t l_u32BuffSize = 0;
    uint32_t l_u32TempDataLen = 0;

    l_u32TempDataLen = Len;

    __disable_fault_irq();

    if (Len > DataToPcBuffSize)
    {
        __enable_fault_irq();

        return;
    }

    while (1)
    {
        if (g_u32DataToPcSave >= g_u32DataToPcGet)
        {
            l_u32BuffSize = DataToPcBuffSize - (g_u32DataToPcSave - g_u32DataToPcGet);
        }
        else
        {
            l_u32BuffSize = g_u32DataToPcGet - g_u32DataToPcSave;
        }

        if (l_u32BuffSize > Len)
        {
            if (Len >= DataToPcBuffSize-g_u32DataToPcSave)
            {
                memcpy(&g_u8DataToPC[g_u32DataToPcSave], Buff, DataToPcBuffSize-g_u32DataToPcSave);
                l_u32TempDataLen -= DataToPcBuffSize-g_u32DataToPcSave;
                g_u32DataToPcSave = 0;
                memcpy(&g_u8DataToPC[g_u32DataToPcSave], &Buff[Len-l_u32TempDataLen], l_u32TempDataLen);
                g_u32DataToPcSave += l_u32TempDataLen;
            }
            else
            {
                memcpy(&g_u8DataToPC[g_u32DataToPcSave], Buff, Len);
                g_u32DataToPcSave += Len;
            }

            __enable_fault_irq();

            BSP_COM_PCCheckSendData();

            break;
        }
    }
}

void SendFrameToPc(uint8_t *pBuff, uint16_t len)
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint8_t  l_u8AddSum = 0;

    BSP_LEDOn();
    memset(MKCMDbuf, 0, MAXPCRXBUF);

    MKCMDbuf[0] = FrameHead;
    MKCMDbuf[1] = 0;
    MKCMDbuf[2] = 0;

    for (i=0,j=0; i<len; i++,j++)
    {
        l_u8AddSum += pBuff[i];

        if ((pBuff[i]!=0x7D)&&(pBuff[i]!=0x7E)&&(pBuff[i]!=0x7F))
        {
            MKCMDbuf[j+3] = pBuff[i];
        }
        else
        {
            switch (pBuff[i])
            {
                case 0x7D:
                    MKCMDbuf[j+3] = DecodeKeyWord;
                    j += 1;
                    MKCMDbuf[j+3] = DecodeValue_7D;
                    break;

                case 0x7E:
                    MKCMDbuf[j+3] = DecodeKeyWord;
                    j += 1;
                    MKCMDbuf[j+3] = DecodeValue_7E;
                    break;

                case 0x7F:
                    MKCMDbuf[j+3] = DecodeKeyWord;
                    j += 1;
                    MKCMDbuf[j+3] = DecodeValue_7F;
                    break;

                default:
                    break;
            }
        }
    }

    if (l_u8AddSum == 0x7D)
    {
        MKCMDbuf[j+3] = DecodeKeyWord;
        j++;
        MKCMDbuf[j+3] = DecodeValue_7D;
    }
    else if (l_u8AddSum == 0x7E)
    {
        MKCMDbuf[j+3] = DecodeKeyWord;
        j++;
        MKCMDbuf[j+3] = DecodeValue_7E;
    }
    else if (l_u8AddSum == 0x7F)
    {
        MKCMDbuf[j+3] = DecodeKeyWord;
        j++;
        MKCMDbuf[j+3] = DecodeValue_7F;
    }
    else
    {
        MKCMDbuf[j+3] = l_u8AddSum;
        j++;
    }

    MKCMDbuf[j+3] = FrameTail;
    MKCMDbuf[1] = ((((j)&0x03FF)>>6)<<4);
    MKCMDbuf[2] = ((j)&0x003F);


    BSP_COM_PCSendBuff(MKCMDbuf, j+4);
}

uint16_t GetFrameLen(uint8_t *pBuff)
{
    uint16_t l_u16DataPeriodLen = 0;

    if (pBuff[0] != FrameHead)
    {
        l_u16DataPeriodLen = 0;
    }
    else
    {
        if ((pBuff[1]&0x0F != 0)||(pBuff[2]&0xC0 != 0)) //���֡���ֽ��а����Ƿ�����λ
        {
            l_u16DataPeriodLen = 0;
        }
        else
        {
            l_u16DataPeriodLen = 0;
            l_u16DataPeriodLen |= ((pBuff[1]>>4)<<6);
            l_u16DataPeriodLen |= (pBuff[2]&0x3F);

            if ((l_u16DataPeriodLen>MAXPCRXBUF)||(l_u16DataPeriodLen<MINPCRXFRAME))   //֡���������֡����С����С֡��
            {
                l_u16DataPeriodLen = 0;
            }
        }
    }

    return 1+2+l_u16DataPeriodLen+1; //Э���г���ָ���������򳤶�(����У��λ)
}

void BSP_StartComRxTimeOutCtr(void)
{
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
    TIM_SetCompare4(TIM3, TIM3->CNT + 2*MAXPCBYTETIME);
    TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
}

void BSP_CANInit(uint32_t Baud)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    CAN_InitTypeDef   CAN_InitStructure;

    CAN_DeInit(CAN1);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, DISABLE);
    GPIO_PinRemapConfig(GPIO_Remap2_CAN1, DISABLE);
    GPIO_PinRemapConfig(GPIO_Remap_CAN2,  DISABLE);

    //CAN1�������
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //CAN1��������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*������*/
    CAN_InitStructure.CAN_SJW = Baud >> 24;
    CAN_InitStructure.CAN_BS1 = (Baud >> 16) & 0xff;
    CAN_InitStructure.CAN_BS2 = (Baud >> 8) & 0xff;
    CAN_InitStructure.CAN_Prescaler = Baud & 0xff; /*����ʹ�������޶���10K����,��ʵ��û����10K���µ�*/
    CAN_InitStructure.CAN_ABOM = ENABLE;           /*���Զ����߻ָ�*/
    CAN_InitStructure.CAN_TXFP = ENABLE;           /*�����ȷ�*/
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
//  CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
    CAN_Init(CAN1, &CAN_InitStructure);
}

/*
****************************************************************************************************
 * ��    �ܣ�CAN�����˲���
 * ���������ID1---->����ͨ����ID1
 *          ID2---->����ͨ����ID2
 *          Mask1-->����1
 *          Mask2-->����2
 * �� �� ֵ����
 * ˵    ��������ѡ��ͨ��ʱ����Ĺ���ѡ��������·CAN
****************************************************************************************************
*/
void SDK_OBD_CANFilterInit(uint8_t FilterNum, uint32_t ID, uint32_t Mask)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    CAN1->FA1R = 0; //�������й�����
    CAN_FilterInitStructure.CAN_FilterNumber = FilterNum;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;

    if (ID > 0x7FF) //��չ֡
    {
        CAN_FilterInitStructure.CAN_FilterIdHigh     = (((ID<<3)>>16)&0xFFFF);
        CAN_FilterInitStructure.CAN_FilterIdLow      = (((ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF);
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (((Mask<<3)>>16)&0xFFFF);
        CAN_FilterInitStructure.CAN_FilterMaskIdLow  = (((Mask<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF);
    }
    else    //��׼֡
    {
        CAN_FilterInitStructure.CAN_FilterIdHigh     = ((ID&0xFFFF)<<5);
        CAN_FilterInitStructure.CAN_FilterIdLow      = ((CAN_ID_STD | CAN_RTR_DATA)&0xFFFF);
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh =((Mask&0xFFFF)<<5);
        CAN_FilterInitStructure.CAN_FilterMaskIdLow  = ((CAN_ID_STD | CAN_RTR_DATA)&0xFFFF);
    }

    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
//    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
//    CAN1->FA1R &= ~( 0x1 << FilterNum );
//    CAN_FilterInitStructure.CAN_FilterNumber = FilterNum;
//    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
//    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
//    CAN_FilterInitStructure.CAN_FilterIdHigh =
//        ( ID > 0x7ff ) ? ( ( ( ID << 3 ) & 0xFFFF0000 ) >> 16 ) : ( ( ( ID << 21 ) & 0xffff0000 ) >> 16 ) ;
//    CAN_FilterInitStructure.CAN_FilterIdLow =
//        ( ID > 0x7ff ) ? ( ( ( ID << 3 ) | CAN_ID_EXT | CAN_RTR_DATA ) & 0xFFFF ) : ( ( ( ID << 21 ) | CAN_ID_STD | CAN_RTR_DATA ) & 0xffff );
//    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (Mask<<3)>>16;
//    CAN_FilterInitStructure.CAN_FilterMaskIdLow = Mask<<3;
//    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
//    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
//    CAN_FilterInit( &CAN_FilterInitStructure );
//   CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

void BSP_CAN_SendFrame(CanTxMsg *Msg)
{
    uint8_t  l_u8CanMailBox = CAN_TxStatus_NoMailBox;
    uint32_t i = 0;

    do
    {
        i++;
        l_u8CanMailBox = CAN_Transmit(CAN1, Msg);
    }   while ((i < 0xFFFFFF)&&(l_u8CanMailBox==CAN_TxStatus_NoMailBox));
}

/*
****************************************************************************************************
 * ��    �ܣ���ʱ�ö�ʱ����ʼ��
 * �����������
 * �� �� ֵ����
 * ˵    ������ʱ��ΪTIM3,4��ͨ�����ɶ�ʱn*1ms���6,553ms,�ú�������ʼ��Ϊ����ģʽ
****************************************************************************************************
*/
void BSP_TIMInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;      //������
    TIM_TimeBaseStructure.TIM_Prescaler = 47999;    //TIMER2CLK = SystemCoreClock / 48000=2K�������μ����Ӽ�ʱ��Ϊ0.5ms���������ʱ0.5ms*65535������=32767.5ms
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig( TIM3, TIM_OCPreload_Disable);

    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig( TIM3, TIM_OCPreload_Disable);

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);

    /* TIM3 enable counter */
    TIM_Cmd(TIM3, ENABLE);

    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
    TIM_SetCompare3(TIM3, TIM3->CNT + 2);
    TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
}

void BSP_DelayMs(uint32_t ms)
{
    while (ms > 0)
    {
        if (ms > 32767)
        {
            TIM_SetCompare2(TIM3, TIM3->CNT + 32767*2);
            ms -= 32767;
        }
        else
        {
            TIM_SetCompare2(TIM3, TIM3->CNT + ms*2);
            ms = 0;
        }

        TIM_ClearFlag(TIM3, TIM_FLAG_CC2);
        while (TIM_GetFlagStatus(TIM3, TIM_FLAG_CC2) == RESET);
        TIM_ClearFlag(TIM3, TIM_FLAG_CC2);
    }
}

bool g_bLedOn = false;
bool g_bLedOffEnough = true;

void BSP_LEDInit(void)
{
    uint32_t i = 0;
    GPIO_InitTypeDef GPIO_InitStructure;

    for (i=0; i<0x3FFFF; i++);  //�±���Ҫ����JTAG����Ƭ������ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure );

    LedOff();
    g_bLedOn = false;
}

void BSP_LEDOn(void)
{
    if ((!g_bLedOn) && (g_bLedOffEnough))
    {
        LedOn();
        g_bLedOn = true;
        g_bLedOffEnough = false;
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        TIM_SetCompare1(TIM3, TIM3->CNT + 100); //50ms
        TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
    }
}

void BSP_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

//    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0xA000);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

    NVIC_InitStructure.NVIC_IRQChannel = BSP_COM_PCIRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void SysTick_Handler(void)
{
    g_u32SysTime++;
}

void BSP_Init(void)
{
    BSP_NVIC_Config();
    BSP_TIMInit();
    BSP_COM_PCInit();
    BSP_LEDInit();
}

void BSP_COM_PCIrqHandle(void)
{
    uint16_t len = 0;
    uint16_t Data = 0;
    uint8_t *ptr=pPcRx;

    Data = USART_ReceiveData(BSP_COM_PC);//BSP_COM_PC->DR;

    if (Data == FrameHead)    //�յ�֡ͷ����һ֡
    {
        pcrxcnt = 0;
    }

    ptr[pcrxcnt++] = Data;

    if(pcrxcnt>MAXPCRXBUF)                         //�������֡��
    {
        pcrxcnt=0;
    }
    else if(pcrxcnt >= MINPCRXFRAME)               // ��ⳤ�Ⱥͽ���
    {
        len = GetFrameLen(ptr);

        if (len == 0)   //�Ƿ�֡
        {
            pcrxcnt=0;
        }

        if(pcrxcnt == len) //֡��������
        {
            BSP_LEDOn();

            pcrxcnt=0;

            //ʵ�ʴ���PC������������������У�Ϊ�˲������ݣ�������4��BUF
            pcbufput=(pcbufput+1)&3;
            pPcRx=PcRxBuf[pcbufput];
        }
        else
        {
            BSP_StartComRxTimeOutCtr();
        }
    }
    else
    {
        BSP_StartComRxTimeOutCtr();
    }
}

void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC4) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC4);

        if (g_u32DataToPcGet + g_u32SendLen >= DataToPcBuffSize)
        {
            g_u32DataToPcGet = g_u32DataToPcGet + g_u32SendLen - DataToPcBuffSize;
        }
        else
        {
            g_u32DataToPcGet += g_u32SendLen;
        }

        g_u32SendLen = 0;

        g_bComPcDmaBusy = false;

        BSP_COM_PCCheckSendData();
    }
}

uint8_t g_u8CanDataReportBuff[256] = {0x88};
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg l_stCanRxMsg;

    CAN_Receive(CAN1, CAN_FIFO0, &l_stCanRxMsg);

    memcpy(&g_u8CanDataReportBuff[1], &g_u32SysTime, 4);

    g_u8CanDataReportBuff[5] = 0;
    g_u8CanDataReportBuff[6] = l_stCanRxMsg.DLC;

    if (l_stCanRxMsg.IDE == CAN_Id_Standard)
    {
        g_u8CanDataReportBuff[7]  = l_stCanRxMsg.StdId >> 24;
        g_u8CanDataReportBuff[8]  = l_stCanRxMsg.StdId >> 16;
        g_u8CanDataReportBuff[9]  = l_stCanRxMsg.StdId >> 8;
        g_u8CanDataReportBuff[10] = l_stCanRxMsg.StdId;
    }
    else
    {
        g_u8CanDataReportBuff[7]   = l_stCanRxMsg.ExtId >> 24;
        g_u8CanDataReportBuff[8]   = l_stCanRxMsg.ExtId >> 16;
        g_u8CanDataReportBuff[9]   = l_stCanRxMsg.ExtId >> 8;
        g_u8CanDataReportBuff[10]  = l_stCanRxMsg.ExtId;
    }

    memcpy(&g_u8CanDataReportBuff[11], l_stCanRxMsg.Data, l_stCanRxMsg.DLC);

    SendFrameToPc(g_u8CanDataReportBuff, 19);
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)   
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        TIM_ITConfig(TIM3, TIM_IT_CC1,DISABLE);

        if (g_bLedOn)
        {
            LedOff();
            g_bLedOn = false;

            TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
            TIM_SetCompare1(TIM3, TIM3->CNT + 100); //50ms
            TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
        }
        else
        {
            g_bLedOffEnough = true;
        }
    }

//    if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
//    {
//        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
//        TIM_ITConfig(TIM3, TIM_IT_CC2,DISABLE);
//    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
        TIM_SetCompare3( TIM3, TIM3->CNT + 2 );
        g_u32SysTime++;
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
        TIM_ITConfig(TIM3, TIM_IT_CC4,DISABLE);
        pcrxcnt = 0;
    }
}




















