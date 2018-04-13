#include "includes.h"

#define PCCmd_Dump          0x00
#define PCCmd_DataCollect   0xAA
#define PCCmd_CANExtSend    0x88

const uint8_t g_u8PCCmdRep_ACK[1] = {0x55};

uint8_t g_u8DataPeriod[MAXPCRXBUF] = {0};   //PC�·���������
typedef struct TYPE_BUSFRAMS
{
	 u8 onefram[15];
}TYPE_BUSFRAMS;

//��������صı���
TYPE_BUSFRAMS g_allbusfram[250];//������������
u16 g_busframetime = 2; //����֡���
u32 g_bustimes = 0;    //���ʹ���
u8 g_busnum = 0;       //����֡����

/*
****************************************************************************************************
 * ��    �ܣ�����PC�·����ݵ�������
 * ���������Ҫ���PC�·�������֡
 * �� �� ֵ����
 * ˵    ����ȥ��ת���ַ�
****************************************************************************************************
*/
void DecodePcDataPeriod(uint8_t *pBuff)
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t l_u16FrameLen = 0;

    l_u16FrameLen = GetFrameLen(pBuff);
	
    pBuff += 3; //��λ����������ʼ

    for (i=0, j=0; i<l_u16FrameLen;i++,j++)
    {
        if (pBuff[i] == DecodeKeyWord)
        {
            i += 1;
            switch (pBuff[i])
            {
                case DecodeValue_7D:
                    g_u8DataPeriod[j] = 0x7D;
                    break;

                case DecodeValue_7E:
                    g_u8DataPeriod[j] = 0x7E;
                    break;

                case DecodeValue_7F:
                    g_u8DataPeriod[j] = 0x7F;
                    break;
    
                default:
                    break;
            }
        }
        else
        {
            g_u8DataPeriod[j] = pBuff[i];
        }
    }
}

void PCCmdHandle_DataCollect(uint8_t *pBuff)
{
    uint8_t i = 0;
//    uint8_t l_u8CanChannelSelection = 0;
    uint8_t l_u8CanBaudIdx = 0;
    uint8_t l_u8FilterNum = 0;
    uint32_t l_u32Filter = 0;
    uint32_t l_u32Mask = 0;

//    l_u8CanChannelSelection = pBuff[1];
    l_u8CanBaudIdx = pBuff[2];
    l_u8FilterNum = pBuff[8];

    BSP_CANInit(g_u32BaudList[l_u8CanBaudIdx]);

    if (l_u8FilterNum == 0)
    {
        SDK_OBD_CANFilterInit(i, 0, 0);
    }
    else
    {
        for (i=0; ((i<l_u8FilterNum)&&(i<14)); i++)
        {
            l_u32Filter = *(uint32_t*)&pBuff[9+i*8];
//            l_u32Filter |= pBuff[9+i*8] << 24;
//            l_u32Filter |= pBuff[9+i*8+1] << 16;
//            l_u32Filter |= pBuff[9+i*8+2] << 8;
//            l_u32Filter |= pBuff[9+i*8+3];

            l_u32Mask = *(uint32_t*)&pBuff[9+i*8+4];
//            l_u32Mask |= pBuff[9+i*8+4] << 24;
//            l_u32Mask |= pBuff[9+i*8+5] << 16;
//            l_u32Mask |= pBuff[9+i*8+6] << 8;
//            l_u32Mask |= pBuff[9+i*8+7];

            SDK_OBD_CANFilterInit(i, l_u32Filter, l_u32Mask);
        }
    }

    SendFrameToPc((uint8_t *)g_u8PCCmdRep_ACK, sizeof(g_u8PCCmdRep_ACK));
}

void PCCmdHandle_CANExtSend(uint8_t *pBuff)
{
    uint16_t i = 0;
    CanTxMsg TxMessage;
    uint16_t l_u16DataPeriodLen = 0;
    uint16_t l_u16FrameNum = 0;
    uint32_t l_u32CanId = 0;

    l_u16DataPeriodLen = GetFrameLen(pBuff) - 6;    //��֡��-֡ͷ-֡β-����-����λ�����־-����λУ���

    l_u16FrameNum = l_u16DataPeriodLen/(13);    //Э���ĵ��У�DLC+ID+DATA=1+4+8

    for (i=0; i<l_u16FrameNum; i++)
    {
        TxMessage.DLC = pBuff[4+i*13];

        l_u32CanId = 0;
        l_u32CanId |= pBuff[4+i*13+1] << 24;
        l_u32CanId |= pBuff[4+i*13+2] << 16;
        l_u32CanId |= pBuff[4+i*13+3] << 8;
        l_u32CanId |= pBuff[4+i*13+4];
        if (l_u32CanId > 0x7FF)
        {
            TxMessage.ExtId = l_u32CanId;
            TxMessage.IDE = CAN_ID_EXT;
        }
        else
        {
            TxMessage.StdId = l_u32CanId;
            TxMessage.IDE = CAN_ID_STD;
        }

        TxMessage.RTR = 0;

        memcpy(TxMessage.Data, &pBuff[4+i*13+5], TxMessage.DLC);

        BSP_CAN_SendFrame(&TxMessage);

        BSP_DelayMs(5);     //Э���ĵ�����֡���
    }
}

/*
****************************************************************************************************
 * ��    �ܣ�PCָ���
 * ���������PC������
 * �� �� ֵ����
 * ˵    ��������������
****************************************************************************************************
*/
void PcCmdProcesser(uint8_t *pBuff)
{
    uint8_t l_u8PcCmd = PCCmd_Dump;

    l_u8PcCmd = pBuff[0];
     
    switch (l_u8PcCmd)
    {
        case PCCmd_DataCollect:     //CAN���ݲɼ�
            PCCmdHandle_DataCollect(pBuff);
            break;

        case PCCmd_CANExtSend:      //CAN��չ֡����
            PCCmdHandle_CANExtSend(pBuff);
            break;
				case 0xCC:
						{
								u16 i=0,j=0;
							  CanTxMsg TxMessage;
							  u8 sendpos = 0;
							  u16 dftime = 0;
							
								g_busframetime =  *(u16*)&pBuff[1] == 0 ? 1 :*(u16*)&pBuff[1]; //��Ҫ����0
								g_bustimes =  *(u16*)&pBuff[3];
								g_busnum = pBuff[7];
								if(g_busframetime == 0)
									g_busframetime =1;
								for(i=0;i!=pBuff[7];i++)
								{
									memcpy(g_allbusfram[i].onefram,&pBuff[8+i*15],15);
								}
							  for(i=0;i!=g_bustimes;i++)
								{
											for(sendpos=0;sendpos!=g_busnum;sendpos++)
											{
														memcpy( &TxMessage.Data, &g_allbusfram[sendpos].onefram[4], 8 );						
														TxMessage.StdId = g_allbusfram[sendpos].onefram[0]<<24 |
																							g_allbusfram[sendpos].onefram[1]<<16 |
																							g_allbusfram[sendpos].onefram[2]<<8  |
																							g_allbusfram[sendpos].onefram[3];
														TxMessage.ExtId = TxMessage.StdId;
														TxMessage.RTR = 0;
														TxMessage.IDE = ( TxMessage.StdId > 0X7FF ) ? 4 : 0;
														TxMessage.DLC = g_allbusfram[sendpos].onefram[12];	
														CAN_Transmit( CAN1, &TxMessage );
														dftime = *(u16*)&g_allbusfram[sendpos].onefram[13];
														BSP_DelayMs(dftime);
														if(sendpos==0xFF)
														{
															 sendpos = 0;
															 return ;
														}
											}
								}
								break;
						}
				case 0xDD:
						{
									 g_busframetime  = 10;
									g_bustimes = 0;
									g_busnum =0;
									break;							
						}
        default:
            break;
    }
}

uint32_t SendIdx = 0;
void CAN_Send(void)
{
    CanTxMsg TxMessage;

    TxMessage.Data[0] = SendIdx >> 24;
    TxMessage.Data[1] = SendIdx >> 16;
    TxMessage.Data[2] = SendIdx >> 8;
    TxMessage.Data[3] = SendIdx;
    TxMessage.Data[4] = SendIdx >> 24;
    TxMessage.Data[5] = SendIdx >> 16;
    TxMessage.Data[6] = SendIdx >> 8;
    TxMessage.Data[7] = SendIdx;
    TxMessage.DLC = 8;
    TxMessage.StdId = 0x7DF;
    TxMessage.RTR = 0;
    TxMessage.IDE = CAN_ID_STD;

    BSP_CAN_SendFrame(&TxMessage);
    SendIdx++;
//    BSP_DelayMs(5);     //Э���ĵ�����֡���
}

int main(void)
{
    BSP_Init();

//    BSP_CANInit(CAN_B500K);
//    BSP_DelayMs(50);

    while(1)
    {
        //�յ�PC����������Ĵ���
        if(pcbufput != pcbufget)
        {
            DecodePcDataPeriod(&PcRxBuf[pcbufget][0]);
            PcCmdProcesser(g_u8DataPeriod);
            pcbufget=(pcbufget+1)&3;
        }
//SendFrameToPc((uint8_t *)g_u8PCCmdRep_ACK, sizeof(g_u8PCCmdRep_ACK));
//BSP_DelayMs(1000);
//        CAN_Send();
    }
}











































