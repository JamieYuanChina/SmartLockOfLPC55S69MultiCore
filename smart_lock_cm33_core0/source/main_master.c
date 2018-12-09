/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rpmsg_lite.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "mcmgr.h"

#include "fsl_common.h"
#include "pin_mux.h"
#include "fsl_usart.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "Common.h"
#include "MqttKit.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_USART USART0
#define DEMO_USART_CLK_SRC kCLOCK_Flexcomm0
#define DEMO_USART_CLK_FREQ CLOCK_GetFreq(kCLOCK_Flexcomm0)
#define DEMO_USART_IRQHandler FLEXCOMM0_IRQHandler
#define DEMO_USART_IRQn FLEXCOMM0_IRQn

#define APP_SW_PORT BOARD_SW1_GPIO_PORT
#define APP_SW_PIN  BOARD_SW1_GPIO_PIN
   
#define ESP8266_WIFI_INFO	"AT+CWJAP=\"dell2330\",\"dadalula\"\r\n"
#define ESP8266_ONENET_INFO	"AT+CIPSTART=\"TCP\",\"183.230.40.39\",6002\r\n"

#define PROID		"141215"
#define AUTH_INFO	"dpsO9ruH0aTZublG9g5SvBtFSEQ="
#define DEVID		"31451353"
   
#define REV_OK		0	// Receive completion flags
#define REV_WAIT	1	// Receive incomplete flags
   
/*! @brief Ring buffer size (Unit: Byte). */
#define DEMO_RING_BUFFER_SIZE 160   //DEBUG
#define FLEXCOMM1_BUFFER_SIZE 128   //Wifi
#define FLEXCOMM2_BUFFER_SIZE 200   //BT
#define FLEXCOMM7_BUFFER_SIZE 100   //LCD

#define RPMSG_LITE_LINK_ID (RL_PLATFORM_LPC55S69_M33_M33_LINK_ID)

/* Address of RAM, where the image for core1 should be copied */
#define CORE1_BOOT_ADDRESS (void *)0x20033000

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern uint32_t Image$$CORE1_REGION$$Base;
extern uint32_t Image$$CORE1_REGION$$Length;
#define CORE1_IMAGE_START &Image$$CORE1_REGION$$Base
#elif defined(__ICCARM__)
extern unsigned char core1_image_start[];
#define CORE1_IMAGE_START core1_image_start
#elif defined(__GNUC__)
extern const char m0_image_start[];
extern const char *m0_image_end;
extern int m0_image_size;
#define CORE1_IMAGE_START ((void *)m0_image_start)
#define CORE1_IMAGE_SIZE ((void *)m0_image_size)
#endif
#define REMOTE_EPT_ADDR (30)
#define LOCAL_EPT_ADDR (40)
#define APP_RPMSG_READY_EVENT_DATA (1)
#define APP_RPMSG_EP_READY_EVENT_DATA (2)

typedef struct the_message
{
    uint32_t DATA;
} THE_MESSAGE, *THE_MESSAGE_PTR;

#define SH_MEM_TOTAL_SIZE (6144)
#if defined(__ICCARM__) /* IAR Workbench */
#pragma location = "rpmsg_sh_mem_section"
char rpmsg_lite_base[SH_MEM_TOTAL_SIZE];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION) /* Keil MDK */
char rpmsg_lite_base[SH_MEM_TOTAL_SIZE] __attribute__((section("rpmsg_sh_mem_section")));
#elif defined(__GNUC__) /* LPCXpresso */
char rpmsg_lite_base[SH_MEM_TOTAL_SIZE] __attribute__((section(".noinit.$rpmsg_sh_mem")));
#else
#error "RPMsg: Please provide your definition of rpmsg_lite_base[]!"
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void);
#endif
void delayms(uint32_t nms);
void connectWifi(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_tipString[]    = "Smart Lock System is working!\r\n";
uint8_t g_liveString[]   = "smart_lock runing!\r\n";
uint8_t g_fc2String[]    = "fc2 runing!\r\n";
uint8_t g_fc7String[]    = "fc7 runing!\r\n";
uint8_t g_lockString[10] = "  Lock!\r\n";

uint8_t g_hmiCtrcmd[12] ={0x70 ,0x30 ,0x2E ,0x70 ,0x69 ,0x63 ,0x3D ,0x31 ,0xff ,0xff ,0xff,0x00}; 
uint8_t g_debugbuff[200];  // fc1 send buffer

unsigned char  esp8266_buf[FLEXCOMM1_BUFFER_SIZE];
unsigned short esp8266_cnt = 0, esp8266_cntPre = 0;


uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
uint8_t flexComm2Buffer[FLEXCOMM2_BUFFER_SIZE];
uint8_t flexComm7Buffer[FLEXCOMM7_BUFFER_SIZE];

volatile uint16_t fc2Index;
volatile uint16_t fc7Index;
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */
volatile uint16_t lockState = 1; // Lock state, 1 is unlock , LED lights out, 0 is lock state, LED lights up
volatile uint16_t syncFlag  = 0; // Synchronization state, need synchronization at 1, need not synchronization at 0

/*******************************************************************************
 * Code
 ******************************************************************************/

#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void)
{
    uint32_t core1_image_size;
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
    core1_image_size = (uint32_t)&Image$$CORE1_REGION$$Length;
#elif defined(__ICCARM__)
#pragma section = "__sec_core"
    core1_image_size = (uint32_t)__section_end("__sec_core") - (uint32_t)&core1_image_start;
#elif defined(__GNUC__)
    core1_image_size = (uint32_t)m0_image_size;
#endif
    return core1_image_size;
}
#endif
THE_MESSAGE volatile msg = {0};

/* This is the read callback, note we are in a task context when this callback
is invoked, so kernel primitives can be used freely */
static int my_ept_read_cb(void *payload, int payload_len, unsigned long src, void *priv)
{
    int *has_received = priv;

    if (payload_len <= sizeof(THE_MESSAGE))
    {
        memcpy((void *)&msg, payload, payload_len);
        *has_received = 1;
    }
    //PRINTF("Primary core received a msg\r\n");
    //PRINTF("Message: Size=%x, DATA = %i\r\n", payload_len, msg.DATA);
    return RL_RELEASE;
}

static void RPMsgRemoteReadyEventHandler(uint16_t eventData, void *context)
{
    uint16_t *data = (uint16_t *)context;

    *data = eventData;
}


void ESP8266_Clear(void)
{
    memset(esp8266_buf, 0, sizeof(esp8266_buf));
    esp8266_cnt = 0;
}

//==========================================================
//	�������ƣ�	ESP8266_WaitRecive
//
//	�������ܣ�	�ȴ��������
//
//	��ڲ�����	��
//
//	���ز�����	REV_OK-�������		REV_WAIT-���ճ�ʱδ���
//
//	˵����		ѭ�����ü���Ƿ�������
//==========================================================
_Bool ESP8266_WaitRecive(void)
{
    if(esp8266_cnt == 0) 					//������ռ���Ϊ0 ��˵��û�д��ڽ��������У�����ֱ����������������
        return REV_WAIT;
		
    if(esp8266_cnt == esp8266_cntPre)				//�����һ�ε�ֵ�������ͬ����˵���������
    {
        esp8266_cnt = 0;					//��0���ռ���
        return REV_OK;						//���ؽ�����ɱ�־
    }	
    esp8266_cntPre = esp8266_cnt;				//��Ϊ��ͬ
    return REV_WAIT;						//���ؽ���δ��ɱ�־
}

//==========================================================
//	�������ƣ�	ESP8266_SendCmd
//
//	�������ܣ�	��������
//
//	��ڲ�����	cmd������
//			res����Ҫ���ķ���ָ��
//
//	���ز�����	0-�ɹ�	1-ʧ��
//
//	˵����		
//==========================================================
_Bool ESP8266_SendCmd(char *cmd, char *res)
{
    unsigned char timeOut = 200;

    USART_WriteBlocking(USART1, (unsigned char *)cmd, strlen((const char *)cmd));

    while(timeOut--)
    {
        if(ESP8266_WaitRecive() == REV_OK)			//����յ�����
	{
            if(strstr((const char *)esp8266_buf, res) != NULL)	//����������ؼ���
            {
                ESP8266_Clear();				//��ջ���
                return 0;
            }
        }		
        delayms(10);
    }
    return 1;
}

//==========================================================
//	�������ƣ�	ESP8266_SendData
//
//	�������ܣ�	��������
//
//	��ڲ�����	data������
//			len������
//
//	���ز�����	��
//
//	˵����		
//==========================================================
void ESP8266_SendData(unsigned char *data, unsigned short len)
{
    char cmdBuf[32];

    ESP8266_Clear();					//��ս��ջ���
    sprintf(cmdBuf, "AT+CIPSEND=%d\r\n", len);		//��������
    if(!ESP8266_SendCmd(cmdBuf, ">"))			//�յ���>��ʱ���Է�������
    {
        USART_WriteBlocking(USART1, data, len);		//�����豸������������
    }
}

//==========================================================
//	�������ƣ�	ESP8266_GetIPD
//
//	�������ܣ�	��ȡƽ̨���ص�����
//
//	��ڲ�����	�ȴ���ʱ��(����10ms)
//
//	���ز�����	ƽ̨���ص�ԭʼ����
//
//	˵����		��ͬ�����豸���صĸ�ʽ��ͬ����Ҫȥ����
//			��ESP8266�ķ��ظ�ʽΪ	"+IPD,x:yyy"	x�������ݳ��ȣ�yyy����������
//==========================================================
unsigned char *ESP8266_GetIPD(unsigned short timeOut)
{
    char *ptrIPD = NULL;
	
    do
    {
        if(ESP8266_WaitRecive() == REV_OK)				//����������
        {
            ptrIPD = strstr((char *)esp8266_buf, "IPD,");		//������IPD��ͷ
            if(ptrIPD == NULL)						//���û�ҵ���������IPDͷ���ӳ٣�������Ҫ�ȴ�һ�ᣬ�����ᳬ���趨��ʱ��
            {
                //UsartPrintf(USART_DEBUG, "\"IPD\" not found\r\n");
            }
            else
            {
                ptrIPD = strchr(ptrIPD, ':');				//�ҵ�':'
                if(ptrIPD != NULL)
                {
                    ptrIPD++;
                    return (unsigned char *)(ptrIPD);
                }
                else
                return NULL;	
            }
        }
    delayms(5);								//��ʱ�ȴ�
    } while(timeOut--);

    return NULL;							//��ʱ��δ�ҵ������ؿ�ָ��
}


//==========================================================
//	�������ƣ�	OneNet_DevLink
//
//	�������ܣ�	��onenet��������
//
//	��ڲ�����	��
//
//	���ز�����	1-�ɹ�	0-ʧ��
//
//	˵����		��onenetƽ̨��������
//==========================================================
_Bool OneNet_DevLink(void)
{
    MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};                //Э���

    unsigned char *dataPtr;
    
    _Bool status = 1;
    memset(g_debugbuff,0x00,200);
    sprintf((char *)g_debugbuff,"OneNet_DevLink\r\nPROID: %s,	AUIF: %s,	DEVID:%s\r\n", PROID, AUTH_INFO, DEVID);
    USART_WriteBlocking(DEMO_USART,g_debugbuff,strlen((char *)g_debugbuff));       

    memset(g_debugbuff,0x00,200);
    if(MQTT_PacketConnect(PROID, AUTH_INFO, DEVID, 256, 0, MQTT_QOS_LEVEL0, NULL, NULL, 0, &mqttPacket) == 0)
    {
        ESP8266_SendData(mqttPacket._data, mqttPacket._len);			//�ϴ�ƽ̨

        dataPtr = ESP8266_GetIPD(250);						//�ȴ�ƽ̨��Ӧ
        if(dataPtr != NULL)
        {
            if(MQTT_UnPacketRecv(dataPtr) == MQTT_PKT_CONNACK)
            {
                switch(MQTT_UnPacketConnectAck(dataPtr))
                {
                    case 0:sprintf((char *)g_debugbuff,"Tips:	���ӳɹ�\r\n");status = 0;break;		
                    case 1:sprintf((char *)g_debugbuff,"WARN:	����ʧ�ܣ�Э�����\r\n");break;
                    case 2:sprintf((char *)g_debugbuff,"WARN:	����ʧ�ܣ��Ƿ���clientid\r\n");break;
                    case 3:sprintf((char *)g_debugbuff,"WARN:	����ʧ�ܣ�������ʧ��\r\n");break;
                    case 4:sprintf((char *)g_debugbuff,"WARN:	����ʧ�ܣ��û������������\r\n");break;
                    case 5:sprintf((char *)g_debugbuff,"WARN:	����ʧ�ܣ��Ƿ�����(����token�Ƿ�)\r\n");break;

                    default:sprintf((char *)g_debugbuff,"ERR:	����ʧ�ܣ�δ֪����\r\n");break;
                }
                USART_WriteBlocking(DEMO_USART,g_debugbuff,strlen((char *)g_debugbuff));    
            }
        }
        MQTT_DeleteBuffer(&mqttPacket);						//ɾ��
    }
    else
    {
        sprintf((char *)g_debugbuff,"WARN:	MQTT_PacketConnect Failed\r\n");
        USART_WriteBlocking(DEMO_USART,g_debugbuff,strlen((char *)g_debugbuff));
    }
    return status;
}

unsigned char OneNet_FillBuf(char *buf)
{
    char text[16];

    memset(text, 0, sizeof(text));

    strcpy(buf, "{");

    memset(text, 0, sizeof(text));
    sprintf(text, "\"Red_Led\":%d,", lockState);
    strcat(buf, text);

    strcat(buf, "}");

    return strlen(buf);
}

//==========================================================
//	�������ƣ�	OneNet_SendData
//
//	�������ܣ�	�ϴ����ݵ�ƽ̨
//
//	��ڲ�����	type���������ݵĸ�ʽ
//
//	���ز�����	��
//
//	˵����		
//==========================================================
void OneNet_SendData(void)
{
    MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};												//Э���
	
    char buf[128];
	
    short body_len = 0, i = 0;
    memset(g_debugbuff,0x00,200);
    sprintf((char *)g_debugbuff,"Tips:	OneNet_SendData-MQTT\r\n");
    USART_WriteBlocking(DEMO_USART,g_debugbuff,strlen((char *)g_debugbuff));
	
    memset(buf, 0, sizeof(buf));
	
    body_len = OneNet_FillBuf(buf);																	//��ȡ��ǰ��Ҫ���͵����������ܳ���

    if(body_len)
    {
        if(MQTT_PacketSaveData(DEVID, body_len, NULL, 3, &mqttPacket) == 0)							//���
        {
            for(; i < body_len; i++)
                mqttPacket._data[mqttPacket._len++] = buf[i];

            ESP8266_SendData(mqttPacket._data, mqttPacket._len);									//�ϴ����ݵ�ƽ̨
            memset(g_debugbuff,0x00,200);
            sprintf((char *)g_debugbuff,"Send %d Bytes\r\n", mqttPacket._len);
            USART_WriteBlocking(DEMO_USART,g_debugbuff,strlen((char *)g_debugbuff));	
            MQTT_DeleteBuffer(&mqttPacket);															//ɾ��
        }
        else
        {
            memset(g_debugbuff,0x00,200);
            sprintf((char *)g_debugbuff,"WARN:	EDP_NewBuffer Failed\r\n");
            USART_WriteBlocking(DEMO_USART,g_debugbuff,strlen((char *)g_debugbuff));
        }
    }
}

//==========================================================
//	�������ƣ�	OneNet_RevPro
//
//	�������ܣ�	ƽ̨�������ݼ��
//
//	��ڲ�����	dataPtr��ƽ̨���ص�����
//
//	���ز�����	��
//
//	˵����		
//==========================================================
void OneNet_RevPro(unsigned char *cmd)
{
    MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};			//Э���
	
    char *req_payload = NULL;
    char *cmdid_topic = NULL;
    unsigned char type = 0;
	
    short result = 0;

    char *dataPtr = NULL;
    char numBuf[10];
    int num = 0;
	
    type = MQTT_UnPacketRecv(cmd);
    switch(type)
    {
        case MQTT_PKT_CMD:															//�����·�
        result = MQTT_UnPacketCmd(cmd, &cmdid_topic, &req_payload);	//���topic����Ϣ��
        if(result == 0)
        {
            memset(g_debugbuff,0x00,200);
            sprintf((char *)g_debugbuff,"cmdid: %s, req: %s\r\n", cmdid_topic, req_payload);
            USART_WriteBlocking(DEMO_USART,g_debugbuff,strlen((char *)g_debugbuff));
            if(MQTT_PacketCmdResp(cmdid_topic, req_payload, &mqttPacket) == 0)	//����ظ����
            {
                memset(g_debugbuff,0x00,200);
                sprintf((char *)g_debugbuff,"Tips:	Send CmdResp\r\n");
                USART_WriteBlocking(DEMO_USART,g_debugbuff,strlen((char *)g_debugbuff));

                ESP8266_SendData(mqttPacket._data, mqttPacket._len);		//�ظ�����
                MQTT_DeleteBuffer(&mqttPacket);									//ɾ��
            }
        }
		
        break;	
        case MQTT_PKT_PUBACK:														//����Publish��Ϣ��ƽ̨�ظ���Ack
        if(MQTT_UnPacketPublishAck(cmd) == 0)
        {
             memset(g_debugbuff,0x00,200);
             sprintf((char *)g_debugbuff,"Tips:	MQTT Publish Send OK\r\n");
             USART_WriteBlocking(DEMO_USART,g_debugbuff,strlen((char *)g_debugbuff));
        }
			
        break;
		
        default:
        result = -1;
        break;
    }
	
    ESP8266_Clear();						//��ջ���
	
    if(result == -1)
        return;
	
    dataPtr = strchr(req_payload, ':');				//����':'

    if(dataPtr != NULL && result != -1)				//����ҵ���
    {
        dataPtr++;
	
        while(*dataPtr >= '0' && *dataPtr <= '9')		//�ж��Ƿ����·��������������
        {
            numBuf[num++] = *dataPtr++;
        }	
        num = atoi((const char *)numBuf);			//תΪ��ֵ��ʽ
	
        if(strstr((char *)req_payload, "redled"))		//����"redled"
        {
            if(num == 1)					//�����������Ϊ1������
            {
                //Led5_Set(LED_ON);
                lockState=0;
            }
            else if(num == 0)				//�����������Ϊ0�������
            {
                //Led5_Set(LED_OFF);
                lockState=1;
            }
            syncFlag=1;
        }
    }

    if(type == MQTT_PKT_CMD || type == MQTT_PKT_PUBLISH)
    {
        MQTT_FreeBuffer(cmdid_topic);
        MQTT_FreeBuffer(req_payload);
    }
}
void connectWifi(void)
{
    ESP8266_Clear();

    USART_WriteBlocking(DEMO_USART,"1. AT\r\n",7);
    while(ESP8266_SendCmd("AT\r\n", "OK"))
            delayms(500);
    
    USART_WriteBlocking(DEMO_USART, "2. CWJAP\r\n",10);
    while(ESP8266_SendCmd(ESP8266_WIFI_INFO, "GOT IP"))
            delayms(500);
    
    USART_WriteBlocking(DEMO_USART, "3. CIPSTART\r\n",13);
    while(ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT"))
            delayms(500);
    
    USART_WriteBlocking(DEMO_USART, "4. ESP8266 Init OK\r\n",20);

    
    while(OneNet_DevLink())			//����OneNET
	delayms(500);
}

void delayms(uint32_t nms)
{
    volatile uint32_t i = 0;
    uint32_t uTime=10000*nms;//10000 about 0.001 second
    for (i = 0; i < uTime; ++i)  
    {
        __asm("NOP"); /* delay */
    }
}

void DEMO_USART_IRQHandler(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(DEMO_USART))
    {
        data = USART_ReadByte(DEMO_USART);
        /* If ring buffer is not full, add data to ring buffer. */
        if ((rxIndex + 1) < DEMO_RING_BUFFER_SIZE) 
        {
            demoRingBuffer[rxIndex] = data;
            rxIndex++;
        }
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
void FLEXCOMM1_IRQHandler(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(USART1))
    {
        data = USART_ReadByte(USART1);

        if(esp8266_cnt >= sizeof(esp8266_buf))	esp8266_cnt = 0; //��ֹ���ڱ�ˢ��
          esp8266_buf[esp8266_cnt++] = data;
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
void FLEXCOMM2_IRQHandler(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(USART2))
    {
        data = USART_ReadByte(USART2);

        if ((fc2Index + 1) < FLEXCOMM2_BUFFER_SIZE)
        {
            flexComm2Buffer[fc2Index] = data;
            fc2Index++;
        }
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
void FLEXCOMM7_IRQHandler(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(USART7))
    {
        data = USART_ReadByte(USART7);

        if ((fc7Index + 1) < FLEXCOMM7_BUFFER_SIZE)
        {
            flexComm7Buffer[fc7Index] = data;
            fc7Index++;
        }
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
/*!
 * @brief Application-specific implementation of the SystemInitHook() weak function.
 */
void SystemInitHook(void)
{
    /* Initialize MCMGR - low level multicore management library. Call this
       function as close to the reset entry as possible to allow CoreUp event
       triggering. The SystemInitHook() weak function overloading is used in this
       application. */
    MCMGR_EarlyInit();
}

/*!
 * @brief Main function
 */
int main(void)
{
    volatile int has_received;
    volatile uint16_t RPMsgRemoteReadyEventData = 0;
    struct rpmsg_lite_ept_static_context my_ept_context;
    struct rpmsg_lite_endpoint *my_ept;
    struct rpmsg_lite_instance rpmsg_ctxt;
    struct rpmsg_lite_instance *my_rpmsg;
    
    uint32_t port_state = 0;
    uint32_t uiLoopCnt = 0;  //main loop count number
    usart_config_t config;
    unsigned char *dataPtr = NULL;

    /* Initialize standard SDK demo application pins */
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM1);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM7);
    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    
    BOARD_InitPins_Core0();
    BOARD_BootClockFROHF96M();
    BOARD_InitDebugConsole();

     /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUSART_ParityDisabled;
     * config.stopBitCount = kUSART_OneStopBit;
     * config.loopback = false;
     * config.enableTxFifo = false;
     * config.enableRxFifo = false;
     */
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    USART_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);
    USART_Init(USART1, &config, CLOCK_GetFreq(kCLOCK_Flexcomm1));
    config.baudRate_Bps = 9600;
    USART_Init(USART2, &config, CLOCK_GetFreq(kCLOCK_Flexcomm2));
    USART_Init(USART7, &config, CLOCK_GetFreq(kCLOCK_Flexcomm7));

    /* Send g_tipString out. */
    USART_WriteBlocking(DEMO_USART, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));
    //USART_WriteBlocking(USART1, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));
    //USART_WriteBlocking(USART2, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));
    //USART_WriteBlocking(USART7, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));

    /* Enable RX interrupt. */
    USART_EnableInterrupts(DEMO_USART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    EnableIRQ(DEMO_USART_IRQn);
       
    USART_EnableInterrupts(USART1, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    EnableIRQ(FLEXCOMM1_IRQn);
       
    USART_EnableInterrupts(USART2, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    EnableIRQ(FLEXCOMM2_IRQn);
       
    USART_EnableInterrupts(USART7, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    EnableIRQ(FLEXCOMM7_IRQn);

    /* Init SW GPIO PORT. */
    GPIO_PortInit(GPIO, APP_SW_PORT);


    /* Port masking */
    port_state = GPIO_PortRead(GPIO, APP_SW_PORT);
    PRINTF("\r\n Standard port read: %x\r\n", port_state);
    port_state = GPIO_PortMaskedRead(GPIO, APP_SW_PORT);
    PRINTF("\r\n Masked port read: %x\r\n", port_state);
    
#ifdef CORE1_IMAGE_COPY_TO_RAM
    /* Calculate size of the image */
    uint32_t core1_image_size;
    core1_image_size = get_core1_image_size();
    PRINTF("Copy CORE1 image to address: 0x%x, size: %d\r\n", CORE1_BOOT_ADDRESS, core1_image_size);

    /* Copy application from FLASH to RAM */
    memcpy(CORE1_BOOT_ADDRESS, (void *)CORE1_IMAGE_START, core1_image_size);
#endif

    /* Initialize MCMGR before calling its API */
    MCMGR_Init();

    /* Register the application event before starting the secondary core */
    MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent, RPMsgRemoteReadyEventHandler,
                        (void *)&RPMsgRemoteReadyEventData);

    /* Boot Secondary core application */
    MCMGR_StartCore(kMCMGR_Core1, CORE1_BOOT_ADDRESS, (uint32_t)rpmsg_lite_base, kMCMGR_Start_Synchronous);

    /* Print the initial banner */
    PRINTF("\r\nRPMsg demo starts\r\n");

    /* Wait until the secondary core application signals the rpmsg remote has been initialized and is ready to
     * communicate. */
    while (APP_RPMSG_READY_EVENT_DATA != RPMsgRemoteReadyEventData)
    {
    };

    my_rpmsg = rpmsg_lite_master_init(rpmsg_lite_base, SH_MEM_TOTAL_SIZE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS, &rpmsg_ctxt);

    my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, my_ept_read_cb, (void *)&has_received, &my_ept_context);

    has_received = 0;

    /* Wait until the secondary core application signals the rpmsg remote endpoint has been created. */
    while (APP_RPMSG_EP_READY_EVENT_DATA != RPMsgRemoteReadyEventData)
    {
    };

    /* Send the first message to the remoteproc */
    msg.DATA = 1;
    rpmsg_lite_send(my_rpmsg, my_ept, REMOTE_EPT_ADDR, (char *)&msg, sizeof(THE_MESSAGE), RL_DONT_BLOCK);
    
   //connectWifi();  //WIFI����OneNET�Ʒ�����
    
   while (1)//(msg.DATA <= 100)
    {
        if (has_received)  //������յ����ݣ����д���
        {
            //msg.DATA++;
            //rpmsg_lite_send(my_rpmsg, my_ept, REMOTE_EPT_ADDR, (char *)&msg, sizeof(THE_MESSAGE), RL_DONT_BLOCK);
        }
 
        uiLoopCnt++;  
        if(uiLoopCnt == 30000000) 
        {
          uiLoopCnt = 0;
        }
        
        //�����Դ����Ƿ��н��յ����ݣ�����н��д���
        if(rxIndex != 0)  //debug����
        {
          //����
          if(rxIndex==8)
          {
            if((demoRingBuffer[0]=='C')&&(demoRingBuffer[1]=='M')&&(demoRingBuffer[2]=='D')&&
               (demoRingBuffer[3]=='S')&&(demoRingBuffer[4]=='L')&&(demoRingBuffer[6]=='\r')&&
               (demoRingBuffer[7]=='\n'))
            {
              if(demoRingBuffer[5]=='0')
              {
                 lockState=1;
              }else if(demoRingBuffer[5]=='1')
              {
                 lockState=0;
              }
              syncFlag = 1;
            }
          }
          //������
          rxIndex = 0;
        }
        
        //������������Ƿ��н��յ����ݣ�����н��д���
        //���WIFI�����Ƿ��н��յ�����
        dataPtr = ESP8266_GetIPD(0);
	if(dataPtr != NULL)
            OneNet_RevPro(dataPtr);

        //������������Ƿ������ݱ�����
        if(fc2Index != 0)//��������
        {
          //����
          //USART_WriteBlocking(USART2, flexComm2Buffer, fc2Index);
          if(flexComm2Buffer[0]=='U'||flexComm2Buffer[0]=='u'){ //��Щ��U����Сд��u����
            lockState=1;
            //ͬ����־
            syncFlag = 1;
          }else if(flexComm2Buffer[0]=='L'||flexComm2Buffer[0]=='l')//��Щ��L����Сд��l����
          {
            lockState=0;
            //ͬ����־
            syncFlag = 1;
          }else
          {
            
          }
          //������
          fc2Index = 0;
        }
        
        //���LCD�����Ƿ������ݱ�����
        if(fc7Index != 0)//LCD����
        {
          //����
          if(flexComm7Buffer[0]==1){
            lockState=1;
          }else
          {
            lockState=0;
          }
          //ͬ����־
          syncFlag = 1;
          //������
          fc7Index = 0;
        }
        
        //����Ϊ��ʱ�������ݲ���
        //ÿһ����Դ��ڷ���һ���ַ�������ʾ��������������
        if(uiLoopCnt%100 == 0)
        {
          USART_WriteBlocking(DEMO_USART, g_liveString, sizeof(g_liveString) / sizeof(g_liveString[0]));
        }
        //ÿ���뷢��һ����������
        if(uiLoopCnt%200 == 0)
        {
            //����ģ�鲻��Ҫ��ʱ��������
        }
        //ÿ���뷢��һ��WIFI����
        if(uiLoopCnt%500 == 0)
        {
          OneNet_SendData();
          ESP8266_Clear();
        }
        //ÿ���뷢��һ��LCD����
        if(uiLoopCnt%800 == 0)
        {
          if(lockState == 1)
          {
            g_hmiCtrcmd[7] = 0x31;
          }else{
            g_hmiCtrcmd[7] = 0x32;
          }
          USART_WriteBlocking(USART7, g_hmiCtrcmd, 11);
        }

        //���²�ѯ��״̬�������и������Ʒ�ʽ��ͬ������
        //������״̬
        port_state = GPIO_PortRead(GPIO, APP_SW_PORT);
        if (!(port_state & (1 << APP_SW_PIN)))//�����鰴�������Կ�����״̬��
        {
          //lockState = 1;
        }else
        {
          //lockState = 0;
        }
        //������lockState������
        if (!lockState)
        {
            //setLock(0);
            msg.DATA = 2;
        }else
        {
            //setLock(1);
            msg.DATA = 1;
        }
        //ÿ0.5�뷢��һ�����ݸ�core1
        //if(uiLoopCnt%10 == 0)
        {
            rpmsg_lite_send(my_rpmsg, my_ept, REMOTE_EPT_ADDR, (char *)&msg, sizeof(THE_MESSAGE), RL_DONT_BLOCK);  
        }

        if(syncFlag==1) //�����Ҫͬ��
        {
          if(lockState==1)
          {
              g_hmiCtrcmd[7] = 0x31;
              g_lockString[0]='U';g_lockString[1]='n';

          }else
          {
              g_hmiCtrcmd[7] = 0x32;
              g_lockString[0]=' ';g_lockString[1]=' ';
          }
          //LCD����ͬ������
          USART_WriteBlocking(USART7, g_hmiCtrcmd, 11);
          //WIFI����ͬ������
          OneNet_SendData();
          ESP8266_Clear();
          //��������ͬ������
          USART_WriteBlocking(USART2, g_lockString, 9);
          //Debug���ڷ���ͬ������
          USART_WriteBlocking(DEMO_USART, g_lockString,9 );

          syncFlag=0;  //ͬ����ɣ��رձ��
        }
        delayms(10);
    }

    //rpmsg_lite_destroy_ept(my_rpmsg, my_ept);
    //my_ept = NULL;
    //rpmsg_lite_deinit(my_rpmsg);

    // Print the ending banner
    //PRINTF("\r\nRPMsg demo ends\r\n");
    //while (1)
    //    ;
}
