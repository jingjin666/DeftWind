/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file test_flexcan.c
 * Tests main file, loads individual tests.
 *
 * @author User <mail@example.com>
 */


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <dp_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "tests.h"

#include <platforms/dp_micro_hal.h>
#include <arch/board/board.h>
#include "imxrt_periphclks.h"

#include "fsl_flexcan.h"

CAN_Type *g_can;

flexcan_frame_t can1_rxframe, can2_rxframe;
bool can1_rxcomplete = false, can2_rxcomplete = false;

#define RX_BUFFER_NUM (8)       //消息缓冲序号
#define TX_BUFFER_NUM (9)       //消息缓冲序号

uint32_t can_txid = 0x321, can_rxid = 0x123;

static int can_isr1(int irq, void *context, void *arg)
{
    //printf("can_isr1\n");
    if (FLEXCAN_GetMbStatusFlags(CAN1,1<<RX_BUFFER_NUM))    //判断CAN1的RX信息缓冲是否收到数据
    {
        FLEXCAN_ClearMbStatusFlags(CAN1,1<<RX_BUFFER_NUM);  //清除中断标志位
        FLEXCAN_ReadRxMb(CAN1,RX_BUFFER_NUM,&can1_rxframe); //读取数据
        can1_rxcomplete=true;                               //标记读取完成
    }
    return 0;
}

static int can_isr2(int irq, void *context, void *arg)
{
    //printf("can_isr2\n");
    if (FLEXCAN_GetMbStatusFlags(CAN2,1<<RX_BUFFER_NUM))    //判断CAN2的RX信息缓冲是否收到数据
    {
        FLEXCAN_ClearMbStatusFlags(CAN2,1<<RX_BUFFER_NUM);  //清除中断标志位
        FLEXCAN_ReadRxMb(CAN2,RX_BUFFER_NUM,&can2_rxframe); //读取数据
        can2_rxcomplete=true;                               //标记读取完成
    }
    return 0;
}

static int16_t flexcan_send_msg(CAN_Type *base, uint8_t *data, uint8_t len)
{
    uint16_t ret;
    flexcan_frame_t msg;

    msg.format = kFLEXCAN_FrameFormatStandard;
    msg.type = kFLEXCAN_FrameTypeData;
    msg.id=FLEXCAN_ID_STD(can_txid);
    msg.length=len;
    msg.dataByte0=data[0];
    msg.dataByte1=data[1];
    msg.dataByte2=data[2];
    msg.dataByte3=data[3];
    msg.dataByte4=data[4];
    msg.dataByte5=data[5];
    msg.dataByte6=data[6];
    msg.dataByte7=data[7];

    if(FLEXCAN_TransferSendBlocking(base, TX_BUFFER_NUM, &msg) == kStatus_Success) {
        printf("Send ok\n");
        ret = 0;//发送数据，阻塞传输
    } else {
        printf("Send error\n");
        ret = -1;
    }

    return ret;
}

static void flexcan_init(CAN_Type *can_, uint8_t mode, uint32_t bitrate)
{
    if(can_ == CAN1) {
        imxrt_clockall_can1();
        imxrt_clockall_can1_serial();
        
        dp_arch_configgpio(GPIO_CAN1_RX);
        dp_arch_configgpio(GPIO_CAN1_TX);
        
    	irq_attach(IMXRT_IRQ_CAN1, can_isr1, NULL);
        up_enable_irq(IMXRT_IRQ_CAN1);
    } else if(can_ == CAN2) {
        imxrt_clockall_can2();
        imxrt_clockall_can2_serial();

        dp_arch_configgpio(GPIO_CAN2_RX);
        dp_arch_configgpio(GPIO_CAN2_TX);

    	irq_attach(IMXRT_IRQ_CAN2, can_isr2, NULL);
        up_enable_irq(IMXRT_IRQ_CAN2);
    } else {
        printf("Invalied can num\n");
        return;
    }
    
    uint32_t mcrTemp;
    uint32_t canclk = 20000000;//20Mhz
    flexcan_rx_mb_config_t mb_config;
    flexcan_config_t can_config;

    FLEXCAN_GetDefaultConfig(&can_config);               //先配置为默认值 
    can_config.baudRate = bitrate;                       //波特率为500Kbit
    can_config.enableLoopBack = mode;                    //设置模式
    FLEXCAN_Init(can_, &can_config, canclk);             //初始化CAN

    FLEXCAN_SetRxMbGlobalMask(can_, FLEXCAN_RX_MB_STD_MASK(can_rxid,0,0));

    FLEXCAN_EnableMbInterrupts(can_, 1<<RX_BUFFER_NUM);  //使能RX消息缓冲中断

    /* Setup Rx Message Buffer. */
    mb_config.format = kFLEXCAN_FrameFormatStandard;      //标准帧
    mb_config.type = kFLEXCAN_FrameTypeData;              //数据帧
    mb_config.id = FLEXCAN_ID_STD(can_rxid);              //接收的ID
    FLEXCAN_SetRxMbConfig(can_, RX_BUFFER_NUM, &mb_config, true);

    /* Setup Tx Message Buffer. */
    FLEXCAN_SetTxMbConfig(can_, TX_BUFFER_NUM, true);
}

/****************************************************************************
 * Name: test_flexcan
 ****************************************************************************/

int test_flexcan(int argc, char *argv[])
{
	uint8_t can1_send_buf[8] = {0,1,2,3,4,5,6,7};
    uint8_t can2_send_buf[8] = {7,6,5,4,3,2,1,0};
    
	printf("Test Flexcan driver.\n");
	

	if(argc < 2) {
        printf("Invalid Param::Please tests flexcan 1|2.\n");
		return -1;
	}

	if(argv[1] != NULL) {
		if(strcmp(argv[1], "1") == 0) {
			printf("test can 1.\n");
            g_can = CAN1;
		} else if(strcmp(argv[1], "2") == 0) {
			printf("test can 2.\n");
            g_can = CAN2;
		} else {
			printf("Invalid Param::Please tests flexcan 1|2.\n");
			return -1;
		}
	} else {
		printf("Param error.\n");
		return -1;
	}

    flexcan_init(g_can, 0, 500000);

    for(int i = 0; i < 5000; i++) {
        flexcan_send_msg(g_can, can1_send_buf, 8);
        sleep(1);
    }
    
	return 0;
}
