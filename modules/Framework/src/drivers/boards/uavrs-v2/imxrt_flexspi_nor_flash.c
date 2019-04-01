/****************************************************************************
 * config/imxrt1050-evk/src/imxrt_flexspi_nor_flash.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*******************************************************************************
 * Included Files
 ******************************************************************************/


#include "imxrt_flexspi_nor_flash.h"

/*******************************************************************************
 * Public Data
 ******************************************************************************/
__attribute__((section(".boot_hdr.conf")))
const struct flexspi_nor_config_s flash_config =
{
	.mem_config =
	{
		.tag = FLEXSPI_CFG_BLK_TAG,/*��־��FCFB*/
		.version = FLEXSPI_CFG_BLK_VERSION,/*�汾��V1.4.0*/
		.read_sample_clksrc = FLASH_READ_SAMPLE_CLK_LOOPBACK_INTERNELLY,/*�ڲ�����*/
		.cs_hold_time = 3u, /*����ʱ��*/
		.cs_setup_time = 3u,/*����ʱ��*/
		.column_address_width = 0u,/*�е�ַ���*/
		.device_mode_cfg_enable = 1u,/*�豸ģʽ����ʹ��*/
		.device_mode_type = 1u,/*Quad ʹ������*/
		.device_mode_seq.reserved = 0u,
		.device_mode_seq.seq_num = 1u,/*LUT���к�*/
		.device_mode_seq.seq_id = 4u, /*LUT��������*/
		.device_mode_arg = 0x000200,/*���� QE=1��S9��*/
		.device_type = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,/*�豸����Ϊnor flash*/
		.sflash_pad_type = SERIAL_FLASH_4PADS,/*�豸��������Ϊ4*/
		.serial_clk_freq = FLEXSPI_SERIAL_CLKFREQ_133MHz,/*flash ʱ��*/
		.sflash_a1size = 32u * 1024u * 1024u,	  /*flash ��С32MBytes*/
		//.data_valid_time = {16u, 16u},
		.lookup_table =
			{
				/*���ٶ�������ߣ�*/
				[0] 	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xEB, RADDR_SDR, FLEXSPI_4PAD, 0x18),
				[1] 	= FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x06, READ_SDR, FLEXSPI_4PAD, 0x04),
				

				/*��״̬����*/
				[1*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x04),
				/*дʹ������*/
				[3*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0),		
				/*������������*/
				[5*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x20, RADDR_SDR, FLEXSPI_1PAD, 0x04),
				/*ҳ���������ߣ�*/
				[9*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x32, RADDR_SDR, FLEXSPI_1PAD, 0x18),	
				[9*4+1] = FLEXSPI_LUT_SEQ(WRITE_SDR,FLEXSPI_4PAD , 0x04, STOP, FLEXSPI_1PAD, 0),				  
				/*��Ƭ����*/
				[11*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xc7, STOP, FLEXSPI_1PAD, 0),					
			},
	},
	.page_size = 256u,/*ҳ��СΪ256�ֽ�*/
	.sector_size = 4u * 1024u,/*������СΪ4k�ֽ�*/
};
