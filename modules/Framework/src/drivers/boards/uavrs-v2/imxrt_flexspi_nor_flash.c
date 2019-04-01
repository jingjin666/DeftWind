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
		.tag = FLEXSPI_CFG_BLK_TAG,/*标志：FCFB*/
		.version = FLEXSPI_CFG_BLK_VERSION,/*版本：V1.4.0*/
		.read_sample_clksrc = FLASH_READ_SAMPLE_CLK_LOOPBACK_INTERNELLY,/*内部环回*/
		.cs_hold_time = 3u, /*保持时间*/
		.cs_setup_time = 3u,/*建立时间*/
		.column_address_width = 0u,/*列地址宽度*/
		.device_mode_cfg_enable = 1u,/*设备模式配置使能*/
		.device_mode_type = 1u,/*Quad 使能命令*/
		.device_mode_seq.reserved = 0u,
		.device_mode_seq.seq_num = 1u,/*LUT序列号*/
		.device_mode_seq.seq_id = 4u, /*LUT序列索引*/
		.device_mode_arg = 0x000200,/*设置 QE=1（S9）*/
		.device_type = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,/*设备类型为nor flash*/
		.sflash_pad_type = SERIAL_FLASH_4PADS,/*设备数据总线为4*/
		.serial_clk_freq = FLEXSPI_SERIAL_CLKFREQ_133MHz,/*flash 时钟*/
		.sflash_a1size = 32u * 1024u * 1024u,	  /*flash 大小32MBytes*/
		//.data_valid_time = {16u, 16u},
		.lookup_table =
			{
				/*快速读命令（四线）*/
				[0] 	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xEB, RADDR_SDR, FLEXSPI_4PAD, 0x18),
				[1] 	= FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x06, READ_SDR, FLEXSPI_4PAD, 0x04),
				

				/*读状态命令*/
				[1*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x04),
				/*写使能命令*/
				[3*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0),		
				/*擦除扇区命令*/
				[5*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x20, RADDR_SDR, FLEXSPI_1PAD, 0x04),
				/*页编程命令（四线）*/
				[9*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x32, RADDR_SDR, FLEXSPI_1PAD, 0x18),	
				[9*4+1] = FLEXSPI_LUT_SEQ(WRITE_SDR,FLEXSPI_4PAD , 0x04, STOP, FLEXSPI_1PAD, 0),				  
				/*整片擦除*/
				[11*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xc7, STOP, FLEXSPI_1PAD, 0),					
			},
	},
	.page_size = 256u,/*页大小为256字节*/
	.sector_size = 4u * 1024u,/*扇区大小为4k字节*/
};
