/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file otp.c
 * otp estimation
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author David "Buzz" Bussenschutt <davidbuzz@gmail.com>
 *
 */

#include <dp_config.h>
#include <board_config.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>  // memset
#include "conversions.h"
#include "otp.h"
#include "err.h"   // warnx 
#include <assert.h>


int val_read(void *dest, volatile const void *src, int bytes)
{

	int i;

	for (i = 0; i < bytes / 4; i++) {
		*(((volatile unsigned *)dest) + i) = *(((volatile unsigned *)src) + i);
	}

	return i * 4;
}


int write_otp(uint8_t id_type, uint32_t vid, uint32_t pid, char *signature)
{

	warnx("write_otp: PX4 / %02X / %02X / %02X  / ... etc  \n", id_type, vid, pid);

	int errors = 0;

	// descriptor
	if (F_write_byte(ADDR_OTP_START, 'P')) {
		errors++;
	}

	//  write the 'P' from PX4. to first byte in OTP
	if (F_write_byte(ADDR_OTP_START + 1, 'X')) {
		errors++;        //  write the 'P' from PX4. to first byte in OTP
	}

	if (F_write_byte(ADDR_OTP_START + 2, '4')) {
		errors++;
	}

	if (F_write_byte(ADDR_OTP_START + 3, '\0')) {
		errors++;
	}

	//id_type
	if (F_write_byte(ADDR_OTP_START + 4, id_type)) {
		errors++;
	}

	// vid and pid are 4 bytes each
	if (F_write_word(ADDR_OTP_START + 5, vid)) {
		errors++;
	}

	if (F_write_word(ADDR_OTP_START + 9, pid)) {
		errors++;
	}

	// leave some 19 bytes of space, and go to the next block...
	// then the auth sig starts
	for (int i = 0 ; i < 128 ; i++) {
		if (F_write_byte(ADDR_OTP_START + 32 + i, signature[i])) {
			errors++;
		}
	}

	return errors;
}

int lock_otp(int blocknum)
{
	return F_write_byte(ADDR_OTP_LOCK_START + blocknum, OTP_LOCK_LOCKED);
}



// COMPLETE, BUSY, or other flash error?
static int F_GetStatus(void)
{
	int fs = F_COMPLETE;

	if ((FLASH->status & F_BSY) == F_BSY) { fs = F_BUSY; } else {

		if ((FLASH->status & F_WRPERR) != (uint32_t)0x00) { fs = F_ERROR_WRP; } else {

			if ((FLASH->status & (uint32_t)0xEF) != (uint32_t)0x00) { fs = F_ERROR_PROGRAM; } else {

				if ((FLASH->status & F_OPERR) != (uint32_t)0x00) { fs = F_ERROR_OPERATION; } else {
					fs = F_COMPLETE;
				}
			}
		}
	}

	return fs;
}


// enable FLASH Registers
void F_unlock(void)
{
	if ((FLASH->control & F_CR_LOCK) != 0) {
		FLASH->key = F_KEY1;
		FLASH->key = F_KEY2;
	}
}

//  lock the FLASH Registers
void F_lock(void)
{
	FLASH->control |= F_CR_LOCK;
}

// flash write word.
int F_write_word(uint32_t Address, uint32_t Data)
{
	unsigned char octet[4] = {0, 0, 0, 0};

	int ret = 0;

	for (int i = 0; i < 4; i++) {
		octet[i] = (Data >> (i * 8)) & 0xFF;
		ret = F_write_byte(Address + i, octet[i]);
	}

	return ret;
}

// flash write byte
int F_write_byte(uint32_t Address, uint8_t Data)
{
	volatile int status = F_COMPLETE;

	//warnx("F_write_byte: %08X %02d", Address , Data ) ;

	//Check the parameters
	assert(IS_F_ADDRESS(Address));

	//Wait for FLASH operation to complete by polling on BUSY flag.
	status = F_GetStatus();

	while (status == F_BUSY) { status = F_GetStatus();}

	if (status == F_COMPLETE) {
		//if the previous operation is completed, proceed to program the new data
		FLASH->control &= CR_PSIZE_MASK;
		FLASH->control |= F_PSIZE_BYTE;
		FLASH->control |= F_CR_PG;

		*(volatile uint8_t *)Address = Data;

		//Wait for FLASH operation to complete by polling on BUSY flag.
		status = F_GetStatus();

		while (status == F_BUSY) { status = F_GetStatus();}

		//if the program operation is completed, disable the PG Bit
		FLASH->control &= (~F_CR_PG);
	}

	//Return the Program Status
	return !(status == F_COMPLETE);
}



