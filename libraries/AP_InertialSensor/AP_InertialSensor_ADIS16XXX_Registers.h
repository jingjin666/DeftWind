#pragma once

#define ADIS_WRITE_REG(reg) ((0x80 | (reg)))
#define ADIS_READ_REG(reg) ((reg) & 0x7f)

#define ADIS_FNCIO_CTRL_DATA_RDY_EN	BIT(3)
#define ADIS_FNCIO_CTRL_DATA_RDY_POL_HIGH	BIT(2)
#define ADIS_FNCIO_CTRL_DATA_RDY_DIO2	BIT(0)

#define ADIS_GLOB_CMD_SW_RESET		BIT(7)
#define ADIS_GLOB_CMD_TEST_MYSELF	BIT(1)
#define ADIS_GLOB_CMD_OFFSET_ZERO	BIT(0)

#define ADIS_REG_PAGE_ID 0x00

#define ADIS16XXX_PAGE_SIZE 0x80

#define ADIS16XXX_REG(page, reg) ((page) * ADIS16XXX_PAGE_SIZE + (reg))

#define ADIS16XXX_REG_PAGE_ID 0x00 /* Same address on each page */
#define ADIS16XXX_REG_SEQ_CNT			ADIS16XXX_REG(0x00, 0x06)
#define ADIS16XXX_REG_SYS_E_FLA			ADIS16XXX_REG(0x00, 0x08)
#define ADIS16XXX_REG_DIAG_STS			ADIS16XXX_REG(0x00, 0x0A)
#define ADIS16XXX_REG_ALM_STS			ADIS16XXX_REG(0x00, 0x0C)
#define ADIS16XXX_REG_TEMP_OUT			ADIS16XXX_REG(0x00, 0x0E)

#define ADIS16XXX_REG_X_GYRO_LOW		ADIS16XXX_REG(0x00, 0x10)
#define ADIS16XXX_REG_X_GYRO_OUT		ADIS16XXX_REG(0x00, 0x12)

#define ADIS16XXX_REG_Y_GYRO_LOW		ADIS16XXX_REG(0x00, 0x14)
#define ADIS16XXX_REG_Y_GYRO_OUT		ADIS16XXX_REG(0x00, 0x16)

#define ADIS16XXX_REG_Z_GYRO_LOW		ADIS16XXX_REG(0x00, 0x18)
#define ADIS16XXX_REG_Z_GYRO_OUT		ADIS16XXX_REG(0x00, 0x1A)

#define ADIS16XXX_REG_X_ACCEL_LOW		ADIS16XXX_REG(0x00, 0x1C)
#define ADIS16XXX_REG_X_ACCEL_OUT		ADIS16XXX_REG(0x00, 0x1E)

#define ADIS16XXX_REG_Y_ACCEL_LOW		ADIS16XXX_REG(0x00, 0x20)
#define ADIS16XXX_REG_Y_ACCEL_OUT		ADIS16XXX_REG(0x00, 0x22)

#define ADIS16XXX_REG_Z_ACCEL_LOW		ADIS16XXX_REG(0x00, 0x24)
#define ADIS16XXX_REG_Z_ACCEL_OUT		ADIS16XXX_REG(0x00, 0x26)

#define ADIS16XXX_REG_X_MAGN_OUT		ADIS16XXX_REG(0x00, 0x28)
#define ADIS16XXX_REG_Y_MAGN_OUT		ADIS16XXX_REG(0x00, 0x2A)
#define ADIS16XXX_REG_Z_MAGN_OUT		ADIS16XXX_REG(0x00, 0x2C)

#define ADIS16XXX_REG_BAROM_OUT			ADIS16XXX_REG(0x00, 0x2E)

#define ADIS16XXX_REG_X_DELTAANG_OUT	ADIS16XXX_REG(0x00, 0x40)
#define ADIS16XXX_REG_Y_DELTAANG_OUT	ADIS16XXX_REG(0x00, 0x44)
#define ADIS16XXX_REG_Z_DELTAANG_OUT	ADIS16XXX_REG(0x00, 0x48)

#define ADIS16XXX_REG_X_DELTAVEL_OUT	ADIS16XXX_REG(0x00, 0x4C)
#define ADIS16XXX_REG_Y_DELTAVEL_OUT	ADIS16XXX_REG(0x00, 0x50)
#define ADIS16XXX_REG_Z_DELTAVEL_OUT	ADIS16XXX_REG(0x00, 0x54)

#define ADIS16XXX_REG_PROD_ID			ADIS16XXX_REG(0x00, 0x7E)

#define ADIS16XXX_REG_X_GYRO_SCALE		ADIS16XXX_REG(0x02, 0x04)
#define ADIS16XXX_REG_Y_GYRO_SCALE		ADIS16XXX_REG(0x02, 0x06)
#define ADIS16XXX_REG_Z_GYRO_SCALE		ADIS16XXX_REG(0x02, 0x08)
#define ADIS16XXX_REG_X_ACCEL_SCALE		ADIS16XXX_REG(0x02, 0x0A)
#define ADIS16XXX_REG_Y_ACCEL_SCALE		ADIS16XXX_REG(0x02, 0x0C)
#define ADIS16XXX_REG_Z_ACCEL_SCALE		ADIS16XXX_REG(0x02, 0x0E)

#define ADIS16XXX_REG_X_GYRO_BIAS		ADIS16XXX_REG(0x02, 0x10)
#define ADIS16XXX_REG_Y_GYRO_BIAS		ADIS16XXX_REG(0x02, 0x14)
#define ADIS16XXX_REG_Z_GYRO_BIAS		ADIS16XXX_REG(0x02, 0x18)
#define ADIS16XXX_REG_X_ACCEL_BIAS		ADIS16XXX_REG(0x02, 0x1C)
#define ADIS16XXX_REG_Y_ACCEL_BIAS		ADIS16XXX_REG(0x02, 0x20)
#define ADIS16XXX_REG_Z_ACCEL_BIAS		ADIS16XXX_REG(0x02, 0x24)
#define ADIS16XXX_REG_X_HARD_IRON		ADIS16XXX_REG(0x02, 0x28)
#define ADIS16XXX_REG_Y_HARD_IRON		ADIS16XXX_REG(0x02, 0x2A)
#define ADIS16XXX_REG_Z_HARD_IRON		ADIS16XXX_REG(0x02, 0x2C)
#define ADIS16XXX_REG_BAROM_BIAS		ADIS16XXX_REG(0x02, 0x40)
#define ADIS16XXX_REG_FLASH_CNT			ADIS16XXX_REG(0x02, 0x7C)

#define ADIS16XXX_REG_GLOB_CMD			ADIS16XXX_REG(0x03, 0x02)
#define ADIS16XXX_REG_FNCTIO_CTRL		ADIS16XXX_REG(0x03, 0x06)
#define ADIS16XXX_REG_GPIO_CTRL			ADIS16XXX_REG(0x03, 0x08)
#define ADIS16XXX_REG_CONFIG			ADIS16XXX_REG(0x03, 0x0A)
#define ADIS16XXX_REG_DEC_RATE			ADIS16XXX_REG(0x03, 0x0C)
#define ADIS16XXX_REG_SLP_CNT			ADIS16XXX_REG(0x03, 0x10)
#define ADIS16XXX_REG_FILTER_BNK0		ADIS16XXX_REG(0x03, 0x16)
#define ADIS16XXX_REG_FILTER_BNK1		ADIS16XXX_REG(0x03, 0x18)
#define ADIS16XXX_REG_ALM_CNFG0			ADIS16XXX_REG(0x03, 0x20)
#define ADIS16XXX_REG_ALM_CNFG1			ADIS16XXX_REG(0x03, 0x22)
#define ADIS16XXX_REG_ALM_CNFG2			ADIS16XXX_REG(0x03, 0x24)
#define ADIS16XXX_REG_XG_ALM_MAGN		ADIS16XXX_REG(0x03, 0x28)
#define ADIS16XXX_REG_YG_ALM_MAGN		ADIS16XXX_REG(0x03, 0x2A)
#define ADIS16XXX_REG_ZG_ALM_MAGN		ADIS16XXX_REG(0x03, 0x2C)
#define ADIS16XXX_REG_XA_ALM_MAGN		ADIS16XXX_REG(0x03, 0x2E)
#define ADIS16XXX_REG_YA_ALM_MAGN		ADIS16XXX_REG(0x03, 0x30)
#define ADIS16XXX_REG_ZA_ALM_MAGN		ADIS16XXX_REG(0x03, 0x32)
#define ADIS16XXX_REG_XM_ALM_MAGN		ADIS16XXX_REG(0x03, 0x34)
#define ADIS16XXX_REG_YM_ALM_MAGN		ADIS16XXX_REG(0x03, 0x36)
#define ADIS16XXX_REG_ZM_ALM_MAGN		ADIS16XXX_REG(0x03, 0x38)
#define ADIS16XXX_REG_BR_ALM_MAGN		ADIS16XXX_REG(0x03, 0x3A)

#define ADIS16XXX_REG_FIRM_REV			ADIS16XXX_REG(0x03, 0x78)
#define ADIS16XXX_REG_FIRM_DM			ADIS16XXX_REG(0x03, 0x7A)
#define ADIS16XXX_REG_FIRM_Y			ADIS16XXX_REG(0x03, 0x7C)

#define ADIS16XXX_REG_SERIAL_NUM		ADIS16XXX_REG(0x04, 0x20)

/* Each filter coefficent bank spans two pages */
#define ADIS16XXX_FIR_COEF(page) (x < 60 ? ADIS16XXX_REG(page, (x) + 8) : \
		ADIS16XXX_REG((page) + 1, (x) - 60 + 8))
#define ADIS16XXX_FIR_COEF_A(x)			ADIS16XXX_FIR_COEF(0x05, (x))
#define ADIS16XXX_FIR_COEF_B(x)			ADIS16XXX_FIR_COEF(0x07, (x))
#define ADIS16XXX_FIR_COEF_C(x)			ADIS16XXX_FIR_COEF(0x09, (x))
#define ADIS16XXX_FIR_COEF_D(x)			ADIS16XXX_FIR_COEF(0x0B, (x))
