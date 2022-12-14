/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file mtd.c
 *
 * mtd service and utility app.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <dp_config.h>
#include <dp_log.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/drivers/drivers.h>

#include <arch/board/board.h>

//#include "systemlib/dp_macros.h"
//#include <parameters/param.h>

#include <board_config.h>

#ifdef CONFIG_ARCH_BOARD_UAVRS_V1
    #define SPI_BUS_RAMTRON PX4_SPI_BUS_RAMTRON
#elif defined(CONFIG_ARCH_BOARD_UAVRS_V2)
    #define SPI_BUS_RAMTRON UAVRS_SPI_BUS_RAMTRON
#endif

__EXPORT int mtd_main(int argc, char *argv[]);

#ifndef CONFIG_MTD

/* create a fake command with decent warning to not confuse users */
int mtd_main(int argc, char *argv[])
{
	DP_WARN("MTD not enabled, skipping.");
	return 1;
}

#else

#  if defined(BOARD_HAS_MTD_PARTITION_OVERRIDE)
#    define MTD_PARTITION_TABLE  BOARD_HAS_MTD_PARTITION_OVERRIDE
#  else
#   define MTD_PARTITION_TABLE  {"/fs/mtd_params", "/fs/mtd_waypoints"}
#  endif


#ifdef CONFIG_MTD_RAMTRON
static int	ramtron_attach(void);
#else

#ifndef DP_I2C_BUS_MTD
#define DP_I2C_BUS_ONBOARD 1
#  ifdef DP_I2C_BUS_ONBOARD
#    define DP_I2C_BUS_MTD DP_I2C_BUS_ONBOARD
#  else
#    error DP_I2C_BUS_MTD and DP_I2C_BUS_ONBOARD not defined, cannot locate onboard EEPROM
#  endif
#endif


static int	at24xxx_attach(void);
#endif
static int	mtd_start(char *partition_names[], unsigned n_partitions);
static int	mtd_erase(char *partition_names[], unsigned n_partitions);
static int	mtd_readtest(char *partition_names[], unsigned n_partitions);
static int	mtd_rwtest(char *partition_names[], unsigned n_partitions);
static int	mtd_print_info(void);
static int	mtd_get_geometry(unsigned long *blocksize, unsigned long *erasesize, unsigned long *neraseblocks,
				 unsigned *blkpererase, unsigned *nblocks, unsigned *partsize, unsigned n_partitions);

static bool attached = false;
static bool started = false;
static struct mtd_dev_s *mtd_dev;
static unsigned n_partitions_current = 0;

#if !defined(arraySize)
#define arraySize(a) (sizeof((a))/sizeof((a[0])))
#endif

/* note, these will be equally sized */
static char *partition_names_default[] = MTD_PARTITION_TABLE;
static const int n_partitions_default = arraySize(partition_names_default);

static int
mtd_status(void)
{
	if (!attached) {
		DP_ERR("MTD driver not started");
		return 1;
	}

	return mtd_print_info();
}

static void	print_usage(void)
{
	printf("Utility to mount and test partitions (based on FRAM/EEPROM storage as defined by the board)");

	printf("mtd::command");
	printf("status::Print status information");

	printf("start::Mount partitions");
	printf("readtest::Perform read test");
	printf("rwtest::Perform read-write test");
	printf("erase::Erase partition(s)");
}

int mtd_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strcmp(argv[1], "start")) {

			/* start mapping according to user request */
			if (argc >= 3) {
				return mtd_start(argv + 2, argc - 2);

			} else {
				return mtd_start(partition_names_default, n_partitions_default);
			}
		}

		if (!strcmp(argv[1], "readtest")) {
			if (argc >= 3) {
				return mtd_readtest(argv + 2, argc - 2);

			} else {
				return mtd_readtest(partition_names_default, n_partitions_default);
			}
		}

		if (!strcmp(argv[1], "rwtest")) {
			if (argc >= 3) {
				return mtd_rwtest(argv + 2, argc - 2);

			} else {
				return mtd_rwtest(partition_names_default, n_partitions_default);
			}
		}

		if (!strcmp(argv[1], "status")) {
			return mtd_status();
		}

		if (!strcmp(argv[1], "erase")) {
			if (argc >= 3) {
				return mtd_erase(argv + 2, argc - 2);

			} else {
				return mtd_erase(partition_names_default, n_partitions_default);
			}
		}
	}

	print_usage();
	return 1;
}

struct mtd_dev_s *ramtron_initialize(FAR struct spi_dev_s *dev);
struct mtd_dev_s *mtd_partition(FAR struct mtd_dev_s *mtd,
				off_t firstblock, off_t nblocks);

#ifdef CONFIG_MTD_RAMTRON
static int
ramtron_attach(void)
{
	/* initialize the right spi */
	struct spi_dev_s *spi = dp_spibus_initialize(SPI_BUS_RAMTRON);

	if (spi == NULL) {
		DP_ERR("failed to locate spi bus");
		return 1;
	}

	/* this resets the spi bus, set correct bus speed again */
	SPI_SETFREQUENCY(spi, 10 * 1000 * 1000);
	SPI_SETBITS(spi, 8);
	SPI_SETMODE(spi, SPIDEV_MODE3);
	SPI_SELECT(spi, SPIDEV_FLASH(0), false);

	/* start the RAMTRON driver, attempt 5 times */

	for (int i = 0; i < 5; i++) {
		mtd_dev = ramtron_initialize(spi);

		if (mtd_dev) {
			/* abort on first valid result */
			if (i > 0) {
				DP_WARN("mtd needed %d attempts to attach", i + 1);
			}

			break;
		}
	}

	/* if last attempt is still unsuccessful, abort */
	if (mtd_dev == NULL) {
		DP_ERR("failed to initialize mtd driver");
		return 1;
	}

	int ret = mtd_dev->ioctl(mtd_dev, MTDIOC_SETSPEED, (unsigned long)10 * 1000 * 1000);

	if (ret != OK) {
		// FIXME: From the previous warning call, it looked like this should have been fatal error instead. Tried
		// that but setting the bus speed does fail all the time. Which was then exiting and the board would
		// not run correctly. So changed to DP_WARN.
		DP_WARN("failed to set bus speed");
	}

	attached = true;
	return 0;
}
#else
#include <dp_i2c.h>
static int
at24xxx_attach(void)
{
	/* find the right I2C */
	struct i2c_master_s *i2c = dp_i2cbus_initialize(DP_I2C_BUS_MTD);

	if (i2c == NULL) {
		DP_ERR("failed to locate I2C bus");
		return 1;
	}

	/* start the MTD driver, attempt 5 times */
	for (int i = 0; i < 5; i++) {
		mtd_dev = at24c_initialize(i2c);

		if (mtd_dev) {
			/* abort on first valid result */
			if (i > 0) {
				DP_WARN("EEPROM needed %d attempts to attach", i + 1);
			}

			break;
		}
	}

	/* if last attempt is still unsuccessful, abort */
	if (mtd_dev == NULL) {
		DP_ERR("failed to initialize EEPROM driver");
		return 1;
	}

	attached = true;
	return 0;
}
#endif

static int
mtd_start(char *partition_names[], unsigned n_partitions)
{
	int ret;

	if (started) {
		DP_ERR("mtd already mounted");
		return 1;
	}

	if (!attached) {
#ifdef CONFIG_MTD_RAMTRON
		ret = ramtron_attach();
#else
		ret = at24xxx_attach();
#endif

		if (ret != 0) {
			return ret;
		}
	}

	if (!mtd_dev) {
		DP_ERR("Failed to create RAMTRON FRAM MTD instance");
		return 1;
	}

	unsigned long blocksize, erasesize, neraseblocks;
	unsigned blkpererase, nblocks, partsize;

	ret = mtd_get_geometry(&blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize, n_partitions);

	if (ret) {
		return ret;
	}

	/* Now create MTD FLASH partitions */

	FAR struct mtd_dev_s *part[n_partitions];
	char blockname[32];

	unsigned offset;
	unsigned i;

	for (offset = 0, i = 0; i < n_partitions; offset += nblocks, i++) {

		/* Create the partition */

		part[i] = mtd_partition(mtd_dev, offset, nblocks);

		if (!part[i]) {
			DP_ERR("mtd_partition failed. offset=%lu nblocks=%lu",
				(unsigned long)offset, (unsigned long)nblocks);
			return 1;
		}

		/* Initialize to provide an FTL block driver on the MTD FLASH interface */

		snprintf(blockname, sizeof(blockname), "/dev/mtdblock%d", i);

		ret = ftl_initialize(i, part[i]);

		if (ret < 0) {
			DP_ERR("ftl_initialize %s failed: %d", blockname, ret);
			return 1;
		}

		/* Now create a character device on the block device */

		ret = bchdev_register(blockname, partition_names[i], false);

		if (ret < 0) {
			DP_ERR("bchdev_register %s failed: %d", partition_names[i], ret);
			return 1;
		}
	}

	n_partitions_current = n_partitions;

	started = true;
	return 0;
}

int mtd_get_geometry(unsigned long *blocksize, unsigned long *erasesize, unsigned long *neraseblocks,
		     unsigned *blkpererase, unsigned *nblocks, unsigned *partsize, unsigned n_partitions)
{
	/* Get the geometry of the FLASH device */

	FAR struct mtd_geometry_s geo;

	int ret = mtd_dev->ioctl(mtd_dev, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));

	if (ret < 0) {
		DP_ERR("mtd->ioctl failed: %d", ret);
		return ret;
	}

	*blocksize = geo.blocksize;
	*erasesize = geo.erasesize;
	*neraseblocks = geo.neraseblocks;

	/* Determine the size of each partition.  Make each partition an even
	 * multiple of the erase block size (perhaps not using some space at the
	 * end of the FLASH).
	 */

	*blkpererase = geo.erasesize / geo.blocksize;
	*nblocks     = (geo.neraseblocks / n_partitions) * *blkpererase;
	*partsize    = *nblocks * geo.blocksize;

	return ret;
}

/*
  get partition size in bytes
 */
static ssize_t mtd_get_partition_size(void)
{
	unsigned long blocksize, erasesize, neraseblocks;
	unsigned blkpererase, nblocks, partsize = 0;

	int ret = mtd_get_geometry(&blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize,
				   n_partitions_current);

	if (ret != OK) {
		DP_ERR("Failed to get geometry");
		return 0;
	}

	return partsize;
}

int mtd_print_info(void)
{
	if (!attached) {
		return 1;
	}

	unsigned long blocksize, erasesize, neraseblocks;
	unsigned blkpererase, nblocks, partsize;

	int ret = mtd_get_geometry(&blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize,
				   n_partitions_current);

	if (ret) {
		return ret;
	}

	DP_INFO("Flash Geometry:");

	printf("  blocksize:      %lu\n", blocksize);
	printf("  erasesize:      %lu\n", erasesize);
	printf("  neraseblocks:   %lu\n", neraseblocks);
	printf("  No. partitions: %u\n", n_partitions_current);
	printf("  Partition size: %u Blocks (%u bytes)\n", nblocks, partsize);
	printf("  TOTAL SIZE: %u Bytes\n", neraseblocks * erasesize);

	return 0;
}

int
mtd_erase(char *partition_names[], unsigned n_partitions)
{
	uint8_t v[64];
	memset(v, 0xFF, sizeof(v));

	for (uint8_t i = 0; i < n_partitions; i++) {
		uint32_t count = 0;
		printf("Erasing %s\n", partition_names[i]);
		int fd = open(partition_names[i], O_WRONLY);

		if (fd == -1) {
			DP_ERR("Failed to open partition");
			return 1;
		}

		while (write(fd, v, sizeof(v)) == sizeof(v)) {
			count += sizeof(v);
		}

		printf("Erased %lu bytes\n", (unsigned long)count);
		close(fd);
	}

	return 0;
}

/*
  readtest is useful during startup to validate the device is
  responding on the bus. It relies on the driver returning an error on
  bad reads (the ramtron driver does return an error)
 */
int
mtd_readtest(char *partition_names[], unsigned n_partitions)
{
	ssize_t expected_size = mtd_get_partition_size();

	if (expected_size == 0) {
		return 1;
	}

	uint8_t v[128];

	for (uint8_t i = 0; i < n_partitions; i++) {
		ssize_t count = 0;
		printf("reading %s expecting %u bytes\n", partition_names[i], expected_size);
		int fd = open(partition_names[i], O_RDONLY);

		if (fd == -1) {
			DP_ERR("Failed to open partition");
			return 1;
		}

		while (read(fd, v, sizeof(v)) == sizeof(v)) {
			count += sizeof(v);
		}

		if (count != expected_size) {
			DP_ERR("Failed to read partition - got %u/%u bytes", count, expected_size);
			return 1;
		}

		close(fd);
	}

	printf("readtest OK\n");
	return 0;
}

/*
  rwtest is useful during startup to validate the device is
  responding on the bus for both reads and writes. It reads data in
  blocks and writes the data back, then reads it again, failing if the
  data isn't the same
 */
int
mtd_rwtest(char *partition_names[], unsigned n_partitions)
{
	ssize_t expected_size = mtd_get_partition_size();

	if (expected_size == 0) {
		return 1;
	}

	uint8_t v[128], v2[128];

	for (uint8_t i = 0; i < n_partitions; i++) {
		ssize_t count = 0;
		off_t offset = 0;
		printf("rwtest %s testing %u bytes\n", partition_names[i], expected_size);
		int fd = open(partition_names[i], O_RDWR);

		if (fd == -1) {
			DP_ERR("Failed to open partition");
			return 1;
		}

		while (read(fd, v, sizeof(v)) == sizeof(v)) {
			count += sizeof(v);

			if (lseek(fd, offset, SEEK_SET) != offset) {
				DP_ERR("seek failed");
				return 1;
			}

			if (write(fd, v, sizeof(v)) != sizeof(v)) {
				DP_ERR("write failed");
				return 1;
			}

			if (lseek(fd, offset, SEEK_SET) != offset) {
				DP_ERR("seek failed");
				return 1;
			}

			if (read(fd, v2, sizeof(v2)) != sizeof(v2)) {
				DP_ERR("read failed");
				return 1;
			}

			if (memcmp(v, v2, sizeof(v2)) != 0) {
				DP_ERR("memcmp failed");
				return 1;
			}

			offset += sizeof(v);
		}

		if (count != expected_size) {
			DP_ERR("Failed to read partition - got %u/%u bytes", count, expected_size);
			return 1;
		}

		close(fd);
	}

	printf("rwtest OK\n");
	return 0;
}

#endif
