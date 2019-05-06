/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
* @file ver.c
*
* Version command, unifies way of showing versions of HW, SW, Build, GCC
* In case you want to add new version just extend version_main function
*
* @author Vladimir Kulla <ufon@kullaonline.net>
*/

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <version/version.h>
#include <systemlib/err.h>
//#include <systemlib/mcu_version.h>
//#include <systemlib/git_version.h>

/* string constants for version commands */
static const char sz_ver_hw_str[] 	= "hw";
static const char sz_ver_hwcmp_str[] = "hwcmp";
static const char sz_ver_git_str[] 	= "git";
static const char sz_ver_bdate_str[] = "bdate";
static const char sz_ver_gcc_str[] 	= "gcc";
static const char sz_ver_all_str[] 	= "all";
static const char mcu_ver_str[]		= "mcu";
static const char mcu_uid_str[]		= "uid";

#define DP_GIT_VERSION_STR  "f5b008dcc0c1f0d179a1e9e93e834685e83479ee"
#define PX4_GIT_VERSION_BINARY 0x5b008dcc0c1f0d17

const char *dp_git_version = DP_GIT_VERSION_STR;
const uint64_t dp_git_version_binary = PX4_GIT_VERSION_BINARY;

static void usage(const char *reason)
{
	if (reason != NULL) {
		printf("%s\n", reason);
	}

	printf("usage: ver {hw|hwcmp|git|bdate|gcc|all|mcu|uid}\n\n");
}

__EXPORT int ver_main(int argc, char *argv[]);

int ver_main(int argc, char *argv[])
{
	/* defaults to an error */
	int ret = 1;

	/* first check if there are at least 2 params */
	if (argc >= 2) {
		if (argv[1] != NULL) {

			if (!strncmp(argv[1], sz_ver_hwcmp_str, sizeof(sz_ver_hwcmp_str))) {
				if (argc >= 3 && argv[2] != NULL) {
					/* compare 3rd parameter with HW_ARCH string, in case of match, return 0 */
					ret = strncmp(HW_ARCH, argv[2], strlen(HW_ARCH));

					if (ret == 0) {
						printf("ver hwcmp match: %s\n", HW_ARCH);
					}

					return ret;

				} else {
					warn("Not enough arguments, try 'ver hwcmp DPFMU_V2'");
					return 1;
				}
			}

			/* check if we want to show all */
			bool show_all = !strncmp(argv[1], sz_ver_all_str, sizeof(sz_ver_all_str));

			if (show_all || !strncmp(argv[1], sz_ver_hw_str, sizeof(sz_ver_hw_str))) {
				printf("HW arch: %s\n", HW_ARCH);
				ret = 0;

			}

			if (show_all || !strncmp(argv[1], sz_ver_git_str, sizeof(sz_ver_git_str))) {
				printf("FW git-hash: %s\n", dp_git_version);
				ret = 0;

			}

			if (show_all || !strncmp(argv[1], sz_ver_bdate_str, sizeof(sz_ver_bdate_str))) {
				printf("Build datetime: %s %s\n", __DATE__, __TIME__);
				ret = 0;

			}

			if (show_all || !strncmp(argv[1], sz_ver_gcc_str, sizeof(sz_ver_gcc_str))) {
				printf("Toolchain: %s\n", __VERSION__);
				ret = 0;

			}

			if (show_all || !strncmp(argv[1], mcu_ver_str, sizeof(mcu_ver_str))) {
                #if 0
				char rev;
				char *revstr;

                int chip_version = mcu_version(&rev, &revstr);

				if (chip_version < 0) {
					printf("UNKNOWN MCU\n");

				} else {
					printf("MCU: %s, rev. %c\n", revstr, rev);
				}
                #else
                #ifdef CONFIG_ARCH_BOARD_UAVRS_V1
                printf("MCU: %s, rev. %c\n", "STM32F42X", 'A');
                #elif defined(CONFIG_ARCH_BOARD_UAVRS_V2)
                printf("MCU: %s, rev. %c\n", "IMXRT105X", 'A');
                #else
                printf("UNKNOWN MCU\n");
                #endif
                #endif
				ret = 0;
			}

			if (show_all || !strncmp(argv[1], mcu_uid_str, sizeof(mcu_uid_str))) {
				uint32_t uid[3];

				//mcu_unique_id(uid);

				printf("UID: %X:%X:%X \n", uid[0], uid[1], uid[2]);

				ret = 0;
			}


			if (ret == 1) {
				warn("unknown command.\n");
				return 1;
			}

		} else {
			usage("Error, input parameter NULL.\n");
		}

	} else {
		usage("Error, not enough parameters.");
	}

	return ret;
}
