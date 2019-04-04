/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file tests_usdhc.c
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

#include "tests.h"

#include <math.h>
#include <float.h>

#ifndef NULL
# define NULL (void*)0
#endif

#define TEST_FILE "/fs/microsd/test.txt"
#define BUF_SIZE 4096
#define WRITE_4K_CNT 100

#define FILE_ACCESS_ON_KERNEL_SPACE
//#define FILE_ACCESS_ON_USER_SPACE




enum dev_cmd {
    DEV_CMD_ERROR,
    SDCARD_READ,
    SDCARD_WRITE,
    SDCARD_WRITE_READ,
    EMMC_READ,
    EMMC_WRITE,
    EMMC_WRITE_READ
};

struct timespec tpstart;
struct timespec tpend;
double dt_ns = 0.0;
double dt_us = 0.0;
double dt_ms = 0.0;
enum dev_cmd dc = DEV_CMD_ERROR;

/****************************************************************************
 * Name: test_sdcard_init
 ****************************************************************************/

static int test_sdcard_init(void)
{        
    return 0;
}

/****************************************************************************
 * Name: test_sdcard_write
 ****************************************************************************/
 #ifdef FILE_ACCESS_ON_KERNEL_SPACE
static int test_sdcard_write(void)
{
    int res = 0;
    int cnt = WRITE_4K_CNT;
    uint8_t *buf = NULL;
    int fd = 0;
    
    printf("*************test_sdcard_write****************\n");
    /* open test.txt file */
    fd = open(TEST_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if(-1 == fd) {
        printf("%s open failed\n", TEST_FILE);
        return -1;
    }

    /* alloc buf to write */
    buf = malloc(BUF_SIZE);
    if(NULL == buf) {
        printf("%s test buf malloc error\n", TEST_FILE);
        return -1;
    }
    memset(buf, 'B', BUF_SIZE);
    buf[BUF_SIZE - 1] = '\n';

    /* write cnt*4k data */
    while(cnt) {
        clock_gettime(CLOCK_REALTIME, &tpstart);
        res = write(fd, buf, BUF_SIZE);
        if(BUF_SIZE != res) {           
            res = -1;
            break;
        } else {
            res = 0;
            cnt--;
        }
        res = fsync(fd);
        if(res) {
            printf("fsync error\n");
        }
        clock_gettime(CLOCK_REALTIME, &tpend);
        dt_ns =  (tpend.tv_sec-tpstart.tv_sec) + (tpend.tv_nsec-tpstart.tv_nsec);
        dt_us = dt_ns / (double)1000.0;
        dt_ms = dt_ns / (double)1000000.0;
        printf("%d  %dms\n", cnt, (int)dt_ms);
    }

    if(res) {
        printf("test_sdcard_write write %d*4k bytes error\n", cnt);
    } else {
        printf("test_sdcard_write write %d*4k bytes ok\n", WRITE_4K_CNT);
    }

    /* close file and free buf */
    close(fd);
    free(buf);
    fd = 0;
    buf = NULL;
    return res;
}

#else
static int test_sdcard_write(void)
{
    int res = 0;
    int cnt = WRITE_4K_CNT;
    uint8_t *buf = NULL;
    FAR FILE *fd_test = NULL;

    printf("*************test_sdcard_write****************\n");
    /* open test.txt file */
    fd_test = fopen(TEST_FILE, "w");
    if(NULL == fd_test) {
        printf("%s open failed\n", TEST_FILE);
        return -1;
    }

    /* alloc buf to write */
    buf = malloc(BUF_SIZE);
    if(NULL == buf) {
        printf("%s test buf malloc error\n", TEST_FILE);
        return -1;
    }
    memset(buf, 'A', BUF_SIZE);
    buf[BUF_SIZE - 1] = '\n';

    /* write cnt*4k data */
    while(cnt) {
        res = fwrite(buf, 1, BUF_SIZE, fd_test);
        if(BUF_SIZE != res) {           
            res = -1;
            break;
        } else {
            res = 0;
            cnt--;
        }            
    }
    
    if(res) {
        printf("test_sdcard_write fwrite %d*4k bytes error\n", cnt);
    } else {
        res = fflush(fd_test);
        if(res) {
            printf("fflush error\n");
        }
        
        res = fsync(fileno(fd_test));
        if(res) {
            printf("fsync error\n");
        } 
        printf("test_sdcard_write fwrite %d*4k bytes ok\n", WRITE_4K_CNT);
    }

    /* close file and free buf */
    fclose(fd_test);
    free(buf);
    fd_test = NULL;
    buf = NULL;
    return res;
}
#endif

/****************************************************************************
 * Name: test_sdcard_read
 ****************************************************************************/
#ifdef FILE_ACCESS_ON_KERNEL_SPACE
static int test_sdcard_read(void)
{
    int res = 0;
    int len = 0;
    uint8_t *buf1 = NULL;
    uint8_t *buf2 = NULL;
    int fd = 0;
    
    printf("*************test_sdcard_read****************\n");
    /* open test.txt file */
    fd = open(TEST_FILE, O_RDONLY);
    if(-1 == fd) {
        printf("%s open failed\n", TEST_FILE);
        return -1;
    }

    /* alloc buf1 and buf2 to recv data */
    buf1 = malloc(BUF_SIZE);
    if(NULL == buf1) {
        printf("malloc error\n");
        return -1;
    }
    memset(buf1, 0, BUF_SIZE);
    
    buf2 = malloc(BUF_SIZE);
    if(NULL == buf2) {
        printf("malloc error\n");
        return -1;
    }
    memset(buf2, 'B', BUF_SIZE);
    buf2[BUF_SIZE - 1] = '\n';

    while(len < WRITE_4K_CNT*BUF_SIZE) {
        res = read(fd, buf1, BUF_SIZE);
        if(BUF_SIZE == res) {
            len += BUF_SIZE;
            res = memcmp(buf1, buf2, BUF_SIZE);
            if(res) {
                printf("memcmp error\n");
                res = -1;
                break;
                
            }
        } else {
            printf("read error\n");
            res = -1;
            break;
           
        }
    }

    if(res)
        printf("read and compare %d*4k(%d) bytes error\n", len / BUF_SIZE, len);
    else
        printf("read and compare %d*4k(%d) bytes ok\n", len / BUF_SIZE, len);

    close(fd);
    free(buf1);
    free(buf2);
    fd = 0;
    buf1 = NULL;
    buf2 = NULL;

    return res;
}

#else
static int test_sdcard_read(void)
{
    int res = 0;
    int len = 0;
    int len_tmp = 0;
    uint8_t *buf1 = NULL;
    uint8_t *buf2 = NULL;
    FAR FILE *fd_test = NULL;

    
    printf("*************test_sdcard_read****************\n");
    /* open test.txt file */
    fd_test = fopen(TEST_FILE, "r");
    if(NULL == fd_test) {
        printf("%s open failed\n", TEST_FILE);
        return -1;
    }

    /* alloc buf1 and buf2 to recv data */
    buf1 = malloc(BUF_SIZE);
    if(NULL == buf1) {
        printf("malloc error\n");
        return -1;
    }
    memset(buf1, 0, BUF_SIZE);
    
    buf2 = malloc(BUF_SIZE);
    if(NULL == buf2) {
        printf("malloc error\n");
        return -1;
    }
    memset(buf2, 'A', BUF_SIZE);
    buf2[BUF_SIZE - 1] = '\n';

    /* get the file length to read */
    /* bug: fseek SEEK_END return -1 , pos pointer are on 99*4k*/
    res = fseek(fd_test, 0L, SEEK_END);
    if(res) {
        printf("fseek SEEK_END error %d\n", res);
        res = -1;
    }
    len = ftell(fd_test);
    res = fseek(fd_test, 0L, SEEK_SET);
    if(res) {
        printf("fseek SEEK_SET error %d\n", res);
        len = 0;
        res = -1;
    }
    len_tmp = len;
    printf("fseek file len: %d*4k(%d) bytes\n", len_tmp / BUF_SIZE, len_tmp);
    if(0 >= len) {
        printf("get file size error\n");
        len = 0;
        res = -1;
    }
    
    /* read test.txt and compare */
    while(len) {
        res = fread(buf1, 1, BUF_SIZE, fd_test);
        if(0 < res) {
            len -= res;
        }

        res = memcmp(buf1, buf2, BUF_SIZE);
        if(res) {
            printf("memcmp error\n");
            res = -1;
        }
    }

    if(res)
        printf("fread and compare %d*4k(%d) bytes error\n", len_tmp - len / BUF_SIZE, len_tmp - len);
    else
        printf("fread and compare %d*4k(%d) bytes ok\n", len_tmp / BUF_SIZE, len_tmp);
    
    /* close file and free buf */
    fclose(fd_test);
    free(buf1);
    free(buf2);
    fd_test = NULL;
    buf1 = NULL;
    buf2 = NULL;
    return res;
}
#endif

/****************************************************************************
 * Name: test_usdhc_help
 ****************************************************************************/
static int test_usdhc_help(void)
{
    printf("example: tests usdhc dev [cmd]\n");
    printf("dev: sdcard/emmc\n");
    printf("cmd: read/write\n");
    return 0;
}

/****************************************************************************
 * Name: test_usdhc_arg_parse
 ****************************************************************************/
static enum dev_cmd test_usdhc_arg_parse(int argc, char *argv[])
{
    enum dev_cmd dc = DEV_CMD_ERROR;
    
    if(3 != argc && 2 != argc) {
        return dc;
    }

    if(2 == argc) {
        if(!strcmp("sdcard", argv[1])) {
            dc = SDCARD_WRITE_READ;
            return dc;
        }

        if(!strcmp("emmc", argv[1])) {
            dc = EMMC_WRITE_READ;
            return dc;
        }        
    }

    if(3 == argc) {   
        if(!strcmp("sdcard", argv[1]) && !strcmp("read", argv[2])) {
            dc = SDCARD_READ;
            return dc;
        }

        if(!strcmp("sdcard", argv[1]) && !strcmp("write", argv[2])) {
            dc = SDCARD_WRITE;
            return dc;
        }

        if(!strcmp("emmc", argv[1]) && !strcmp("read", argv[2])) {
            dc = EMMC_READ;
            return dc;
        }

        if(!strcmp("emmc", argv[1]) && !strcmp("write", argv[2])) {
            dc = EMMC_WRITE;
            return dc;
        }
    }
    
    return dc;   
}

/****************************************************************************
 * Name: test_usdhc
 ****************************************************************************/

int test_usdhc(int argc, char *argv[])
{
    int res = 0;

    dc = test_usdhc_arg_parse(argc, argv);
    if(DEV_CMD_ERROR == dc) { 
        printf("\nerrror cmd: argc %d\n", argc);
        for(int i = 0; i < argc; i++) {
            printf("argv[%d]:%s\n", i, argv[i]);
        }
        test_usdhc_help();
        return -1;
    }
    
    res = test_sdcard_init();
    if(0 != res) {
        printf("test_sdcard_init error\n");
        return -1;
    }

    switch(dc) {
        case SDCARD_READ:
            test_sdcard_read();
            break;
        case SDCARD_WRITE:
            test_sdcard_write();
            break;
        case SDCARD_WRITE_READ:
            res = test_sdcard_write();
            if(!res) {
                test_sdcard_read();
            }
            break;
        case EMMC_READ:
            test_sdcard_read();
            break;
        case EMMC_WRITE:
            test_sdcard_write();
            break;
        case EMMC_WRITE_READ:
            res = test_sdcard_write();
            if(!res) {
                test_sdcard_read();
            }
            break;
        default:
            break;
    }

	return 0;
}
