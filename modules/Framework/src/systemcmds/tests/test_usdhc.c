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
#define FILE_ACCESS_ON_KERNEL_SPACE
//#define FILE_ACCESS_ON_USER_SPACE

enum dev_cmd {
    DEV_CMD_ERROR,
    MMCSD_READ,
    MMCSD_WRITE,
    MMCSD_WRITE_READ
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

static int test_mmcsd_init(void)
{        
    return 0;
}

/****************************************************************************
 * Name: test_sdcard_write
 ****************************************************************************/
 #ifdef FILE_ACCESS_ON_KERNEL_SPACE
static int test_mmcsd_write(uint32_t blockcnt, uint32_t blocksize)
{
    int res = 0;
    int len = 0;
    uint32_t blockcnt_tmp = blockcnt;
    uint8_t *buf = NULL;
    int fd = 0;
    
    printf("*************test_mmcsd_write****************\n");
    /* open test.txt file */
    fd = open(TEST_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if(-1 == fd) {
        printf("%s open failed\n", TEST_FILE);
        return -1;
    }

    /* alloc buf to write */
    buf = malloc(blocksize);
    if(NULL == buf) {
        printf("malloc write buf[%d] error\n", blocksize);
        return -1;
    }
    memset(buf, 'A', blocksize);

    /* write blocksize * blockcnt bytes data */
    while(blockcnt) {
        clock_gettime(CLOCK_REALTIME, &tpstart);
        res = write(fd, buf, blocksize);
        blockcnt--;
        if(blocksize != res) {
            len = res;
            res = -1;
            break;
        } else {
            res = 0;
        }
        res = fsync(fd);
        if(res) {
            printf("fsync error\n");
        }
        clock_gettime(CLOCK_REALTIME, &tpend);
        dt_ns =  (tpend.tv_sec-tpstart.tv_sec) + (tpend.tv_nsec-tpstart.tv_nsec);
        dt_us = dt_ns / (double)1000.0;
        dt_ms = dt_ns / (double)1000000.0;
        printf("%d*%d  %dms\n", blockcnt_tmp - blockcnt, blocksize, (int)dt_ms);
    }

    if(res) {
        printf("write %d*%d bytes error, real write %d\n", blockcnt_tmp - blockcnt, blockcnt, len);
    } else {
        printf("write %d*%d bytes ok\n", blockcnt_tmp - blockcnt, blocksize);
    }

    /* close file and free buf */
    close(fd);
    free(buf);
    fd = 0;
    buf = NULL;
    return res;
}

#else
static int test_mmcsd_write(uint32_t blockcnt, uint32_t blocksize)
{
    int res = 0;
    int len = 0;
    uint32_t blockcnt_tmp = blockcnt;
    uint8_t *buf = NULL;
    FAR FILE *fd_test = NULL;

    printf("*************test_mmcsd_write****************\n");
    /* open test.txt file */
    fd_test = fopen(TEST_FILE, "w");
    if(NULL == fd_test) {
        printf("%s open failed\n", TEST_FILE);
        return -1;
    }

    /* alloc buf to write */
    buf = malloc(blocksize);
    if(NULL == buf) {
        printf("malloc write buf[%d] error\n", blocksize);
        return -1;
    }
    memset(buf, 'B', blocksize);

    /* write blocksize * blockcnt bytes data */
    while(blockcnt) {
        res = fwrite(buf, 1, blocksize, fd_test);
        blockcnt--;
        if(blocksize != res) {
            len = res;
            res = -1;
            break;
        } else {
            res = 0;
        }            
    }
    
    if(res) {
        printf("write %d*%d bytes error, real write %d\n", blockcnt_tmp - blockcnt, blockcnt, len);
    } else {
        res = fflush(fd_test);
        if(res) {
            printf("fflush error\n");
        }
        
        res = fsync(fileno(fd_test));
        if(res) {
            printf("fsync error\n");
        } 
        printf("fwrite %d*%d bytes ok\n", blockcnt_tmp - blockcnt, blockcnt);
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
static int test_mmcsd_read(uint32_t blockcnt, uint32_t blocksize)
{
    int res = 0;
    int len = 0;
    int blockcnt_tmp = blockcnt;
    uint8_t *buf1 = NULL;
    uint8_t *buf2 = NULL;
    int fd = 0;
    
    printf("*************test_mmcsd_read****************\n");
    /* open test.txt file */
    fd = open(TEST_FILE, O_RDONLY);
    if(-1 == fd) {
        printf("%s open failed\n", TEST_FILE);
        return -1;
    }

    /* alloc buf1 and buf2 to recv data */
    buf1 = malloc(blocksize);
    if(NULL == buf1) {
        printf("malloc read buf1[%d] error\n", blocksize);
        return -1;
    }
    memset(buf1, 0, blocksize);
    
    buf2 = malloc(blocksize);
    if(NULL == buf2) {
        printf("malloc compare buf2[%d] error\n", blocksize);
        close(fd);
        free(buf1);
        fd = 0;
        buf1 = NULL;
        return -1;
    }
    memset(buf2, 'A', blocksize);

    while(blockcnt) {
        res = read(fd, buf1, blocksize);
        blockcnt--;
        if(blocksize == res) {
            res = memcmp(buf1, buf2, blocksize);
            if(res) {
                printf("memcmp error\n");
                res = -1;
                break;
                
            }
        } else {
            printf("read error\n");
            len = res;
            res = -1;
            break;
           
        }
    }

    if(res)
        printf("read and compare %d*%d bytes error, real read:%d\n", blockcnt_tmp - blockcnt, blocksize, len);
    else
        printf("read and compare %d*%d bytes ok\n", blockcnt_tmp - blockcnt, blocksize);

    close(fd);
    free(buf1);
    free(buf2);
    fd = 0;
    buf1 = NULL;
    buf2 = NULL;

    return res;
}

#else
static int test_mmcsd_read(uint32_t uint32_t blockcnt, uint32_t blocksize)
{
    int res = 0;
    int len = 0;
    int blockcnt_tmp = blockcnt;
    uint8_t *buf1 = NULL;
    uint8_t *buf2 = NULL;
    FAR FILE *fd_test = NULL;

    
    printf("*************test_mmcsd_read****************\n");
    /* open test.txt file */
    fd_test = fopen(TEST_FILE, "r");
    if(NULL == fd_test) {
        printf("%s open failed\n", TEST_FILE);
        return -1;
    }
#if 0
    /* get the file length to read */
    /* bug: fseek SEEK_END return -1 , pos pointer are on 99*4k*/
    res = fseek(fd_test, 0L, SEEK_END);
    if(res) {
        printf("fseek SEEK_END error %d\n", res);
        return -1;
    }

    res = ftell(fd_test);
    if(res) {
        printf("fseek SEEK_END error %d\n", res);
        return -1;
    }

    
    res = fseek(fd_test, 0L, SEEK_SET);
    if(res) {
        printf("fseek SEEK_SET error %d\n", res);
        len = 0;
        res = -1;
    }
    
    if(0 >= len) {
        printf("get file size error\n");
        len = 0;
        res = -1;
    }
    
    printf("fseek file len: %d(%d*%d) bytes\n", len, len / blocksize, blocksize);
#endif

    /* alloc buf1 and buf2 to recv data */
    buf1 = malloc(blocksize);
    if(NULL == buf1) {
        printf("malloc read buf1[%d] error\n", blocksize);
        return -1;
    }
    memset(buf1, 0, blocksize);
    
    buf2 = malloc(blocksize);
    if(NULL == buf2) {
        printf("malloc compare buf2[%d] error\n", blocksize);
        fclose(fd);
        free(buf1);
        fd = 0;
        buf1 = NULL;
        return -1;
    }
    memset(buf2, 'B', blocksize);
    
    /* read test.txt and compare */
    while(blockcnt) {
        res = fread(buf1, 1, blocksize, fd_test);
        blockcnt--;
        if(blocksize == res) {
            res = memcmp(buf1, buf2, blocksize);
            if(res) {
                printf("memcmp error\n");
                res = -1;
                break;
            }
        } else {
            printf("read error\n");
            len = res;
            res = -1;
            break;
           
        }
    }

    if(res)
        printf("fread and compare %d*%d bytes error, real read:%d\n", blockcnt_tmp - blockcnt, blocksize, len);
    else
        printf("fread and compare %d*%d bytes ok\n", blockcnt_tmp - blockcnt, blocksize);
    
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
    printf("example: tests usdhc [cmd blockcnt blocksize]\n");
    printf("cmd: read/write\n");
    printf("blockcnt: 8\n");
    printf("blocksize: 512\n");
    return 0;
}

/****************************************************************************
 * Name: test_usdhc_arg_parse
 ****************************************************************************/
static enum dev_cmd test_usdhc_arg_parse(int argc, char *argv[])
{
    enum dev_cmd dc = DEV_CMD_ERROR;
    
    if(1 != argc && 4 != argc) {
        test_usdhc_help();
        return DEV_CMD_ERROR;
    }

    if(1 == argc) {   
        if(!strcmp("usdhc", argv[0])) {
            dc = MMCSD_WRITE_READ;
            return dc;
        } else {
            test_usdhc_help();
        }
    }

    if(4 == argc) {   
        if(!strcmp("read", argv[1])) {
            dc = MMCSD_READ;
            return dc;
        }

        if(!strcmp("write", argv[1])) {
            dc = MMCSD_WRITE;
            return dc;
        }

        return DEV_CMD_ERROR;
    }
    
    return dc;   
}

/****************************************************************************
 * Name: test_usdhc
 ****************************************************************************/

int test_usdhc(int argc, char *argv[])
{
    int res = 0;
    uint32_t blockcnt = 0;
    uint32_t blocksize = 0;

    dc = test_usdhc_arg_parse(argc, argv);
    if(DEV_CMD_ERROR == dc) { 
        printf("\nerror cmd: argc %d\n", argc);
        for(int i = 0; i < argc; i++) {
            printf("argv[%d]:%s\n", i, argv[i]);
        }
        test_usdhc_help();
        return -1;
    }
    
    res = test_mmcsd_init();
    if(0 != res) {
        printf("test_sdcard_init error\n");
        return -1;
    }

    switch(dc) {
        case MMCSD_READ:
            blockcnt = atoi(argv[2]);
            blocksize = atoi(argv[3]);
            test_mmcsd_read(blockcnt, blocksize);
            break;
        case MMCSD_WRITE:
            blockcnt = atoi(argv[2]);
            blocksize = atoi(argv[3]);
            test_mmcsd_write(blockcnt, blocksize);
            break;
        case MMCSD_WRITE_READ:
            blockcnt = 1;
            blocksize = 4096;
            res = test_mmcsd_write(blockcnt, blocksize);
            if(!res) {
                test_mmcsd_read(blockcnt, blocksize);
            }
            break;
        default:
            break;
    }

	return 0;
}
