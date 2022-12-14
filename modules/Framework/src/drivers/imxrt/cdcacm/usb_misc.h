/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __USB_MISC_H__
#define __USB_MISC_H__

#include "usb_device_config.h"

/*! @name Min/max macros */
/* @{ */
#if !defined(MIN)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#if !defined(MAX)
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
/* @} */

/*! @brief Computes the number of elements in an array. */
#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef ENDIANNESS

#error ENDIANNESS should be defined, and then rebulid the project.

#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Define USB printf */
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#define usb_echo printf

#ifndef STRUCT_PACKED
#define STRUCT_PACKED
#endif

#ifndef STRUCT_UNPACKED
#define STRUCT_UNPACKED __attribute__((__packed__))
#endif

#define USB_SHORT_GET_LOW(x) (((uint16_t)x) & 0xFFU)
#define USB_SHORT_GET_HIGH(x) ((uint8_t)(((uint16_t)x) >> 8U) & 0xFFU)

#define USB_LONG_GET_BYTE0(x) ((uint8_t)(((uint32_t)(x))) & 0xFFU)
#define USB_LONG_GET_BYTE1(x) ((uint8_t)(((uint32_t)(x)) >> 8U) & 0xFFU)
#define USB_LONG_GET_BYTE2(x) ((uint8_t)(((uint32_t)(x)) >> 16U) & 0xFFU)
#define USB_LONG_GET_BYTE3(x) ((uint8_t)(((uint32_t)(x)) >> 24U) & 0xFFU)

#define USB_MEM4_ALIGN_MASK (0x03U)

/* accessory macro */
#define USB_MEM4_ALIGN(n) ((n + 3U) & (0xFFFFFFFCu))
#define USB_MEM32_ALIGN(n) ((n + 31U) & (0xFFFFFFE0u))
#define USB_MEM64_ALIGN(n) ((n + 63U) & (0xFFFFFFC0u))

/* big/little endian */
#define SWAP2BYTE_CONST(n) ((((n)&0x00FFU) << 8U) | (((n)&0xFF00U) >> 8U))
#define SWAP4BYTE_CONST(n) \
    ((((n)&0x000000FFU) << 24U) | (((n)&0x0000FF00U) << 8U) | (((n)&0x00FF0000U) >> 8U) | (((n)&0xFF000000U) >> 24U))

#define USB_ASSIGN_VALUE_ADDRESS_LONG_BY_BYTE(n, m)      \
    {                                                    \
        *((uint8_t *)&(n)) = *((uint8_t *)&(m));         \
        *((uint8_t *)&(n) + 1) = *((uint8_t *)&(m) + 1); \
        *((uint8_t *)&(n) + 2) = *((uint8_t *)&(m) + 2); \
        *((uint8_t *)&(n) + 3) = *((uint8_t *)&(m) + 3); \
    }

#define USB_ASSIGN_VALUE_ADDRESS_SHORT_BY_BYTE(n, m)     \
    {                                                    \
        *((uint8_t *)&(n)) = *((uint8_t *)&(m));         \
        *((uint8_t *)&(n) + 1) = *((uint8_t *)&(m) + 1); \
    }

#define USB_ASSIGN_MACRO_VALUE_ADDRESS_LONG_BY_BYTE(n, m) \
    {                                                     \
        *((uint8_t *)&(n)) = (uint8_t)m;                  \
        *((uint8_t *)&(n) + 1) = (uint8_t)(m >> 8);       \
        *((uint8_t *)&(n) + 2) = (uint8_t)(m >> 16);      \
        *((uint8_t *)&(n) + 3) = (uint8_t)(m >> 24);      \
    }

#define USB_ASSIGN_MACRO_VALUE_ADDRESS_SHORT_BY_BYTE(n, m) \
    {                                                      \
        *((uint8_t *)&(n)) = (uint8_t)m;                   \
        *((uint8_t *)&(n) + 1) = (uint8_t)(m >> 8);        \
    }

#if (ENDIANNESS == USB_BIG_ENDIAN)

#define USB_SHORT_TO_LITTLE_ENDIAN(n) SWAP2BYTE_CONST(n)
#define USB_LONG_TO_LITTLE_ENDIAN(n) SWAP4BYTE_CONST(n)
#define USB_SHORT_FROM_LITTLE_ENDIAN(n) SWAP2BYTE_CONST(n)
#define USB_LONG_FROM_LITTLE_ENDIAN(n) SWAP2BYTE_CONST(n)

#define USB_SHORT_TO_BIG_ENDIAN(n) (n)
#define USB_LONG_TO_BIG_ENDIAN(n) (n)
#define USB_SHORT_FROM_BIG_ENDIAN(n) (n)
#define USB_LONG_FROM_BIG_ENDIAN(n) (n)

#define USB_LONG_TO_LITTLE_ENDIAN_ADDRESS(n, m)    \
    {                                              \
        m[3] = ((((uint32_t)(n)) >> 24U) & 0xFFU); \
        m[2] = ((((uint32_t)(n)) >> 16U) & 0xFFU); \
        m[1] = ((((uint32_t)(n)) >> 8U) & 0xFFU);  \
        m[0] = (((uint32_t)(n)) & 0xFFU);          \
    }

#define USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS(n)                                                  \
    ((uint32_t)((((uint8_t)n[3]) << 24U) | (((uint8_t)n[2]) << 16U) | (((uint8_t)n[1]) << 8U) | \
                (((uint8_t)n[0]) << 0U)))

#define USB_LONG_TO_BIG_ENDIAN_ADDRESS(n, m)       \
    {                                              \
        m[0] = ((((uint32_t)(n)) >> 24U) & 0xFFU); \
        m[1] = ((((uint32_t)(n)) >> 16U) & 0xFFU); \
        m[2] = ((((uint32_t)(n)) >> 8U) & 0xFFU);  \
        m[3] = (((uint32_t)(n)) & 0xFFU);          \
    }

#define USB_LONG_FROM_BIG_ENDIAN_ADDRESS(n)                                                     \
    ((uint32_t)((((uint8_t)n[0]) << 24U) | (((uint8_t)n[1]) << 16U) | (((uint8_t)n[2]) << 8U) | \
                (((uint8_t)n[3]) << 0U)))

#define USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(n, m)  \
    {                                             \
        m[1] = ((((uint16_t)(n)) >> 8U) & 0xFFU); \
        m[0] = (((uint16_t)(n)) & 0xFFU);         \
    }

#define USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(n) ((uint32_t)((((uint8_t)n[1]) << 8U) | (((uint8_t)n[0]) << 0U)))

#define USB_SHORT_TO_BIG_ENDIAN_ADDRESS(n, m)     \
    {                                             \
        m[0] = ((((uint16_t)(n)) >> 8U) & 0xFFU); \
        m[1] = (((uint16_t)(n)) & 0xFFU);         \
    }

#define USB_SHORT_FROM_BIG_ENDIAN_ADDRESS(n) ((uint32_t)((((uint8_t)n[0]) << 8U) | (((uint8_t)n[1]) << 0U)))

#define USB_LONG_TO_LITTLE_ENDIAN_DATA(n, m)                         \
    {                                                                \
        *((uint8_t *)&(m) + 3) = ((((uint32_t)(n)) >> 24U) & 0xFFU); \
        *((uint8_t *)&(m) + 2) = ((((uint32_t)(n)) >> 16U) & 0xFFU); \
        *((uint8_t *)&(m) + 1) = ((((uint32_t)(n)) >> 8U) & 0xFFU);  \
        *((uint8_t *)&(m) + 0) = (((uint32_t)(n)) & 0xFFU);          \
    }

#define USB_LONG_FROM_LITTLE_ENDIAN_DATA(n)                                             \
    ((uint32_t)(((*((uint8_t *)&(n) + 3)) << 24U) | ((*((uint8_t *)&(n) + 2)) << 16U) | \
                ((*((uint8_t *)&(n) + 1)) << 8U) | ((*((uint8_t *)&(n))) << 0U)))

#define USB_SHORT_TO_LITTLE_ENDIAN_DATA(n, m)                       \
    {                                                               \
        *((uint8_t *)&(m) + 1) = ((((uint16_t)(n)) >> 8U) & 0xFFU); \
        *((uint8_t *)&(m)) = ((((uint16_t)(n))) & 0xFFU);           \
    }

#define USB_SHORT_FROM_LITTLE_ENDIAN_DATA(n) ((uint32_t)(((*((uint8_t *)&(n) + 1)) << 8U) | ((*((uint8_t *)&(n))))))

#else

#define USB_SHORT_TO_LITTLE_ENDIAN(n) (n)
#define USB_LONG_TO_LITTLE_ENDIAN(n) (n)
#define USB_SHORT_FROM_LITTLE_ENDIAN(n) (n)
#define USB_LONG_FROM_LITTLE_ENDIAN(n) (n)

#define USB_SHORT_TO_BIG_ENDIAN(n) SWAP2BYTE_CONST(n)
#define USB_LONG_TO_BIG_ENDIAN(n) SWAP4BYTE_CONST(n)
#define USB_SHORT_FROM_BIG_ENDIAN(n) SWAP2BYTE_CONST(n)
#define USB_LONG_FROM_BIG_ENDIAN(n) SWAP4BYTE_CONST(n)

#define USB_LONG_TO_LITTLE_ENDIAN_ADDRESS(n, m)    \
    {                                              \
        m[3] = ((((uint32_t)(n)) >> 24U) & 0xFFU); \
        m[2] = ((((uint32_t)(n)) >> 16U) & 0xFFU); \
        m[1] = ((((uint32_t)(n)) >> 8U) & 0xFFU);  \
        m[0] = (((uint32_t)(n)) & 0xFFU);          \
    }

#define USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS(n)                                                  \
    ((uint32_t)((((uint8_t)n[3]) << 24U) | (((uint8_t)n[2]) << 16U) | (((uint8_t)n[1]) << 8U) | \
                (((uint8_t)n[0]) << 0U)))

#define USB_LONG_TO_BIG_ENDIAN_ADDRESS(n, m)       \
    {                                              \
        m[0] = ((((uint32_t)(n)) >> 24U) & 0xFFU); \
        m[1] = ((((uint32_t)(n)) >> 16U) & 0xFFU); \
        m[2] = ((((uint32_t)(n)) >> 8U) & 0xFFU);  \
        m[3] = (((uint32_t)(n)) & 0xFFU);          \
    }

#define USB_LONG_FROM_BIG_ENDIAN_ADDRESS(n)                                                     \
    ((uint32_t)((((uint8_t)n[0]) << 24U) | (((uint8_t)n[1]) << 16U) | (((uint8_t)n[2]) << 8U) | \
                (((uint8_t)n[3]) << 0U)))

#define USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(n, m)  \
    {                                             \
        m[1] = ((((uint16_t)(n)) >> 8U) & 0xFFU); \
        m[0] = (((uint16_t)(n)) & 0xFFU);         \
    }

#define USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(n) ((uint32_t)((((uint8_t)n[1]) << 8U) | (((uint8_t)n[0]) << 0U)))

#define USB_SHORT_TO_BIG_ENDIAN_ADDRESS(n, m)     \
    {                                             \
        m[0] = ((((uint16_t)(n)) >> 8U) & 0xFFU); \
        m[1] = (((uint16_t)(n)) & 0xFFU);         \
    }

#define USB_SHORT_FROM_BIG_ENDIAN_ADDRESS(n) ((uint32_t)((((uint8_t)n[0]) << 8U) | (((uint8_t)n[1]) << 0U)))

#define USB_LONG_TO_LITTLE_ENDIAN_DATA(n, m)                         \
    {                                                                \
        *((uint8_t *)&(m) + 3) = ((((uint32_t)(n)) >> 24U) & 0xFFU); \
        *((uint8_t *)&(m) + 2) = ((((uint32_t)(n)) >> 16U) & 0xFFU); \
        *((uint8_t *)&(m) + 1) = ((((uint32_t)(n)) >> 8U) & 0xFFU);  \
        *((uint8_t *)&(m) + 0) = (((uint32_t)(n)) & 0xFFU);          \
    }

#define USB_LONG_FROM_LITTLE_ENDIAN_DATA(n)                                             \
    ((uint32_t)(((*((uint8_t *)&(n) + 3)) << 24U) | ((*((uint8_t *)&(n) + 2)) << 16U) | \
                ((*((uint8_t *)&(n) + 1)) << 8U) | ((*((uint8_t *)&(n))) << 0U)))

#define USB_SHORT_TO_LITTLE_ENDIAN_DATA(n, m)                       \
    {                                                               \
        *((uint8_t *)&(m) + 1) = ((((uint16_t)(n)) >> 8U) & 0xFFU); \
        *((uint8_t *)&(m)) = ((((uint16_t)(n))) & 0xFFU);           \
    }

#define USB_SHORT_FROM_LITTLE_ENDIAN_DATA(n) ((uint32_t)(((*((uint8_t *)&(n) + 1)) << 8U) | ((*((uint8_t *)&(n))))))

#endif

/*
 * The following MACROs (USB_GLOBAL, USB_BDT, USB_RAM_ADDRESS_ALIGNMENT, etc) are only used for USB device stack.
 * The USB device global variables are put into the section NonCacheable and m_usb_bdt or the section
 * .bss.NonCacheable and .bss.m_usb_bdt by using the MACRO USB_GLOBAL and USB_BDT. In this way, the USB device
 * global variables can be linked into USB dedicated RAM by USB_STACK_USE_DEDICATED_RAM.
 * The MACRO USB_STACK_USE_DEDICATED_RAM is used to decide the USB stack uses dedicated RAM or not. The value of
 * the marco can be set as 0, USB_STACK_DEDICATED_RAM_TYPE_BDT_GLOBAL, or USB_STACK_DEDICATED_RAM_TYPE_BDT.
 * The MACRO USB_STACK_DEDICATED_RAM_TYPE_BDT_GLOBAL means USB device global variables, including USB_BDT and
 * USB_GLOBAL, are put into the USB dedicated RAM. This feature can only be enabled when the USB dedicated RAM
 * is not less than 2K Bytes.
 * The MACRO USB_STACK_DEDICATED_RAM_TYPE_BDT means USB device global variables, only including USB_BDT, are put
 * into the USB dedicated RAM, the USB_GLOBAL will be put into .bss section. This feature is used for some SOCs,
 * the USB dedicated RAM size is not more than 512 Bytes.
 */
#define USB_STACK_DEDICATED_RAM_TYPE_BDT_GLOBAL 1
#define USB_STACK_DEDICATED_RAM_TYPE_BDT 2

#define USB_WEAK_VAR __attribute__((weak))
#define USB_WEAK_FUN __attribute__((weak))
#define USB_RAM_ADDRESS_ALIGNMENT(n) __attribute__((aligned(n)))
#define USB_LINK_DMA_INIT_DATA(sec) __attribute__((section(#sec)))
#define USB_LINK_USB_GLOBAL __attribute__((section(".NonCacheable, \"aw\", %nobits @")))
#define USB_LINK_USB_BDT __attribute__((section("m_usb_bdt, \"aw\", %nobits @")))
#define USB_LINK_USB_GLOBAL_BSS __attribute__((section(".bss.NonCacheable, \"aw\", %nobits @")))
#define USB_LINK_USB_BDT_BSS __attribute__((section(".bss.m_usb_bdt, \"aw\", %nobits @")))
#define USB_LINK_DMA_NONINIT_DATA __attribute__((section("m_usb_dma_noninit_data, \"aw\", %nobits @")))
#define USB_LINK_NONCACHE_NONINIT_DATA __attribute__((section("NonCacheable, \"aw\", %nobits @")))

#define USB_CACHE_LINESIZE 4

#define USB_DATA_ALIGN 4

#define USB_DATA_ALIGN_SIZE MAX(USB_CACHE_LINESIZE, USB_DATA_ALIGN)

#define USB_DATA_ALIGN_SIZE_MULTIPLE(n) ((n + USB_DATA_ALIGN_SIZE - 1) & (~(USB_DATA_ALIGN_SIZE - 1)))

#define USB_GLOBAL
#define USB_BDT
#define USB_DMA_DATA_NONINIT_SUB
#define USB_DMA_DATA_INIT_SUB
#define USB_CONTROLLER_DATA USB_LINK_USB_GLOBAL

#define USB_DMA_NONINIT_DATA_ALIGN(n) USB_RAM_ADDRESS_ALIGNMENT(n) USB_DMA_DATA_NONINIT_SUB
#define USB_DMA_INIT_DATA_ALIGN(n) USB_RAM_ADDRESS_ALIGNMENT(n) USB_DMA_DATA_INIT_SUB

#define USB_DMA_DATA_NONCACHEABLE

#define USB_GLOBAL_DEDICATED_RAM USB_LINK_USB_GLOBAL

/* #define USB_RAM_ADDRESS_NONCACHEREG_ALIGNMENT(n, var) AT_NONCACHEABLE_SECTION_ALIGN(var, n) */
/* #define USB_RAM_ADDRESS_NONCACHEREG(var) AT_NONCACHEABLE_SECTION(var) */

#endif /* __USB_MISC_H__ */
