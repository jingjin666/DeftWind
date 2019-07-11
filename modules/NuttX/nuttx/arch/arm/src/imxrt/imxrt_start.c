/****************************************************************************
 * arch/arm/src/imxrt/imxrt_start.c
 *
 *   Copyright (C) 2018, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/cache.h>
#include <nuttx/init.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "barriers.h"
#include "nvic.h"

#include "imxrt_clockconfig.h"
#include "imxrt_mpuinit.h"
#include "imxrt_userspace.h"
#include "imxrt_start.h"
#include "imxrt_gpio.h"

#ifdef CONFIG_ARMV7M_DTCM
#include "imxrt_periphclks.h"
#include "imxrt_iomuxc.h"

#define IOMUXC_GPR_GPR16_INIT_ITCM_EN_MASK       (0x1U)
#define IOMUXC_GPR_GPR16_INIT_ITCM_EN_SHIFT      (0U)
#define IOMUXC_GPR_GPR16_INIT_ITCM_EN(x)         (((uint32_t)(((uint32_t)(x)) << IOMUXC_GPR_GPR16_INIT_ITCM_EN_SHIFT)) & IOMUXC_GPR_GPR16_INIT_ITCM_EN_MASK)
#define IOMUXC_GPR_GPR16_INIT_DTCM_EN_MASK       (0x2U)
#define IOMUXC_GPR_GPR16_INIT_DTCM_EN_SHIFT      (1U)
#define IOMUXC_GPR_GPR16_INIT_DTCM_EN(x)         (((uint32_t)(((uint32_t)(x)) << IOMUXC_GPR_GPR16_INIT_DTCM_EN_SHIFT)) & IOMUXC_GPR_GPR16_INIT_DTCM_EN_MASK)
#define IOMUXC_GPR_GPR16_FLEXRAM_BANK_CFG_SEL_MASK (0x4U)
#define IOMUXC_GPR_GPR16_FLEXRAM_BANK_CFG_SEL_SHIFT (2U)
#define IOMUXC_GPR_GPR16_FLEXRAM_BANK_CFG_SEL(x) (((uint32_t)(((uint32_t)(x)) << IOMUXC_GPR_GPR16_FLEXRAM_BANK_CFG_SEL_SHIFT)) & IOMUXC_GPR_GPR16_FLEXRAM_BANK_CFG_SEL_MASK)

#define IOMUXC_GPR_GPR14_CM7_CFGITCMSZ_MASK      (0xF0000U)
#define IOMUXC_GPR_GPR14_CM7_CFGITCMSZ_SHIFT     (16U)
#define IOMUXC_GPR_GPR14_CM7_CFGITCMSZ(x)        (((uint32_t)(((uint32_t)(x)) << IOMUXC_GPR_GPR14_CM7_CFGITCMSZ_SHIFT)) & IOMUXC_GPR_GPR14_CM7_CFGITCMSZ_MASK)
#define IOMUXC_GPR_GPR14_CM7_CFGDTCMSZ_MASK      (0xF00000U)
#define IOMUXC_GPR_GPR14_CM7_CFGDTCMSZ_SHIFT     (20U)
#define IOMUXC_GPR_GPR14_CM7_CFGDTCMSZ(x)        (((uint32_t)(((uint32_t)(x)) << IOMUXC_GPR_GPR14_CM7_CFGDTCMSZ_SHIFT)) & IOMUXC_GPR_GPR14_CM7_CFGDTCMSZ_MASK)

/* @brief Bank size */
#define FSL_FEATURE_FLEXRAM_INTERNAL_RAM_BANK_SIZE (32 * 1024)
/* @brief Total Bank numbers */
#define FSL_FEATURE_FLEXRAM_INTERNAL_RAM_TOTAL_BANK_NUMBERS (16)

enum _flexram_tcm_size
{
    kFLEXRAM_TCMSize32KB = 32 * 1024U,   /*!< TCM total size 32KB */
    kFLEXRAM_TCMSize64KB = 64 * 1024U,   /*!< TCM total size 64KB */
    kFLEXRAM_TCMSize128KB = 128 * 1024U, /*!< TCM total size 128KB */
    kFLEXRAM_TCMSize256KB = 256 * 1024U, /*!< TCM total size 256KB */
    kFLEXRAM_TCMSize512KB = 512 * 1024U, /*!< TCM total size 512KB */
};
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map ***************************************************************/
/* 0x2020:0000 - Start of on-chip RAM (OCRAM) and start of .data (_sdata)
 *             - End of .data (_edata) and start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack,
 *               start of heap. NOTE that the ARM uses a decrement before
 *               store stack so that the correct initial value is the end of
 *               the stack + 4;
 * 0x2027:ffff - End of OCRAM and end of heap (assuming 512Kb OCRAM)
 *
 * NOTE:  This assumes that all internal RAM is configured for OCRAM (vs.
 * ITCM or DTCM).  The RAM that holds .data and .bss is called the "Primary
 * RAM".  Many other configurations are possible, including configurations
 * where the primary ram is in external memory.  Those are not considered
 * here.
 */

#define IDLE_STACK ((uintptr_t)&_ebss + CONFIG_IDLETHREAD_STACKSIZE - 4)
#define HEAP_BASE  ((uintptr_t)&_ebss + CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
static inline void imxrt_fpuconfig(void);
#endif
#ifdef CONFIG_STACK_COLORATION
static void go_nx_start(void *pv, unsigned int nbytes)
  __attribute__ ((naked, no_instrument_function, noreturn));
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_STACKCHECK
/* we need to get r10 set before we can allow instrumentation calls */

void __start(void) __attribute__ ((no_instrument_function));
#endif

/****************************************************************************
 * Name: imxrt_fpuconfig
 *
 * Description:
 *   Configure the FPU.  Relative bit settings:
 *
 *     CPACR:  Enables access to CP10 and CP11
 *     CONTROL.FPCA: Determines whether the FP extension is active in the
 *       current context:
 *     FPCCR.ASPEN:  Enables automatic FP state preservation, then the
 *       processor sets this bit to 1 on successful completion of any FP
 *       instruction.
 *     FPCCR.LSPEN:  Enables lazy context save of FP state. When this is
 *       done, the processor reserves space on the stack for the FP state,
 *       but does not save that state information to the stack.
 *
 *  Software must not change the value of the ASPEN bit or LSPEN bit while
 *  either:
 *
 *   - the CPACR permits access to CP10 and CP11, that give access to the FP
 *     extension, or
 *   - the CONTROL.FPCA bit is set to 1
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
#ifndef CONFIG_ARMV7M_LAZYFPU

static inline void imxrt_fpuconfig(void)
{
  uint32_t regval;

  /* Set CONTROL.FPCA so that we always get the extended context frame
   * with the volatile FP registers stacked above the basic context.
   */

  regval = getcontrol();
  regval |= (1 << 2);
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behavior.  Clear FPCCR.ASPEN since we
   * are going to turn on CONTROL.FPCA for all contexts.
   */

  regval = getreg32(NVIC_FPCCR);
  regval &= ~((1 << 31) | (1 << 30));
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval = getreg32(NVIC_CPACR);
  regval |= ((3 << (2*10)) | (3 << (2*11)));
  putreg32(regval, NVIC_CPACR);
}

#else

static inline void imxrt_fpuconfig(void)
{
  uint32_t regval;

  /* Clear CONTROL.FPCA so that we do not get the extended context frame
   * with the volatile FP registers stacked in the saved context.
   */

  regval = getcontrol();
  regval &= ~(1 << 2);
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behavior.  Clear FPCCR.ASPEN since we
   * are going to keep CONTROL.FPCA off for all contexts.
   */

  regval = getreg32(NVIC_FPCCR);
  regval &= ~((1 << 31) | (1 << 30));
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval = getreg32(NVIC_CPACR);
  regval |= ((3 << (2*10)) | (3 << (2*11)));
  putreg32(regval, NVIC_CPACR);
}

#endif

#else
#  define imxrt_fpuconfig()
#endif

/****************************************************************************
 * Name: imxrt_tcmenable
 *
 * Description:
 *   Enable/disable tightly coupled memories.  Size of tightly coupled
 *   memory regions is controlled by GPNVM Bits 7-8.
 *
 ****************************************************************************/

static inline void imxrt_tcmenable(void)
{
  uint32_t regval;

  ARM_DSB();
  ARM_ISB();

  /* Enabled/disabled ITCM */

#ifdef CONFIG_ARMV7M_ITCM
  regval  = NVIC_TCMCR_EN | NVIC_TCMCR_RMW | NVIC_TCMCR_RETEN;
#else
  regval  = getreg32(NVIC_ITCMCR);
  regval &= ~NVIC_TCMCR_EN;
#endif
  putreg32(regval, NVIC_ITCMCR);

  /* Enabled/disabled DTCM */

#ifdef CONFIG_ARMV7M_DTCM
  regval  = NVIC_TCMCR_EN | NVIC_TCMCR_RMW | NVIC_TCMCR_RETEN;
#else
  regval  = getreg32(NVIC_DTCMCR);
  regval &= ~NVIC_TCMCR_EN;
#endif
  putreg32(regval, NVIC_DTCMCR);

  ARM_DSB();
  ARM_ISB();

#ifdef CONFIG_ARMV7M_ITCM
  /* Copy TCM code from flash to ITCM */

#warning Missing logic
#endif
}

/****************************************************************************
 * Name: go_nx_start
 *
 * Description:
 *   Set the IDLE stack to the coloration value and jump into nx_start()
 *
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static void go_nx_start(void *pv, unsigned int nbytes)
{
  /* Set the IDLE stack to the stack coloration value then jump to
   * nx_start().  We take extreme care here because were currently
   * executing on this stack.
   *
   * We want to avoid sneak stack access generated by the compiler.
   */

  __asm__ __volatile__
  (
    "\tmovs r1, r1, lsr #2\n"   /* R1 = nwords = nbytes >> 2 */
    "\tcmp  r1, #0\n"           /* Check (nwords == 0) */
    "\tbeq  2f\n"               /* (should not happen) */

    "\tbic  r0, r0, #3\n"       /* R0 = Aligned stackptr */
    "\tmovw r2, #0xbeef\n"      /* R2 = STACK_COLOR = 0xdeadbeef */
    "\tmovt r2, #0xdead\n"

    "1:\n"                      /* Top of the loop */
    "\tsub  r1, r1, #1\n"       /* R1 nwords-- */
    "\tcmp  r1, #0\n"           /* Check (nwords == 0) */
    "\tstr  r2, [r0], #4\n"     /* Save stack color word, increment stackptr */
    "\tbne  1b\n"               /* Bottom of the loop */

    "2:\n"
    "\tmov  r14, #0\n"          /* LR = return address (none) */
    "\tb    nx_start\n"         /* Branch to nx_start */
  );
}
#endif

#if CONFIG_ARMV7M_DTCM
/****************************************************************************
 * Name: flexram_configure
 *
 * Description:
 *   FLEXRAM map TCM size to register value.
 *
 ****************************************************************************/

static uint8_t flexram_map_tcm_size_to_register(uint8_t tcm_bank_num)
{
    uint8_t tcm_size_config = 0U;

    switch (tcm_bank_num * FSL_FEATURE_FLEXRAM_INTERNAL_RAM_BANK_SIZE)
    {
        case kFLEXRAM_TCMSize32KB:
          tcm_size_config = 6U;
          break;

        case kFLEXRAM_TCMSize64KB:
          tcm_size_config = 7U;
          break;

        case kFLEXRAM_TCMSize128KB:
          tcm_size_config = 8U;
          break;

        case kFLEXRAM_TCMSize256KB:
          tcm_size_config = 9U;
          break;

        case kFLEXRAM_TCMSize512KB:
          tcm_size_config = 10U;
          break;

        default:
          break;
    }

    return tcm_size_config;
}

/****************************************************************************
 * Name: flexram_configure
 *
 * Description:
 *   This function is used to set the TCM to the actual size.
 *   When access to the TCM memory boundary ,hardfault will raised by core.
 *
 ****************************************************************************/

static void flexram_configure()
{
    imxrt_clockall_flexram();

    up_irq_disable();

    /*
    * OCRAM | DTCM | ITCM
    * [KB]    [KB]   [KB]
    *  128    256    128
    */
    *((uint32_t *)IMXRT_IOMUXC_GPR_GPR17) = 0x5AAFFAA5;

    // dtcm
    uint8_t dtcm_bank_num = 256/32;
    *((uint32_t *)IMXRT_IOMUXC_GPR_GPR14) &= ~IOMUXC_GPR_GPR14_CM7_CFGDTCMSZ_MASK;
    *((uint32_t *)IMXRT_IOMUXC_GPR_GPR14) |= IOMUXC_GPR_GPR14_CM7_CFGDTCMSZ(flexram_map_tcm_size_to_register(dtcm_bank_num));
    *((uint32_t *)IMXRT_IOMUXC_GPR_GPR16) |= IOMUXC_GPR_GPR16_INIT_DTCM_EN_MASK;

    // itcm
    uint8_t itcm_bank_num = 128/32;
    *((uint32_t *)IMXRT_IOMUXC_GPR_GPR14) &= ~IOMUXC_GPR_GPR14_CM7_CFGITCMSZ_MASK;
    *((uint32_t *)IMXRT_IOMUXC_GPR_GPR14) |= IOMUXC_GPR_GPR14_CM7_CFGITCMSZ(flexram_map_tcm_size_to_register(itcm_bank_num));
    *((uint32_t *)IMXRT_IOMUXC_GPR_GPR16) |= IOMUXC_GPR_GPR16_INIT_ITCM_EN_MASK;

    // select source
    *((uint32_t *)IMXRT_IOMUXC_GPR_GPR16) &= ~IOMUXC_GPR_GPR16_FLEXRAM_BANK_CFG_SEL_MASK;
    *((uint32_t *)IMXRT_IOMUXC_GPR_GPR16) |= IOMUXC_GPR_GPR16_FLEXRAM_BANK_CFG_SEL(1);

    up_irq_enable();
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/

void __start(void)
{
  const uint32_t *src;
  uint32_t *dest;

#if CONFIG_ARMV7M_DTCM
  flexram_configure();
#endif

#ifdef CONFIG_ARMV7M_STACKCHECK
  /* Set the stack limit before we attempt to call any functions */

  __asm__ volatile ("sub r10, sp, %0" : : "r" (CONFIG_IDLETHREAD_STACKSIZE - 64) : );
#endif

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in OCRAM.  The correct place in OCRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = &_eronly, dest = &_sdata; dest < &_edata; )
    {
      *dest++ = *src++;
    }

  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in OCRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs.  This should be done before imxrt_clockconfig() is
   * called (in case it has some dependency on initialized C variables).
   */

#ifdef CONFIG_ARCH_RAMFUNCS
  for (src = &_framfuncs, dest = &_sramfuncs; dest < &_eramfuncs; )
    {
      *dest++ = *src++;
    }
#endif

  /* Configure the UART so that we can get debug output as soon as possible */

  imxrt_clockconfig();
  imxrt_fpuconfig();
  imxrt_lowsetup();

  /* Enable/disable tightly coupled memories */

  imxrt_tcmenable();

  /* Initialize onboard resources */

  imxrt_boardinitialize();

#ifdef CONFIG_ARM_MPU
#ifdef CONFIG_BUILD_PROTECTED
  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

  imxrt_userspace();
#endif

  /* Configure the MPU to permit user-space access to its FLASH and RAM (for
   * CONFIG_BUILD_PROTECTED) or to manage cache properties in external
   * memory regions.
   */

  imxrt_mpu_initialize();
#endif

  /* Enable I- and D-Caches */

  up_enable_icache();
  up_enable_dcache();

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif

  /* Then start NuttX */

#ifdef CONFIG_STACK_COLORATION
  /* Set the IDLE stack to the coloration value and jump into nx_start() */

  go_nx_start((FAR void *)&_ebss, CONFIG_IDLETHREAD_STACKSIZE);
#else
  /* Call nx_start() */

  nx_start();

  /* Shouldn't get here */

  for (; ; );
#endif
}
