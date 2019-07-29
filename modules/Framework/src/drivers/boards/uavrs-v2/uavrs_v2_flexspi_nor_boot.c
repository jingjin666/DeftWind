/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "uavrs_v2_flexspi_nor_boot.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

__attribute__((section(".boot_hdr.ivt")))
const struct ivt_s image_vector_table =
{
  IVT_HEADER,                         /* IVT Header */
  IMAGE_ENTRY_ADDRESS,                         /* Image  Entry Function */
  IVT_RSVD,                           /* Reserved = 0 */
  (uint32_t)DCD_ADDRESS,              /* Address where DCD information is stored */
  (uint32_t)BOOT_DATA_ADDRESS,        /* Address where BOOT Data Structure is stored */
  (uint32_t)&image_vector_table,      /* Pointer to IVT Self (absolute address */
  (uint32_t)CSF_ADDRESS,              /* Address where CSF file is stored */
  IVT_RSVD                            /* Reserved = 0 */
};

__attribute__((section(".boot_hdr.boot_data")))
const struct boot_data_s boot_data =
{
  FLASH_BASE,                         /* boot start location */
  (FLASH_END - FLASH_BASE),           /* size */
  PLUGIN_FLAG,                        /* Plugin flag*/
  0xFFFFFFFF                          /* empty - extra data word */
};

