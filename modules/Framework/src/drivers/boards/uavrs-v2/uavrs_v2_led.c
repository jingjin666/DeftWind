/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/
 
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/board.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include "board_config.h"

#include <arch/board/board.h>

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS


/****************************************************************************
 * Name: led_init
 *
 * Description:
 *   Initialize NuttX-controlled LED logic
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

__EXPORT void led_init(void)
{
  /* Configure LED GPIO for output */

  imxrt_config_gpio(GPIO_LED);
}

/****************************************************************************
 * Name: led_on
 *
 * Description:
 *   Turn on the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

__EXPORT void led_on(int led)
{
  imxrt_gpio_write(GPIO_LED, true); /* High illuminates */
}

/****************************************************************************
 * Name: led_off
 *
 * Description:
 *   Turn off the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

__EXPORT void led_off(int led)
{
  imxrt_gpio_write(GPIO_LED, false); /* Low illuminates */
}

/****************************************************************************
 * Name: led_toggle
 *
 * Description:
 *   Toggle the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
__EXPORT void led_toggle(int led)
{
    if (imxrt_gpio_read(GPIO_LED)) {
    	imxrt_gpio_write(GPIO_LED, false);
    } else {
    	imxrt_gpio_write(GPIO_LED, true);
    }
}

