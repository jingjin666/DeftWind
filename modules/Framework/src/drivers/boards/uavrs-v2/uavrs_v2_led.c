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
  bool ledoff = false;

  switch (led)
    {
      case 0:  /* LED Off */
        ledoff = true;
        break;

      case 2:  /* LED No change */
        return;

      case 1:  /* LED On */
      case 3:  /* LED On */
        break;
    }

  imxrt_gpio_write(GPIO_LED, ledoff); /* Low illuminates */
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
  switch (led)
    {
      case 0:  /* LED Off */
      case 1:  /* LED Off */
      case 3:  /* LED Off */
        break;

      case 2:  /* LED No change */
        return;
    }

  imxrt_gpio_write(GPIO_LED, true); /* Low illuminates */
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
	if (led == 1) {
		if (imxrt_gpio_read(GPIO_LED)) {
			imxrt_gpio_write(GPIO_LED, false);

		} else {
			imxrt_gpio_write(GPIO_LED, true);
		}
	}
}

