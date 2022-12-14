/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *   AP_BoardConfig - px4 driver loading and setup
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_BoardConfig.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_UAVRS
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_sbus.h>
#include <nuttx/arch.h>
#include <spawn.h>

extern const AP_HAL::HAL& hal;

AP_BoardConfig::px4_board_type AP_BoardConfig::px4_configured_board;

/* 
   declare driver main entry points
 */
extern "C" {
    int fmu_main(int, char **);
    int px4io_main(int, char **);
    int adc_main(int, char **);
    int tone_alarm_main(int, char **);
};

/*
  setup PWM pins
 */
void AP_BoardConfig::px4_setup_pwm()
{
    /* configure the FMU driver for the right number of PWMs */
    static const struct {
        uint8_t mode_parm;
        uint8_t mode_value;
        uint8_t num_gpios;
    } mode_table[] = {
        /* table mapping BRD_PWM_COUNT to ioctl arguments */
        { 0, PWM_SERVO_MODE_NONE, 6 },
        { 2, PWM_SERVO_MODE_2PWM, 4 },
        { 4, PWM_SERVO_MODE_4PWM, 2 },
        { 6, PWM_SERVO_MODE_6PWM, 0 },
        { 7, PWM_SERVO_MODE_3PWM1CAP, 2 },
#if defined(CONFIG_ARCH_BOARD_UAVRS_V1)
        { 8, PWM_SERVO_MODE_12PWM, 0 },
#elif defined(CONFIG_ARCH_BOARD_UAVRS_V2)
        { 8, PWM_SERVO_MODE_12PWM, 0 },
        { 15,PWM_SERVO_MODE_15PWM, 0 },
#endif
    };
    uint8_t mode_parm = (uint8_t)px4.pwm_count.get();
    uint8_t i;
    for (i=0; i<ARRAY_SIZE(mode_table); i++) {
        if (mode_table[i].mode_parm == mode_parm) {
            break;
        }
    }
    
    if (i == ARRAY_SIZE(mode_table)) {
        hal.console->printf("RCOutput: invalid BRD_PWM_COUNT %u\n", mode_parm);
    } else {
        int fd = open("/dev/fmu", 0);
        if (fd == -1) {
            AP_HAL::panic("Unable to open /dev/fmu");
        }
        if (ioctl(fd, PWM_SERVO_SET_MODE, mode_table[i].mode_value) != 0) {
            hal.console->printf("RCOutput: unable to setup AUX PWM with BRD_PWM_COUNT %u\n", mode_parm);
        }
        close(fd);
    }
}

/*
  setup flow control on UARTs
 */
void AP_BoardConfig::px4_setup_uart()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_UAVRS
    /* Enable uart mavlink flow control */
    if (hal.uartE != nullptr) {
        hal.uartE->set_flow_control((AP_HAL::UARTDriver::flow_control)px4.ser1_rtscts.get());
    }
    if (hal.uartF != nullptr) {
        hal.uartF->set_flow_control((AP_HAL::UARTDriver::flow_control)px4.ser2_rtscts.get());
    }
#endif
}

/*
  setup safety switch
 */
void AP_BoardConfig::px4_setup_safety_mask()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_UAVRS
    // setup channels to ignore the armed state
    int fmu_fd = open("/dev/fmu", 0);
    if (fmu_fd != -1) {
        uint16_t mask = px4.ignore_safety_channels;
        if (ioctl(fmu_fd, PWM_SERVO_IGNORE_SAFETY, (uint16_t)mask) != 0) {
            hal.console->printf("IGNORE_SAFETY failed\n");
        }
        close(fmu_fd);
    }
#endif
}

/*
  init safety state
 */
void AP_BoardConfig::px4_init_safety()
{
    if (px4.safety_enable.get() == 0) {
        hal.rcout->force_safety_off();
        hal.rcout->force_safety_no_wait();
        // wait until safety has been turned off
        uint8_t count = 20;
        while (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_ARMED && count--) {
            hal.scheduler->delay(20);
        }
    }
}

/*
  setup SBUS
 */
void AP_BoardConfig::px4_setup_sbus(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_UAVRS
    if (px4.sbus_out_rate.get() >= 1) {
        static const struct {
            uint8_t value;
            uint16_t rate;
        } rates[] = {
            { 1, 50 },
            { 2, 75 },
            { 3, 100 },
            { 4, 150 },
            { 5, 200 },
            { 6, 250 },
            { 7, 300 }
        };
        uint16_t rate = 300;
        for (uint8_t i=0; i<ARRAY_SIZE(rates); i++) {
            if (rates[i].value == px4.sbus_out_rate) {
                rate = rates[i].rate;
            }
        }
        if (!hal.rcout->enable_sbus_out(rate)) {
            hal.console->printf("Failed to enable SBUS out\n");
        }
    }
#endif
}

extern "C" int waitpid(pid_t, int *, int);

/*
  start one px4 driver
 */
bool AP_BoardConfig::px4_start_driver(main_fn_t main_function, const char *name, const char *arguments)
{
    char *s = strdup(arguments);
    char *args[10];
    uint8_t nargs = 0;
    char *saveptr = nullptr;

    // parse into separate arguments
    for (char *tok=strtok_r(s, " ", &saveptr); tok; tok=strtok_r(nullptr, " ", &saveptr)) {
        args[nargs++] = tok;
        if (nargs == ARRAY_SIZE(args)-1) {
            break;
        }
    }
    args[nargs++] = nullptr;

    printf("Starting driver %s %s\n", name, arguments);
    pid_t pid;

    if (task_spawn(&pid, name, main_function, nullptr, nullptr,
                   args, nullptr) != 0) {
        free(s);
        printf("Failed to spawn %s\n", name);
        return false;
    }

    // wait for task to exit and gather status
    int status = -1;
    if (waitpid(pid, &status, 0) != pid) {
        printf("waitpid failed for %s\n", name);
        free(s);
        return false;
    }
    free(s);
    return (status >> 8) == 0;
}

void AP_BoardConfig::px4_setup_drivers(void)
{
    // run board auto-detection
    px4_autodetect();

    px4_configured_board = (enum px4_board_type)px4.board_type.get();

    switch (px4_configured_board) {
	case PX4_BOARD_UAVRS:
        printf("dp_setup_drivers:: DP_BOARD_UAVRS\n");
        break;
    default:
        sensor_config_error("Unknown board type");
        break;
    }
}

/*
  play a tune
 */
void AP_BoardConfig::px4_tone_alarm(const char *tone_string)
{
    px4_start_driver(tone_alarm_main, "tone_alarm", tone_string);
}

/*
  setup px4io, possibly updating firmware
 */
void AP_BoardConfig::px4_setup_px4io(void)
{
    if (px4_start_driver(px4io_main, "px4io", "start norc")) {
        printf("px4io started OK\n");
        px4_tone_alarm("MBABGP");
    } else {
        // might be in bootloader mode if user held down safety switch
        // at power on
        printf("Loading /etc/px4io/px4io.bin\n");
        px4_tone_alarm("MBABGP");
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
        // we need to close uartC to prevent conflict between bootloader and
        // uartC reada
        hal.uartC->end();
#endif
        if (px4_start_driver(px4io_main, "px4io", "update /etc/px4io/px4io.bin")) {
            printf("upgraded PX4IO firmware OK\n");
            px4_tone_alarm("MSPAA");
        } else {
            printf("Failed to upgrade PX4IO firmware\n");
            px4_tone_alarm("MNGGG");
        }
        hal.scheduler->delay(1000);
        if (px4_start_driver(px4io_main, "px4io", "start norc")) {
            printf("px4io started OK\n");
        } else {
            sensor_config_error("px4io start failed");
        }
    }

    /*
      see if we need to update px4io firmware
     */
    if (px4_start_driver(px4io_main, "px4io", "checkcrc /etc/px4io/px4io.bin")) {
        printf("PX4IO CRC OK\n");
    } else {
        printf("PX4IO CRC failure\n");
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
        // we need to close uartC to prevent conflict between bootloader and
        // uartC reada
        hal.uartC->end();
#endif
        px4_tone_alarm("MBABGP");
        if (px4_start_driver(px4io_main, "px4io", "safety_on")) {
            printf("PX4IO disarm OK\n");
        } else {
            printf("PX4IO disarm failed\n");
        }
        hal.scheduler->delay(1000);

        if (px4_start_driver(px4io_main, "px4io", "forceupdate 14662 /etc/px4io/px4io.bin")) {
            hal.scheduler->delay(1000);
            if (px4_start_driver(px4io_main, "px4io", "start norc")) {
                printf("px4io restart OK\n");
                px4_tone_alarm("MSPAA");
            } else {
                px4_tone_alarm("MNGGG");
                sensor_config_error("PX4IO restart failed");
            }
        } else {
            printf("PX4IO update failed\n");
            px4_tone_alarm("MNGGG");
        }
    }
}

/*
  setup required peripherals like adc, rcinput and rcoutput
 */
void AP_BoardConfig::px4_setup_peripherals(void)
{
    // always start adc
    if (px4_start_driver(adc_main, "adc", "start")) {
        hal.analogin->init();
        printf("ADC started OK\n");
    } else {
        sensor_config_error("no ADC found");
    }

    const char *fmu_mode = "mode_pwm4";

    if (px4_start_driver(fmu_main, "fmu", fmu_mode)) {
        printf("fmu %s started OK\n", fmu_mode);
    } else {
        sensor_config_error("fmu start failed");
    }

    hal.gpio->init();
    hal.rcin->init();
    hal.rcout->init();
}

/*
  check a SPI device for a register value
 */
bool AP_BoardConfig::spi_check_register(const char *devname, uint8_t regnum, uint8_t value, uint8_t read_flag)
{
    auto dev = hal.spi->get_device(devname);
    if (!dev) {
        printf("%s: no device\n", devname);
        return false;
    }
    dev->set_read_flag(read_flag);
    uint8_t v;
    if (!dev->read_registers(regnum, &v, 1)) {
        printf("%s: reg %02x read fail\n", devname, (unsigned)regnum);
        return false;
    }
    printf("%s: reg %02x %02x %02x\n", devname, (unsigned)regnum, (unsigned)value, (unsigned)v);
    return v == value;
}

#define MPUREG_WHOAMI 0x75
#define MPU_WHOAMI_MPU60X0  0x68
#define MPU_WHOAMI_MPU9250  0x71
#define MPU_WHOAMI_ICM20608 0xaf
#define MPU_WHOAMI_ICM20602 0x12

#define LSMREG_WHOAMI 0x0f
#define LSM_WHOAMI_LSM303D 0x49

/*
  validation of the board type
 */
void AP_BoardConfig::validate_board_type(void)
{
    /* some boards can be damaged by the user setting the wrong board
       type.  The key one is the cube which has a heater which can
       cook the IMUs if the user uses an old paramater file. We
       override the board type for that specific case
     */
}

/*
  auto-detect board type
 */
void AP_BoardConfig::px4_autodetect(void)
{
    if (px4.board_type != PX4_BOARD_AUTO) {
        validate_board_type();
        // user has chosen a board type
        return;
    }

#if defined(CONFIG_ARCH_BOARD_UAVRS_V1) || defined(CONFIG_ARCH_BOARD_UAVRS_V2)
    // UAVRS has ADIS16375 and xxx on external bus
    px4.board_type.set_and_notify(PX4_BOARD_UAVRS);
    hal.console->printf("Detected UAVRS\n");
#endif

}

/*
  setup px4 peripherals and drivers
 */
void AP_BoardConfig::px4_setup()
{
    px4_setup_peripherals();
    px4_setup_pwm();
    px4_setup_safety_mask();
    px4_setup_uart();
    px4_setup_sbus();
    px4_setup_drivers();
}

#endif // HAL_BOARD_UAVRS
