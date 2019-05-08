/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file fmu.cpp
 *
 * Driver/configurator for the DP FMU multi-purpose port on v1 and v2 boards.
 */

#include <dp_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>

#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>

#include <board_config.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/pwm_limit/pwm_limit.h>
#include <systemlib/board_serial.h>
//#include <systemlib/param/param.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_input_capture.h>

#include <lib/rc/sbus.h>
#include <lib/rc/dsm.h>
#include <lib/rc/st24.h>
#include <lib/rc/sumd.h>
#include <lib/rc/srxl.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/safety.h>


#ifdef HRT_PPM_CHANNEL
# include <systemlib/ppm_decode.h>
#endif

//#include <systemlib/circuit_breaker.h>

#define SCHEDULE_INTERVAL	2000	/**< The schedule interval in usec (500 Hz) */
#define NAN_VALUE	(0.0f/0.0f)		/**< NaN value for throttle lock mode */
#define CYCLE_COUNT 10			/* safety switch must be held for 1 second to activate */

/*
 * Define the various LED flash sequences for each system state.
 */
#define LED_PATTERN_FMU_OK_TO_ARM 		0x0003		/**< slow blinking			*/
#define LED_PATTERN_FMU_REFUSE_TO_ARM 	0x5555		/**< fast blinking			*/
#define LED_PATTERN_IO_ARMED 			0x5050		/**< long off, then double blink 	*/
#define LED_PATTERN_FMU_ARMED 			0x5500		/**< long off, then quad blink 		*/
#define LED_PATTERN_IO_FMU_ARMED 		0xffff		/**< constantly on			*/

class FMU : public device::CDev
{
public:
	enum Mode {
		MODE_NONE,
		MODE_2PWM,
		MODE_2PWM2CAP,
		MODE_3PWM,
		MODE_3PWM1CAP,
		MODE_4PWM,
		MODE_6PWM,
		MODE_8PWM,
		MODE_4CAP,
		MODE_5CAP,
		MODE_6CAP,
		MODE_10PWM,
		MODE_12PWM,
	};
	FMU();
	virtual ~FMU();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t	write(file *filp, const char *buffer, size_t len);

	virtual int	init();

	int		set_mode(Mode mode);
	Mode		get_mode() { return _mode; }

	int		set_pwm_alt_rate(unsigned rate);
	int		set_pwm_alt_channels(uint32_t channels);

	static int	set_i2c_bus_clock(unsigned bus, unsigned clock_hz);

	static void	capture_trampoline(void *context, uint32_t chan_index,
					   hrt_abstime edge_time, uint32_t edge_state,
					   uint32_t overflow);

	void		status(void);

private:
	enum RC_SCAN {
		RC_SCAN_PPM = 0,
		RC_SCAN_SBUS,
		RC_SCAN_DSM,
		RC_SCAN_SUMD,
		RC_SCAN_ST24,
		RC_SCAN_SRXL
	};
	enum RC_SCAN _rc_scan_state = RC_SCAN_SBUS;

	char const *RC_SCAN_STRING[6] = {
		"PPM",
		"SBUS",
		"DSM",
		"SUMD",
		"ST24",
		"SRXL"
	};

	hrt_abstime _rc_scan_begin = 0;
	bool _rc_scan_locked = false;
	bool _report_lock = true;

	hrt_abstime _cycle_timestamp = 0;
	hrt_abstime _last_safety_check = 0;

	static const unsigned _max_actuators = DIRECT_PWM_OUTPUT_CHANNELS;

	Mode		_mode;
	unsigned	_pwm_default_rate;
	unsigned	_pwm_alt_rate;
	uint32_t	_pwm_alt_rate_channels;
	uint8_t		_pwm_clock;
	unsigned	_current_update_rate;
	struct work_s	_work;
	int		_armed_sub;
	int		_param_sub;
	int		_safety_sub;
	struct rc_input_values	_rc_in;
	orb_advert_t	_to_input_rc;
	orb_advert_t	_outputs_pub;
	unsigned	_num_outputs;
	int		_class_instance;
	int		_rcs_fd;
	// allow for RC frames up to 32 bytes. Reading more than the max improves the robustness of the
	// frame detection for non-CRC protocols like SBUS and DSM
	uint8_t _rcs_buf[32];

	volatile bool	_initialized;
	bool		_throttle_armed;
	bool		_pwm_on;
	uint32_t	_pwm_mask;
	bool		_pwm_initialized;
	bool		_servos_armed;

	MixerGroup	*_mixers;

	uint32_t	_groups_required;
	uint32_t	_groups_subscribed;
	int		_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	orb_id_t	_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	pollfd	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	unsigned	_poll_fds_num;

	static pwm_limit_t	_pwm_limit;
	static actuator_armed_s	_armed;
	uint16_t	_failsafe_pwm[_max_actuators];
	uint16_t	_disarmed_pwm[_max_actuators];
	uint16_t	_output_pwm[_max_actuators];
	uint16_t	_min_pwm[_max_actuators];
	uint16_t	_max_pwm[_max_actuators];
	uint16_t	_reverse_pwm_mask;
	unsigned	_num_failsafe_set;
	unsigned	_num_disarmed_set;
	bool		_safety_off;
	bool		_safety_disabled;
	orb_advert_t	_to_safety;
	bool		_oneshot_mode;
	hrt_abstime	_oneshot_delay_till;
	uint16_t	_ignore_safety_mask;

	static bool	arm_nothrottle() { return (_armed.prearmed && !_armed.armed); }

	static void	cycle_trampoline(void *arg);
	void		cycle();
	void		work_start();
	void		work_stop();

	static int	control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);
	void		capture_callback(uint32_t chan_index,
					 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);
	void		subscribe();
	int		set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
	int		set_pwm_clock(unsigned rate_map, unsigned clock_MHz);
	int		pwm_ioctl(file *filp, int cmd, unsigned long arg);
	void		update_pwm_rev_mask();
	void		publish_pwm_outputs(uint16_t *values, size_t numvalues);
	void		update_pwm_out_state(void);
	void		pwm_output_set(unsigned i, unsigned value);

	struct GPIOConfig {
		uint32_t	input;
		uint32_t	output;
		uint32_t	alt;
	};

	static const GPIOConfig	_gpio_tab[];
	static const unsigned	_ngpio;

	void		gpio_reset(void);
	void		sensor_reset(int ms);
	void		peripheral_reset(int ms);
	void		gpio_set_function(uint32_t gpios, int function);
	void		gpio_write(uint32_t gpios, int function);
	uint32_t	gpio_read(void);
	int		gpio_ioctl(file *filp, int cmd, unsigned long arg);

	int		capture_ioctl(file *filp, int cmd, unsigned long arg);

	/* do not allow to copy due to ptr data members */
	FMU(const FMU &);
	FMU operator=(const FMU &);
	void fill_rc_in(uint16_t raw_rc_count,
			uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS],
			hrt_abstime now, bool frame_drop, bool failsafe,
			unsigned frame_drops, int rssi);
	void dsm_bind_ioctl(int dsmMode);
	void set_rc_scan_state(RC_SCAN _rc_scan_state);
	void rc_io_invert();
	void rc_io_invert(bool invert);
	void safety_check_button(void);
};

const FMU::GPIOConfig FMU::_gpio_tab[] = {
#if defined(CONFIG_ARCH_BOARD_UAVRS_V1)
	{GPIO_GPIO0_INPUT,       GPIO_GPIO0_OUTPUT,       0},
	{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,       0},
	{GPIO_GPIO2_INPUT,       GPIO_GPIO2_OUTPUT,       0},
	{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,       0},
	{GPIO_GPIO4_INPUT,       GPIO_GPIO4_OUTPUT,       0},
	{GPIO_GPIO5_INPUT,       GPIO_GPIO5_OUTPUT,       0},
	{GPIO_GPIO6_INPUT,       GPIO_GPIO6_OUTPUT,       0},
	{GPIO_GPIO7_INPUT,       GPIO_GPIO7_OUTPUT,       0},
	{GPIO_GPIO8_INPUT,       GPIO_GPIO8_OUTPUT,       0},
	{GPIO_GPIO9_INPUT,       GPIO_GPIO9_OUTPUT,       0},
	{GPIO_CAMERA_TRIGGER_INPUT,     GPIO_CAMERA_TRIGGER_OUTPUT,     0},
	{GPIO_CAMERA_FEEDBACK_INPUT,    GPIO_CAMERA_FEEDBACK_OUTPUT,    0},
#endif
};

const unsigned		FMU::_ngpio = sizeof(FMU::_gpio_tab) / sizeof(FMU::_gpio_tab[0]);
pwm_limit_t		FMU::_pwm_limit;
actuator_armed_s	FMU::_armed = {};

namespace
{

FMU	*g_fmu;

} // namespace

FMU::FMU() :
	CDev("fmu", FMU_DEVICE_PATH),
	_mode(MODE_NONE),
	_pwm_default_rate(50),
	_pwm_alt_rate(50),
	_pwm_alt_rate_channels(0),
	_pwm_clock(1),
	_current_update_rate(0),
	_work{},
	_armed_sub(-1),
	_param_sub(-1),
	_safety_sub(-1),
	_rc_in{},
	_to_input_rc(nullptr),
	_outputs_pub(nullptr),
	_num_outputs(0),
	_class_instance(0),
	_rcs_fd(-1),
	_initialized(false),
	_throttle_armed(false),
	_pwm_on(false),
	_pwm_mask(0),
	_pwm_initialized(false),
	_servos_armed(false),
	_mixers(nullptr),
	_groups_required(0),
	_groups_subscribed(0),
	_control_subs{ -1},
	_poll_fds_num(0),
	_failsafe_pwm{0},
	_disarmed_pwm{0},
	_output_pwm{0},
	_reverse_pwm_mask(0),
	_num_failsafe_set(0),
	_num_disarmed_set(0),
	_safety_off(false),
	_safety_disabled(false),
	_to_safety(nullptr),
	_oneshot_mode(false),
	_oneshot_delay_till(0),
        _ignore_safety_mask(0)
{
	for (unsigned i = 0; i < _max_actuators; i++) {
		_min_pwm[i] = PWM_DEFAULT_MIN;
		_max_pwm[i] = PWM_DEFAULT_MAX;
	}

	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);

	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

	// rc input, published to ORB
	memset(&_rc_in, 0, sizeof(_rc_in));
	_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;

#ifdef GPIO_SBUS_INV
	// this board has a GPIO to control SBUS inversion
	stm32_configgpio(GPIO_SBUS_INV);
#endif

	// If there is no safety button, disable it on boot.
#ifndef GPIO_BTN_SAFETY
	_safety_off = true;
#endif

	/* only enable this during development */
	_debug_enabled = false;
}

FMU::~FMU()
{
	if (_initialized) {
		/* tell the task we want it to go away */
		work_stop();

		int i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);
			i--;

		} while (_initialized && i > 0);
	}

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	g_fmu = nullptr;
}

int
FMU::init()
{
	int ret;

	ASSERT(!_initialized);

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

    /* try to claim the generic PWM output device node as well - it's OK if we fail at this */
    _class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

    if (_class_instance == CLASS_DEVICE_PRIMARY) {
        /* lets not be too verbose */
    } else if (_class_instance < 0) {
        warnx("FAILED registering class device");
        
    }

	_safety_disabled = 0;

	work_start();

	return OK;
}

void
FMU:: safety_check_button(void)
{
}

int
FMU::set_mode(Mode mode)
{
	unsigned old_mask = _pwm_mask;

	/*
	 * Configure for PWM output.
	 *
	 * Note that regardless of the configured mode, the task is always
	 * listening and mixing; the mode just selects which of the channels
	 * are presented on the output pins.
	 */
	switch (mode) {
	case MODE_2PWM2CAP:	// v1 multi-port with flow control lines as PWM
		up_input_capture_set(2, Rising, 0, NULL, NULL);
		up_input_capture_set(3, Rising, 0, NULL, NULL);
		DEVICE_DEBUG("MODE_2PWM2CAP");

	// no break
	case MODE_2PWM:	// v1 multi-port with flow control lines as PWM
		DEVICE_DEBUG("MODE_2PWM");
		_pwm_mask = 0x3;
		_pwm_initialized = false;

		break;

	case MODE_3PWM1CAP:	// v1 multi-port with flow control lines as PWM
		DEVICE_DEBUG("MODE_3PWM1CAP");
		up_input_capture_set(3, Rising, 0, NULL, NULL);

	// no break
	case MODE_3PWM:	// v1 multi-port with flow control lines as PWM
		DEVICE_DEBUG("MODE_3PWM");

		_pwm_mask = 0x7;
		_pwm_initialized = false;
		break;

	case MODE_4PWM: // v1 or v2 multi-port as 4 PWM outs
		DEVICE_DEBUG("MODE_4PWM");

		_pwm_mask = 0xf;
		_pwm_initialized = false;
		break;

	case MODE_6PWM: // v2 PWMs as 6 PWM outs
		DEVICE_DEBUG("MODE_6PWM");

		_pwm_mask = 0x3f;
		_pwm_initialized = false;
		break;

#if defined(CONFIG_ARCH_BOARD_AEROCORE) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V51) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V52) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V54) || defined(CONFIG_ARCH_BOARD_VRCORE_V10) || defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51) || defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52) || defined(CONFIG_ARCH_BOARD_UAVRS_V1)

	case MODE_8PWM: // AeroCore PWMs as 8 PWM outs
		DEVICE_DEBUG("MODE_8PWM");
		_pwm_mask = 0xff;
		_pwm_initialized = false;
		break;
#endif

#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V51) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V52) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V54) || defined(CONFIG_ARCH_BOARD_VRCORE_V10) || defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51) || defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52) || defined(CONFIG_ARCH_BOARD_UAVRS_V1)

	case MODE_10PWM:
		DEVICE_DEBUG("MODE_10PWM");

		_pwm_mask = 0x3ff;
		_pwm_initialized = false;
		break;

	case MODE_12PWM:
		DEVICE_DEBUG("MODE_12PWM");

		_pwm_mask = 0xfff;
		_pwm_initialized = false;
		break;
#endif

	case MODE_NONE:
		DEVICE_DEBUG("MODE_NONE");

		_pwm_default_rate = 10;	/* artificially reduced output rate */
		_pwm_alt_rate = 10;
		_pwm_mask = 0x0;
		_pwm_initialized = false;

		if (old_mask != _pwm_mask) {
			/* disable servo outputs - no need to set rates */
			up_pwm_servo_deinit();
		}

		break;

	default:
		return -EINVAL;
	}

	_mode = mode;
	_pwm_alt_rate_channels &= _pwm_mask;
	return OK;
}

int
FMU::set_pwm_rate(uint32_t rate_map, unsigned default_rate, unsigned alt_rate)
{
	DEVICE_DEBUG("set_pwm_rate 0x%02x %u %u", rate_map, default_rate, alt_rate);

	for (unsigned pass = 0; pass < 2; pass++) {
		for (unsigned group = 0; group < _max_actuators; group++) {

			// get the channel mask for this rate group
			uint32_t mask = up_pwm_servo_get_rate_group(group);

			if (mask == 0) {
				continue;
			}

			// all channels in the group must be either default or alt-rate
			uint32_t alt = rate_map & mask;

			if (pass == 0) {
				// preflight
				if ((alt != 0) && (alt != mask)) {
					warn("rate group %u mask %x bad overlap %x", group, mask, alt);
					// not a legal map, bail
					return -EINVAL;
				}

			} else {
				// set it - errors here are unexpected
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, alt_rate) != OK) {
						warn("rate group set alt failed");
						return -EINVAL;
					}

				} else {
					if (up_pwm_servo_set_rate_group_update(group, default_rate) != OK) {
						warn("rate group set default failed");
						return -EINVAL;
					}
				}
			}
		}
	}

	_pwm_alt_rate_channels = rate_map;
	_pwm_default_rate = default_rate;
	_pwm_alt_rate = alt_rate;

	set_pwm_clock(rate_map, _pwm_clock);

	return OK;
}


int
FMU::set_pwm_clock(uint32_t rate_map, unsigned clock_MHz)
{
    for (unsigned group = 0; group < _max_actuators; group++) {

        // get the channel mask for this rate group
        uint32_t mask = up_pwm_servo_get_rate_group(group);

        if ((rate_map & mask) != 0) {
            #if 0
            if (up_pwm_servo_set_rate_group_clock(group, clock_MHz) != OK) {
                return -EINVAL;
            }
            #endif
        }
	}

    _pwm_clock = clock_MHz;

	return OK;
}

int
FMU::set_pwm_alt_rate(unsigned rate)
{
	return set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, rate);
}

int
FMU::set_pwm_alt_channels(uint32_t channels)
{
	return set_pwm_rate(channels, _pwm_default_rate, _pwm_alt_rate);
}

int
FMU::set_i2c_bus_clock(unsigned bus, unsigned clock_hz)
{
	return device::I2C::set_bus_clock(bus, clock_hz);
}

void
FMU::subscribe()
{
	/* subscribe/unsubscribe to required actuator control groups */
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	_poll_fds_num = 0;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			DEVICE_DEBUG("subscribe to actuator_controls_%d", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}

		if (unsub_groups & (1 << i)) {
			DEVICE_DEBUG("unsubscribe from actuator_controls_%d", i);
			::close(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] > 0) {
			_poll_fds[_poll_fds_num].fd = _control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}
	}
}

void
FMU::update_pwm_rev_mask()
{
	_reverse_pwm_mask = 0;

	for (unsigned i = 0; i < _max_actuators; i++) {
		char pname[16];
		int32_t ival;

		/* fill the channel reverse mask from parameters */
		sprintf(pname, "PWM_AUX_REV%d", i + 1);
        #if 0
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			param_get(param_h, &ival);
			_reverse_pwm_mask |= ((int16_t)(ival != 0)) << i;
		}
		#endif
	}
}

void
FMU::publish_pwm_outputs(uint16_t *values, size_t numvalues)
{
	actuator_outputs_s outputs = {};
	outputs.noutputs = numvalues;
	outputs.timestamp = hrt_absolute_time();

	for (size_t i = 0; i < _max_actuators; ++i) {
		outputs.output[i] = i < numvalues ? (float)values[i] : 0;
	}

	if (_outputs_pub == nullptr) {
		int instance = -1;
		_outputs_pub = orb_advertise_multi(ORB_ID(actuator_outputs), &outputs, &instance, ORB_PRIO_DEFAULT);

	} else {
		orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &outputs);
	}
}


void
FMU::work_start()
{
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&FMU::cycle_trampoline, this, 0);
}

void
FMU::cycle_trampoline(void *arg)
{
	FMU *dev = reinterpret_cast<FMU *>(arg);

	dev->cycle();
}

void
FMU::capture_trampoline(void *context, uint32_t chan_index,
			   hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	FMU *dev = reinterpret_cast<FMU *>(context);
	dev->capture_callback(chan_index, edge_time, edge_state, overflow);
}

void
FMU::capture_callback(uint32_t chan_index,
			 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	fprintf(stdout, "FMU: Capture chan:%d time:%lld state:%d overflow:%d\n", chan_index, edge_time, edge_state, overflow);
}

void
FMU::fill_rc_in(uint16_t raw_rc_count,
		   uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS],
		   hrt_abstime now, bool frame_drop, bool failsafe,
		   unsigned frame_drops, int rssi = -1)
{
	// fill rc_in struct for publishing
	_rc_in.channel_count = raw_rc_count;

	if (_rc_in.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_rc_in.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	for (uint8_t i = 0; i < _rc_in.channel_count; i++) {
		_rc_in.values[i] = raw_rc_values[i];
	}

	_rc_in.timestamp_publication = now;
	_rc_in.timestamp_last_signal = _rc_in.timestamp_publication;
	_rc_in.rc_ppm_frame_length = 0;

	/* fake rssi if no value was provided */
	if (rssi == -1) {
		_rc_in.rssi =
			(!frame_drop) ? RC_INPUT_RSSI_MAX : (RC_INPUT_RSSI_MAX / 2);

	} else {
		_rc_in.rssi = rssi;
	}

	_rc_in.rc_failsafe = failsafe;
	_rc_in.rc_lost = false;
	_rc_in.rc_lost_frame_count = frame_drops;
	_rc_in.rc_total_frame_count = 0;
}

#ifdef RC_SERIAL_PORT
void FMU::set_rc_scan_state(RC_SCAN newState)
{
//    warnx("RCscan: %s failed, trying %s", FMU::RC_SCAN_STRING[_rc_scan_state], FMU::RC_SCAN_STRING[newState]);
	_rc_scan_begin = 0;
	_rc_scan_state = newState;
}

void FMU::rc_io_invert(bool invert)
{
	INVERT_RC_INPUT(invert);

#ifdef GPIO_RC_OUT
	if (!invert) {
		// set FMU_RC_OUTPUT high to pull RC_INPUT up
		stm32_gpiowrite(GPIO_RC_OUT, 1);
	}
#endif
}
#endif

void
FMU::pwm_output_set(unsigned i, unsigned value)
{
	if (_pwm_initialized) {
		up_pwm_servo_set(i, value);
		if (i < _max_actuators) {
			_output_pwm[i] = value;
		}
	}
}

void
FMU::update_pwm_out_state(void)
{
	/* update PWM status if armed or if disarmed PWM values are set */
	bool pwm_on = _servos_armed && (_safety_off || (_num_disarmed_set > 0) || _ignore_safety_mask != 0);

        // cope with setting of _pwm_initialized=false in set_mode
	if (pwm_on && _pwm_on && !_pwm_initialized && _pwm_mask != 0) {
            _pwm_on = false;
        }
        
	if (_pwm_on == pwm_on) {
		return;
	}
	_pwm_on = pwm_on;

	if (_pwm_on && !_pwm_initialized && _pwm_mask != 0) {
		up_pwm_servo_init(_pwm_mask);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);
		_pwm_initialized = true;
	} else {
		up_pwm_servo_deinit();
		_pwm_initialized = false;
	}

	up_pwm_servo_arm(_pwm_on);
}

void
FMU::cycle()
{
	if (!_initialized) {
		/* force a reset of the update rate */
		_current_update_rate = 0;

		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
		//_param_sub = orb_subscribe(ORB_ID(parameter_update));

		/* initialize PWM limit lib */
		pwm_limit_init(&_pwm_limit);

		update_pwm_rev_mask();

#ifdef RC_SERIAL_PORT
		// dsm_init sets some file static variables and returns a file descriptor
		_rcs_fd = dsm_init(RC_SERIAL_PORT);
		// assume SBUS input
		sbus_config(_rcs_fd, false);
		// disable CPPM input by mapping it away from the timer capture input
#ifdef GPIO_PPM_IN
		stm32_unconfiggpio(GPIO_PPM_IN);
#endif
#endif

		_initialized = true;
	}

	if (_safety_sub == -1) {
		_safety_sub = orb_subscribe(ORB_ID(safety));
	}

	if (_groups_subscribed != _groups_required) {
		subscribe();
		_groups_subscribed = _groups_required;
		/* force setting update rate */
		_current_update_rate = 0;
	}

	/*
	 * Adjust actuator topic update rate to keep up with
	 * the highest servo update rate configured.
	 *
	 * We always mix at max rate; some channels may update slower.
	 */
	unsigned max_rate = (_pwm_default_rate > _pwm_alt_rate) ? _pwm_default_rate : _pwm_alt_rate;

	if (_current_update_rate != max_rate) {
		_current_update_rate = max_rate;
		int update_rate_in_ms = int(1000 / _current_update_rate);

		/* reject faster than 500 Hz updates */
		if (update_rate_in_ms < 2) {
			update_rate_in_ms = 2;
		}

		/* reject slower than 10 Hz updates */
		if (update_rate_in_ms > 100) {
			update_rate_in_ms = 100;
		}

		DEVICE_DEBUG("adjusted actuator update interval to %ums", update_rate_in_ms);

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] > 0) {
				orb_set_interval(_control_subs[i], update_rate_in_ms);
			}
		}

		// set to current max rate, even if we are actually checking slower/faster
		_current_update_rate = max_rate;
	}

	/* check if anything updated */
	int ret = ::poll(_poll_fds, _poll_fds_num, 0);

	/* log if main actuator updated and sync */
	int main_out_latency = 0;

	/* this would be bad... */
	if (ret < 0) {
		DEVICE_LOG("poll error %d", errno);

	} else if (ret == 0) {
		/* timeout: no control data, switch to failsafe values */
//			warnx("no PWM: failsafe");

	} else {

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] > 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);

					/* main outputs */
					if (i == 0) {
//						main_out_latency = hrt_absolute_time() - _controls[i].timestamp - 250;
//						warnx("lat: %llu", hrt_absolute_time() - _controls[i].timestamp);

						/* do only correct within the current phase */
						if (abs(main_out_latency) > SCHEDULE_INTERVAL) {
							main_out_latency = SCHEDULE_INTERVAL;
						}

						if (main_out_latency < 250) {
							main_out_latency = 0;
						}
					}
				}

				poll_id++;
			}
		}

		/* can we mix? */
		if (_mixers != nullptr) {

			size_t num_outputs;

			switch (_mode) {
			case MODE_2PWM:
			case MODE_2PWM2CAP:
				num_outputs = 2;
				break;

			case MODE_3PWM:
			case MODE_3PWM1CAP:
				num_outputs = 3;
				break;

			case MODE_4PWM:
				num_outputs = 4;
				break;

			case MODE_6PWM:
				num_outputs = 6;
				break;

			case MODE_8PWM:
				num_outputs = 8;
				break;

			case MODE_10PWM:
				num_outputs = 10;
				break;

			case MODE_12PWM:
				num_outputs = 12;
				break;

			default:
				num_outputs = 0;
				break;
			}

			/* do mixing */
			float outputs[_max_actuators];
			num_outputs = _mixers->mix(outputs, num_outputs, NULL);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++) {
				if (i >= num_outputs) {
					outputs[i] = NAN_VALUE;
				}
			}

			uint16_t pwm_limited[_max_actuators];

			/* the PWM limit call takes care of out of band errors, NaN and constrains */
			pwm_limit_calc(_throttle_armed, arm_nothrottle(), num_outputs, _reverse_pwm_mask, _disarmed_pwm, _min_pwm, _max_pwm,
				       outputs, pwm_limited, &_pwm_limit);

			/* overwrite outputs in case of lockdown with disarmed PWM values */
			if (_armed.lockdown) {
				for (size_t i = 0; i < num_outputs; i++) {
					pwm_limited[i] = _disarmed_pwm[i];
				}
			}

			/* output to the servos */
			for (size_t i = 0; i < num_outputs; i++) {
				pwm_output_set(i, pwm_limited[i]);
			}

			publish_pwm_outputs(pwm_limited, num_outputs);
		}
	}

	_cycle_timestamp = hrt_absolute_time();

#ifdef GPIO_BTN_SAFETY

	if (_cycle_timestamp - _last_safety_check >= (unsigned int)1e5) {
		_last_safety_check = _cycle_timestamp;

		/**
		 * Get and handle the safety status at 10Hz
		 */
		struct safety_s safety = {};

		if (_safety_disabled) {
			/* safety switch disabled, turn LED on solid */
			stm32_gpiowrite(GPIO_LED_SAFETY, 0);
			_safety_off = true;

		} else {
			/* read safety switch input and control safety switch LED at 10Hz */
			safety_check_button();
		}

		safety.timestamp = hrt_absolute_time();

		if (_safety_off) {
			safety.safety_off = true;
			safety.safety_switch_available = true;

		} else {
			safety.safety_off = false;
			safety.safety_switch_available = true;
		}

		/* lazily publish the safety status */
		if (_to_safety != nullptr) {
			orb_publish(ORB_ID(safety), _to_safety, &safety);

		} else {
			_to_safety = orb_advertise(ORB_ID(safety), &safety);
		}

		update_pwm_out_state();
	}

#else
	// slave safety from IO
	if (_cycle_timestamp - _last_safety_check >= (unsigned int)1e5) {
		_last_safety_check = _cycle_timestamp;
		bool updated = false;
		orb_check(_safety_sub, &updated);
		if (updated) {
			static safety_s	_safety;			
			orb_copy(ORB_ID(safety), _safety_sub, &_safety);
			if (_safety.safety_switch_available) {
				_safety_off = _safety.safety_off;
			}
		}
		update_pwm_out_state();
	}
#endif
	
	/* check arming state */
	bool updated = false;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

		/* update the armed status and check that we're not locked down */
		_throttle_armed = _safety_off && _armed.armed && !_armed.lockdown;

		update_pwm_out_state();
	}

	//orb_check(_param_sub, &updated);

	if (updated) {
		parameter_update_s pupdate;
		//orb_copy(ORB_ID(parameter_update), _param_sub, &pupdate);

		update_pwm_rev_mask();

		int32_t dsm_bind_val;
        #if 0
        param_t dsm_bind_param;

		/* see if bind parameter has been set, and reset it to -1 */
		param_get(dsm_bind_param = param_find("RC_DSM_BIND"), &dsm_bind_val);

		if (dsm_bind_val > -1) {
			dsm_bind_ioctl(dsm_bind_val);
			dsm_bind_val = -1;
			param_set(dsm_bind_param, &dsm_bind_val);
		}
        #endif
	}

	bool rc_updated = false;

#ifdef RC_SERIAL_PORT
	// This block scans for a supported serial RC input and locks onto the first one found
	// Scan for 200 msec, then switch protocol
	constexpr hrt_abstime rc_scan_max = 200 * 1000;

	bool sbus_failsafe, sbus_frame_drop;
	uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS];
	uint16_t raw_rc_count;
	unsigned frame_drops;
	bool dsm_11_bit;


	if (_report_lock && _rc_scan_locked) {
		_report_lock = false;
		warnx("RCscan: %s RC input locked", RC_SCAN_STRING[_rc_scan_state]);
	}

	// read all available data from the serial RC input UART
	int newBytes = ::read(_rcs_fd, &_rcs_buf[0], sizeof(_rcs_buf));

	switch (_rc_scan_state) {
	case RC_SCAN_SBUS:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = _cycle_timestamp;
			// Configure serial port for SBUS
			sbus_config(_rcs_fd, false);
			rc_io_invert(true);

		} else if (_rc_scan_locked
			   || _cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			// parse new data
			if (newBytes > 0 && newBytes <= SBUS_FRAME_SIZE) {
				rc_updated = sbus_parse(_cycle_timestamp, &_rcs_buf[0], newBytes, &raw_rc_values[0], &raw_rc_count, &sbus_failsafe,
							&sbus_frame_drop, &frame_drops, input_rc_s::RC_INPUT_MAX_CHANNELS);

				if (rc_updated) {
					// we have a new SBUS frame. Publish it.
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS;
					fill_rc_in(raw_rc_count, raw_rc_values, _cycle_timestamp,
						   sbus_frame_drop, sbus_failsafe, frame_drops);
					_rc_scan_locked = true;
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_ST24);
		}

		break;

	case RC_SCAN_ST24:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = _cycle_timestamp;
			// Configure serial port for DSM
			dsm_config(_rcs_fd);
			rc_io_invert(false);

		} else if (_rc_scan_locked
			   || _cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			if (newBytes > 0) {
				// parse new data
				uint8_t st24_rssi, rx_count;

				rc_updated = false;

				for (unsigned i = 0; i < (unsigned)newBytes; i++) {
					/* set updated flag if one complete packet was parsed */
					st24_rssi = RC_INPUT_RSSI_MAX;
					rc_updated = (OK == st24_decode(_rcs_buf[i], &st24_rssi, &rx_count,
									&raw_rc_count, raw_rc_values, input_rc_s::RC_INPUT_MAX_CHANNELS));
				}

				if (rc_updated) {
					// we have a new ST24 frame. Publish it.
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_ST24;
					fill_rc_in(raw_rc_count, raw_rc_values, _cycle_timestamp,
						   false, false, frame_drops, st24_rssi);
					_rc_scan_locked = true;
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_SUMD);
		}

		break;

	case RC_SCAN_SUMD:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = _cycle_timestamp;
			// Configure serial port for DSM
			dsm_config(_rcs_fd);
			rc_io_invert(false);

		} else if (_rc_scan_locked
			   || _cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			if (newBytes > 0) {
				// parse new data
				uint8_t sumd_rssi, rx_count;

				rc_updated = false;

				for (unsigned i = 0; i < (unsigned)newBytes; i++) {
					/* set updated flag if one complete packet was parsed */
					sumd_rssi = RC_INPUT_RSSI_MAX;
					rc_updated = (OK == sumd_decode(_rcs_buf[i], &sumd_rssi, &rx_count,
									&raw_rc_count, raw_rc_values, input_rc_s::RC_INPUT_MAX_CHANNELS));
				}

				if (rc_updated) {
					// we have a new SUMD frame. Publish it.
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SUMD;
					fill_rc_in(raw_rc_count, raw_rc_values, _cycle_timestamp,
						   false, false, frame_drops, sumd_rssi);
					_rc_scan_locked = true;
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_SRXL);
		}

		break;


	case RC_SCAN_SRXL:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = _cycle_timestamp;
			// Configure serial port for 115200, not inverted
			dsm_config(_rcs_fd);
			rc_io_invert(false);

		} else if (_rc_scan_locked
			   || _cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			if (newBytes > 0) {
				// parse new data
				rc_updated = false;
                                uint8_t srxl_chan_count;
				bool failsafe_state;
                                
				for (unsigned i = 0; i < (unsigned)newBytes; i++) {
					/* set updated flag if one complete packet was parsed */
					rc_updated = (OK == srxl_decode(_cycle_timestamp, _rcs_buf[i],
									&srxl_chan_count, raw_rc_values, input_rc_s::RC_INPUT_MAX_CHANNELS, &failsafe_state));
				}

				if (rc_updated) {
					// we have a new SRXL frame. Publish it.
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SRXL;

					fill_rc_in(srxl_chan_count, raw_rc_values, _cycle_timestamp,
						   false, failsafe_state, 0);
					_rc_scan_locked = true;
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_DSM);
		}

		break;

	case RC_SCAN_DSM:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = _cycle_timestamp;
			// Configure serial port for DSM
			dsm_config(_rcs_fd);
			rc_io_invert(false);

		} else if (_rc_scan_locked
			   || _cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			rc_io_invert(false);

			if (newBytes > 0 && newBytes <= 16) {
				// parse new data
				rc_updated = dsm_parse(_cycle_timestamp, &_rcs_buf[0], newBytes, &raw_rc_values[0], &raw_rc_count,
						       &dsm_11_bit, &frame_drops, input_rc_s::RC_INPUT_MAX_CHANNELS);

				if (rc_updated) {
					// we have a new DSM frame. Publish it.
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM;
					fill_rc_in(raw_rc_count, raw_rc_values, _cycle_timestamp,
						   false, false, frame_drops);
					_rc_scan_locked = true;
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_PPM);
		}

		break;
                
	case RC_SCAN_PPM:
		// skip PPM if it's not supported
#ifdef HRT_PPM_CHANNEL
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = _cycle_timestamp;
			// Configure timer input pin for CPPM
			stm32_configgpio(GPIO_PPM_IN);
			rc_io_invert(false);

		} else if (_rc_scan_locked
			   || _cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			// see if we have new PPM input data
			if ((ppm_last_valid_decode != _rc_in.timestamp_last_signal)
			    && ppm_decoded_channels > 3) {
				// we have a new PPM frame. Publish it.
				rc_updated = true;
				_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
				fill_rc_in(ppm_decoded_channels, ppm_buffer, _cycle_timestamp,
					   false, false, 0);
				_rc_scan_locked = true;
			}

		} else {
			// disable CPPM input by mapping it away from the timer capture input
			stm32_unconfiggpio(GPIO_PPM_IN);
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_SBUS);
		}

#else   // skip PPM if it's not supported
		set_rc_scan_state(RC_SCAN_SBUS);

#endif  // HRT_PPM_CHANNEL

		break;
	}

#else  // RC_SERIAL_PORT not defined
#ifdef HRT_PPM_CHANNEL

	// see if we have new PPM input data
	if ((ppm_last_valid_decode != _rc_in.timestamp_last_signal)
	    && ppm_decoded_channels > 3) {
		// we have a new PPM frame. Publish it.
		rc_updated = true;
		fill_rc_in(ppm_decoded_channels, ppm_buffer, hrt_absolute_time(),
			   false, false, 0);
	}

#endif  // HRT_PPM_CHANNEL
#endif  // RC_SERIAL_PORT

	if (rc_updated) {
		/* lazily advertise on first publication */
		if (_to_input_rc == nullptr) {
			_to_input_rc = orb_advertise(ORB_ID(input_rc), &_rc_in);

		} else {
			orb_publish(ORB_ID(input_rc), _to_input_rc, &_rc_in);
		}
	}

	work_queue(HPWORK, &_work, (worker_t)&FMU::cycle_trampoline, this,
		   USEC2TICK(SCHEDULE_INTERVAL - main_out_latency));
}

void FMU::work_stop()
{
	work_cancel(HPWORK, &_work);

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] > 0) {
			::close(_control_subs[i]);
			_control_subs[i] = -1;
		}
	}

	::close(_armed_sub);
	//::close(_param_sub);

	/* make sure servos are off */
	up_pwm_servo_deinit();

	DEVICE_LOG("stopping");

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_initialized = false;
}

int
FMU::control_callback(uintptr_t handle,
			 uint8_t control_group,
			 uint8_t control_index,
			 float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];

	/* limit control input */
	if (input > 1.0f) {
		input = 1.0f;

	} else if (input < -1.0f) {
		input = -1.0f;
	}

	/* motor spinup phase - lock throttle to zero */
	if (_pwm_limit.state == PWM_LIMIT_STATE_RAMP) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			input = 0.0f;
		}
	}

	/* throttle not arming - mark throttle input as invalid */
	if (arm_nothrottle()) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* set the throttle to an invalid value */
			input = NAN_VALUE;
		}
	}

	return 0;
}

int
FMU::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	/* try it as a GPIO ioctl first */
	ret = gpio_ioctl(filp, cmd, arg);

	if (ret != -ENOTTY) {
		return ret;
	}

	/* try it as a Capture ioctl next */
	ret = capture_ioctl(filp, cmd, arg);

	if (ret != -ENOTTY) {
		return ret;
	}

	/* if we are in valid PWM mode, try it as a PWM ioctl as well */
	switch (_mode) {
	case MODE_2PWM:
	case MODE_3PWM:
	case MODE_4PWM:
	case MODE_2PWM2CAP:
	case MODE_3PWM1CAP:
	case MODE_6PWM:
	case MODE_8PWM:
	case MODE_10PWM:
	case MODE_12PWM:
		ret = pwm_ioctl(filp, cmd, arg);
		break;

	default:
		DEVICE_DEBUG("not in a PWM mode");
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int
FMU::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		_servos_armed = true;
		update_pwm_out_state();
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
		/* force safety switch off */
		_safety_off = true;
		update_pwm_out_state();
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		/* force safety switch on */
		_safety_off = false;
		update_pwm_out_state();
		break;

	case PWM_SERVO_DISARM:
		_servos_armed = false;
		update_pwm_out_state();
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_default_rate;
		break;

	case PWM_SERVO_SET_DEFAULT_UPDATE_RATE:
		ret = set_pwm_rate(_pwm_alt_rate_channels, (uint32_t)arg, _pwm_alt_rate);
		break;
        
	case PWM_SERVO_SET_UPDATE_RATE:
		ret = set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, arg);
		break;

	case PWM_SERVO_SET_UPDATE_CLOCK:
		ret = set_pwm_clock(0xFF, arg);
		break;
        
	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate;
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		ret = set_pwm_rate(arg, _pwm_default_rate, _pwm_alt_rate);
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate_channels;
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_failsafe_pwm[i] = PWM_HIGHEST_MAX;

				} else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_failsafe_pwm[i] = PWM_LOWEST_MIN;

				} else {
					_failsafe_pwm[i] = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_failsafe_set = 0;

			for (unsigned i = 0; i < _max_actuators; i++) {
				if (_failsafe_pwm[i] > 0) {
					_num_failsafe_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _failsafe_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			break;
		}

	case PWM_SERVO_SET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_disarmed_pwm[i] = PWM_HIGHEST_MAX;

				} else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_disarmed_pwm[i] = PWM_LOWEST_MIN;

				} else {
					_disarmed_pwm[i] = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_disarmed_set = 0;

			for (unsigned i = 0; i < _max_actuators; i++) {
				if (_disarmed_pwm[i] > 0) {
					_num_disarmed_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _disarmed_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			break;
		}

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MIN) {
					_min_pwm[i] = PWM_HIGHEST_MIN;

				} else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_min_pwm[i] = PWM_LOWEST_MIN;

				} else {
					_min_pwm[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _min_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] < PWM_LOWEST_MAX) {
					_max_pwm[i] = PWM_LOWEST_MAX;

				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_max_pwm[i] = PWM_HIGHEST_MAX;

				} else {
					_max_pwm[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _max_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET(11):
	case PWM_SERVO_SET(10):
		if (_mode < MODE_12PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_SET(9):
	case PWM_SERVO_SET(8):
		if (_mode < MODE_10PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_SET(7):
	case PWM_SERVO_SET(6):
		if (_mode < MODE_8PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_SET(5):
	case PWM_SERVO_SET(4):
		if (_mode < MODE_6PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(3):
		if (_mode < MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(2):
		if (_mode < MODE_3PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(1):
	case PWM_SERVO_SET(0):
		if (arg <= 2100) {
			pwm_output_set(cmd - PWM_SERVO_SET(0), arg);

		} else {
			ret = -EINVAL;
		}

		break;

	case PWM_SERVO_GET(11):
	case PWM_SERVO_GET(10):
		if (_mode < MODE_12PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_GET(9):
	case PWM_SERVO_GET(8):
		if (_mode < MODE_10PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_GET(7):
	case PWM_SERVO_GET(6):
		if (_mode < MODE_8PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_GET(5):
	case PWM_SERVO_GET(4):
		if (_mode < MODE_6PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(3):
		if (_mode < MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(2):
		if (_mode < MODE_3PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(1):
	case PWM_SERVO_GET(0):
		*(servo_position_t *)arg = up_pwm_servo_get(cmd - PWM_SERVO_GET(0));
		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
	case PWM_SERVO_GET_RATEGROUP(4):
	case PWM_SERVO_GET_RATEGROUP(5):
	case PWM_SERVO_GET_RATEGROUP(6):
	case PWM_SERVO_GET_RATEGROUP(7):
	case PWM_SERVO_GET_RATEGROUP(8):
	case PWM_SERVO_GET_RATEGROUP(9):
	case PWM_SERVO_GET_RATEGROUP(10):
	case PWM_SERVO_GET_RATEGROUP(11):
		*(uint32_t *)arg = up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:
	case MIXERIOCGETOUTPUTCOUNT:
		switch (_mode) {

		case MODE_12PWM:
			*(unsigned *)arg = 12;
			break;

		case MODE_10PWM:
			*(unsigned *)arg = 10;
			break;

		case MODE_8PWM:
			*(unsigned *)arg = 8;
			break;

		case MODE_6PWM:
			*(unsigned *)arg = 6;
			break;

		case MODE_4PWM:
			*(unsigned *)arg = 4;
			break;

		case MODE_3PWM:
		case MODE_3PWM1CAP:
			*(unsigned *)arg = 3;
			break;

		case MODE_2PWM:
		case MODE_2PWM2CAP:
			*(unsigned *)arg = 2;
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case PWM_SERVO_SET_COUNT: {
			/* change the number of outputs that are enabled for
			 * PWM. This is used to change the split between GPIO
			 * and PWM under control of the flight config
			 * parameters. Note that this does not allow for
			 * changing a set of pins to be used for serial on
			 * FMUv1
			 */
			switch (arg) {
			case 0:
				set_mode(MODE_NONE);
				break;

			case 2:
				set_mode(MODE_2PWM);
				break;

			case 3:
				set_mode(MODE_3PWM);
				break;

			case 4:
				set_mode(MODE_4PWM);
				break;

			case 6:
				set_mode(MODE_6PWM);
				break;

			case 8:
				set_mode(MODE_8PWM);
				break;

			case 10:
				set_mode(MODE_10PWM);
				break;

			case 12:
				set_mode(MODE_12PWM);
				break;

			default:
				ret = -EINVAL;
				break;
			}

			break;
		}

	case PWM_SERVO_SET_MODE: {
			switch (arg) {
			case PWM_SERVO_MODE_NONE:
				ret = set_mode(MODE_NONE);
				break;

			case PWM_SERVO_MODE_2PWM:
				ret = set_mode(MODE_2PWM);
				break;

			case PWM_SERVO_MODE_2PWM2CAP:
				ret = set_mode(MODE_2PWM2CAP);
				break;

			case PWM_SERVO_MODE_3PWM:
				ret = set_mode(MODE_3PWM);
				break;

			case PWM_SERVO_MODE_3PWM1CAP:
				ret = set_mode(MODE_3PWM1CAP);
				break;

			case PWM_SERVO_MODE_4PWM:
				ret = set_mode(MODE_4PWM);
				break;

			case PWM_SERVO_MODE_6PWM:
				ret = set_mode(MODE_6PWM);
				break;

			case PWM_SERVO_MODE_8PWM:
				ret = set_mode(MODE_8PWM);
				break;

			case PWM_SERVO_MODE_10PWM:
				ret = set_mode(MODE_10PWM);
				break;

			case PWM_SERVO_MODE_12PWM:
				ret = set_mode(MODE_12PWM);
				break;

			case PWM_SERVO_MODE_4CAP:
				ret = set_mode(MODE_4CAP);
				break;

			case PWM_SERVO_MODE_5CAP:
				ret = set_mode(MODE_5CAP);
				break;

			case PWM_SERVO_MODE_6CAP:
				ret = set_mode(MODE_6CAP);
				break;

			default:
				ret = -EINVAL;
			}

			break;
		}

	case PWM_SERVO_SET_ONESHOT:
		_oneshot_mode = arg ? true : false;
		ret = OK;
		break;

	case PWM_SERVO_IGNORE_SAFETY:
		_ignore_safety_mask = arg;
		ret = OK;
		if (_ignore_safety_mask & 0xFF) {
			update_pwm_out_state();
		}
		break;

#if defined(RC_SERIAL_PORT) && defined(GPIO_SPEKTRUM_PWR_EN)

	case DSM_BIND_START:
		/* only allow DSM2, DSM-X and DSM-X with more than 7 channels */
		warnx("fmu pwm_ioctl: DSM_BIND_START, arg: %lu", arg);

		if (arg == DSM2_BIND_PULSES ||
		    arg == DSMX_BIND_PULSES ||
		    arg == DSMX8_BIND_PULSES) {

			dsm_bind(DSM_CMD_BIND_POWER_DOWN, 0);
			dsm_bind(DSM_CMD_BIND_SET_RX_OUT, 0);
			usleep(500000);

			dsm_bind(DSM_CMD_BIND_POWER_UP, 0);
			usleep(72000);

			dsm_bind(DSM_CMD_BIND_SEND_PULSES, arg);
			usleep(50000);

			dsm_bind(DSM_CMD_BIND_REINIT_UART, 0);

			ret = OK;

		} else {
			ret = -EINVAL;
		}

		break;
#endif

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
			_groups_required = 0;
		}

		break;

	case MIXERIOCADDSIMPLE: {
			mixer_simple_s *mixinfo = (mixer_simple_s *)arg;

			SimpleMixer *mixer = new SimpleMixer(control_callback,
							     (uintptr_t)_controls, mixinfo);

			if (mixer->check()) {
				delete mixer;
				_groups_required = 0;
				ret = -EINVAL;

			} else {
				if (_mixers == nullptr)
					_mixers = new MixerGroup(control_callback,
								 (uintptr_t)_controls);

				_mixers->add_mixer(mixer);
				_mixers->groups_required(_groups_required);
			}

			break;
		}

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == nullptr) {
				_mixers = new MixerGroup(control_callback, (uintptr_t)_controls);
			}

			if (_mixers == nullptr) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					DEVICE_DEBUG("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					_groups_required = 0;
					ret = -EINVAL;

				} else {

					_mixers->groups_required(_groups_required);
				}
			}

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

/*
  this implements PWM output via a write() method, for compatibility
  with dpio
 */
ssize_t
FMU::write(file *filp, const char *buffer, size_t len)
{
	unsigned count = len / 2;
	uint16_t values[12];

	if (count > 12) {
		// we have at most 12 outputs
		count = 12;
	}

	// allow for misaligned values
	memcpy(values, buffer, count * 2);

	if (_oneshot_mode) {
		hrt_abstime now = hrt_absolute_time();

		/*
		  when doing oneshot we need to guarantee that one
		  pulse won't interrupt the previous pulse, while
		  still getting the new pulse out as quickly as
		  possible. To do this we need to wait till the widest
		  pulse of the last output has finished. We add 50
		  microseconds to ensure the ESC has registered the
		  end of the pulse
		 */
		if (now < _oneshot_delay_till) {
			up_udelay(_oneshot_delay_till - now);
		}
	}

	uint16_t widest_pulse = 0;

	for (uint8_t i = 0; i < count; i++) {
            if (values[i] != PWM_IGNORE_THIS_CHANNEL && (((1U<<i) & _pwm_mask))) {
			if (_safety_off || ((1U<<i) & _ignore_safety_mask)) {
				pwm_output_set(i, values[i]);
			} else {
				pwm_output_set(i, 0);
			}

			if (values[i] > widest_pulse) {
				widest_pulse = values[i];
			}
		}
	}

	if (widest_pulse > 2300) {
			// don't allow extreme pulses to cause issues with oneshot delays
			widest_pulse = 2300;
	}
    
	for (uint8_t i = count; i < _max_actuators; i++) {
            if ((1U<<i) & _pwm_mask) {
                pwm_output_set(i, 0);
            }
        }

	if (_oneshot_mode) {
		//up_pwm_servo_trigger(_pwm_alt_rate_channels);
		_oneshot_delay_till = hrt_absolute_time() + widest_pulse + 50;
	}

	return count * 2;
}

void
FMU::sensor_reset(int ms)
{
#if  defined(CONFIG_ARCH_BOARD_UAVRS_V1)

	if (ms < 1) {
		ms = 1;
	}

    /* disable SPI bus */
	stm32_configgpio(GPIO_SPI_CS_BARO_OFF);
	stm32_configgpio(GPIO_SPI_CS_MPU_OFF);
	stm32_configgpio(GPIO_SPI_CS_ADIS_OFF);	

	stm32_gpiowrite(GPIO_SPI_CS_BARO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI_CS_MPU_OFF, 0);
	stm32_gpiowrite(GPIO_SPI_CS_ADIS_OFF, 0);

	stm32_configgpio(GPIO_SPI4_SCK_OFF);
	stm32_configgpio(GPIO_SPI4_MISO_OFF);
	stm32_configgpio(GPIO_SPI4_MOSI_OFF);

	stm32_gpiowrite(GPIO_SPI4_SCK_OFF, 0);
	stm32_gpiowrite(GPIO_SPI4_MISO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI4_MOSI_OFF, 0);

	stm32_configgpio(GPIO_SPI3_SCK_OFF);
	stm32_configgpio(GPIO_SPI3_MISO_OFF);
	stm32_configgpio(GPIO_SPI3_MOSI_OFF);

	stm32_gpiowrite(GPIO_SPI3_SCK_OFF, 0);
	stm32_gpiowrite(GPIO_SPI3_MISO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI3_MOSI_OFF, 0);

	/* wait for the sensor rail to reach GND */
	usleep(ms * 1000);
	warnx("reset done, %d ms", ms);

	/* re-enable power */

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */
#ifdef CONFIG_STM32_SPI4
	stm32_configgpio(GPIO_SPI_CS_ADIS);	

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	stm32_gpiowrite(GPIO_SPI_CS_ADIS, 1);

	stm32_configgpio(GPIO_SPI4_SCK);
	stm32_configgpio(GPIO_SPI4_MISO);
	stm32_configgpio(GPIO_SPI4_MOSI);
#endif

#ifdef CONFIG_STM32_SPI3
	stm32_configgpio(GPIO_SPI_CS_BARO);
	stm32_configgpio(GPIO_SPI_CS_MPU);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	stm32_gpiowrite(GPIO_SPI_CS_BARO, 1);
	stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);

	stm32_configgpio(GPIO_SPI3_SCK);
	stm32_configgpio(GPIO_SPI3_MISO);
	stm32_configgpio(GPIO_SPI3_MOSI);
#endif
#endif
}

void
FMU::peripheral_reset(int ms)
{
#if defined(CONFIG_ARCH_BOARD_UAVRS_V1)

	if (ms < 1) {
		ms = 10;
	}

#endif
}

void
FMU::gpio_reset(void)
{
}

void
FMU::gpio_set_function(uint32_t gpios, int function)
{
}

void
FMU::gpio_write(uint32_t gpios, int function)
{
}

uint32_t
FMU::gpio_read(void)
{
return 0;
}

int
FMU::capture_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = -EINVAL;

	lock();

	input_capture_config_t *pconfig = 0;

	input_capture_stats_t *stats = (input_capture_stats_t *)arg;

	if (_mode == MODE_3PWM1CAP || _mode == MODE_2PWM2CAP) {

		pconfig = (input_capture_config_t *)arg;
	}

	switch (cmd) {

	case INPUT_CAP_SET:
		if (pconfig) {
			ret =  up_input_capture_set(pconfig->channel, pconfig->edge, pconfig->filter,
						    pconfig->callback, pconfig->context);
		}

		break;

	case INPUT_CAP_SET_CALLBACK:
		if (pconfig) {
			ret =  up_input_capture_set_callback(pconfig->channel, pconfig->callback, pconfig->context);
		}

		break;

	case INPUT_CAP_GET_CALLBACK:
		if (pconfig) {
			ret =  up_input_capture_get_callback(pconfig->channel, &pconfig->callback, &pconfig->context);
		}

		break;

	case INPUT_CAP_GET_STATS:
		if (arg) {
			ret =  up_input_capture_get_stats(stats->chan_in_edges_out, stats, false);
		}

		break;

	case INPUT_CAP_GET_CLR_STATS:
		if (arg) {
			ret =  up_input_capture_get_stats(stats->chan_in_edges_out, stats, true);
		}

		break;

	case INPUT_CAP_SET_EDGE:
		if (pconfig) {
			ret =  up_input_capture_set_trigger(pconfig->channel, pconfig->edge);
		}

		break;

	case INPUT_CAP_GET_EDGE:
		if (pconfig) {
			ret =  up_input_capture_get_trigger(pconfig->channel, &pconfig->edge);
		}

		break;

	case INPUT_CAP_SET_FILTER:
		if (pconfig) {
			ret =  up_input_capture_set_filter(pconfig->channel, pconfig->filter);
		}

		break;

	case INPUT_CAP_GET_FILTER:
		if (pconfig) {
			ret =  up_input_capture_get_filter(pconfig->channel, &pconfig->filter);
		}

		break;

	case INPUT_CAP_GET_COUNT:
		ret = OK;

		switch (_mode) {
		case MODE_3PWM1CAP:
			*(unsigned *)arg = 1;
			break;

		case MODE_2PWM2CAP:
			*(unsigned *)arg = 2;
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case INPUT_CAP_SET_COUNT:
		ret = OK;

		switch (_mode) {
		case MODE_3PWM1CAP:
			set_mode(MODE_3PWM1CAP);
			break;

		case MODE_2PWM2CAP:
			set_mode(MODE_2PWM2CAP);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();
	return ret;
}

int
FMU::gpio_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = OK;

	lock();

	switch (cmd) {

	case GPIO_RESET:
		gpio_reset();
		break;

	case GPIO_SENSOR_RAIL_RESET:
		sensor_reset(arg);
		break;

	case GPIO_PERIPHERAL_RAIL_RESET:
		peripheral_reset(arg);
		break;

	case GPIO_SET_OUTPUT:
	case GPIO_SET_OUTPUT_LOW:
	case GPIO_SET_OUTPUT_HIGH:
	case GPIO_SET_INPUT:
	case GPIO_SET_ALT_1:
		gpio_set_function(arg, cmd);
		break;

	case GPIO_SET_ALT_2:
	case GPIO_SET_ALT_3:
	case GPIO_SET_ALT_4:
		ret = -EINVAL;
		break;

	case GPIO_SET:
	case GPIO_CLEAR:
		gpio_write(arg, cmd);
		break;

	case GPIO_GET:
		*(uint32_t *)arg = gpio_read();
		break;

	default:
		ret = -ENOTTY;
	}

	unlock();

	return ret;
}

void
FMU::dsm_bind_ioctl(int dsmMode)
{
	if (!_armed.armed) {
//      mavlink_log_info(&_mavlink_log_pub, "[FMU] binding DSM%s RX", (dsmMode == 0) ? "2" : ((dsmMode == 1) ? "-X" : "-X8"));
		warnx("[FMU] binding DSM%s RX", (dsmMode == 0) ? "2" : ((dsmMode == 1) ? "-X" : "-X8"));
		int ret = ioctl(nullptr, DSM_BIND_START,
				(dsmMode == 0) ? DSM2_BIND_PULSES : ((dsmMode == 1) ? DSMX_BIND_PULSES : DSMX8_BIND_PULSES));

		if (ret) {
//            mavlink_log_critical(&_mavlink_log_pub, "binding failed.");
			warnx("binding failed.");
		}

	} else {
//        mavlink_log_info(&_mavlink_log_pub, "[FMU] system armed, bind request rejected");
		warnx("[FMU] system armed, bind request rejected");
	}
}

void
FMU::status(void)
{
	static const char *modes[] = {"NONE", "2PWM", "2PWM2CAP", "3PWM", "3PWM1CAP",
				      "4PWM", "6PWM", "8PWM", "4CAP", "5CAP", "6CAP"
				     };

	printf("status %s%s%s%s MODE_%s pwm_mask:0x%02x alt_mask:0x%02x rate:%u alt_rate:%u\n",
	       _safety_off ? " SAFETY_OFF" : " SAFETY_SAFE",
	       _servos_armed ? " SERVOS_ARMED" : " SERVOS_DISARMED",
	       _pwm_on ? " PWM_ON" : " PWM_OFF",
	       _oneshot_mode ? " PWM_ONESHOT" : " PWM_NORMAL",
	       _mode <= MODE_6CAP ? modes[_mode] : "INVALID",
	       (unsigned)_pwm_mask,
	       (unsigned)_pwm_alt_rate_channels,
	       (unsigned)_pwm_default_rate,
	       (unsigned)_pwm_alt_rate);
	printf("ignore_safety: 0x%02x pwm_init: %s\n",
	       (unsigned)_ignore_safety_mask,
	       _pwm_initialized?"YES":"NO");

	printf("failsafe PWM ");
	for (uint8_t i = 0; i < _max_actuators; i++) {
		printf("%u ", _failsafe_pwm[i]);
	}
	printf("\n");
	
	printf("disarmed PWM ");
	for (uint8_t i = 0; i < _max_actuators; i++) {
		printf("%u ", _disarmed_pwm[i]);
	}
	printf("\n");

	printf("output PWM ");
	for (uint8_t i = 0; i < _max_actuators; i++) {
		printf("%u ", _output_pwm[i]);
	}
	printf("\n");
}

namespace
{

enum PortMode {
	PORT_MODE_UNSET = 0,
	PORT_FULL_GPIO,
	PORT_FULL_SERIAL,
	PORT_FULL_PWM,
	PORT_GPIO_AND_SERIAL,
	PORT_PWM_AND_SERIAL,
	PORT_PWM_AND_GPIO,
	PORT_PWM4,
	PORT_PWM3,
	PORT_PWM2,
	PORT_PWM3CAP1,
	PORT_PWM2CAP2,
	PORT_CAPTURE,
};

PortMode g_port_mode;

int
fmu_new_mode(PortMode new_mode)
{
	uint32_t gpio_bits;
	FMU::Mode servo_mode;
	bool mode_with_input = false;

	gpio_bits = 0;
	servo_mode = FMU::MODE_NONE;

	switch (new_mode) {
	case PORT_FULL_GPIO:
	case PORT_MODE_UNSET:
		break;

	case PORT_FULL_PWM:
		servo_mode = FMU::MODE_12PWM;
		break;

	case PORT_PWM4:
		/* select 4-pin PWM mode */
		servo_mode = FMU::MODE_4PWM;
		break;

	case PORT_PWM3:
		/* select 4-pin PWM mode */
		servo_mode = FMU::MODE_3PWM;
		break;

	case PORT_PWM3CAP1:
		/* select 3-pin PWM mode 1 capture */
		servo_mode = FMU::MODE_3PWM1CAP;
		mode_with_input = true;
		break;

	case PORT_PWM2:
		/* select 2-pin PWM mode */
		servo_mode = FMU::MODE_2PWM;
		break;

	case PORT_PWM2CAP2:
		/* select 2-pin PWM mode 2 capture */
		servo_mode = FMU::MODE_2PWM2CAP;
		mode_with_input = true;
		break;

	default:
		return -1;
	}

	if (servo_mode != g_fmu->get_mode()) {

		/* reset to all-inputs */
		if (mode_with_input) {
			g_fmu->ioctl(0, GPIO_RESET, 0);

			/* adjust GPIO config for serial mode(s) */
			if (gpio_bits != 0) {
				g_fmu->ioctl(0, GPIO_SET_ALT_1, gpio_bits);
			}
		}

		/* (re)set the PWM output mode */
		g_fmu->set_mode(servo_mode);
	}

	return OK;
}

int fmu_new_i2c_speed(unsigned bus, unsigned clock_hz)
{
	return FMU::set_i2c_bus_clock(bus, clock_hz);
}

int
fmu_start(void)
{
	int ret = OK;

	if (g_fmu == nullptr) {

		g_fmu = new FMU;

		if (g_fmu == nullptr) {
			ret = -ENOMEM;

		} else {
			ret = g_fmu->init();

			if (ret != OK) {
				delete g_fmu;
				g_fmu = nullptr;
			}
		}
	}

	return ret;
}

int
fmu_stop(void)
{
	int ret = OK;

	if (g_fmu != nullptr) {

		delete g_fmu;
		g_fmu = nullptr;
	}

	return ret;
}

void
sensor_reset(int ms)
{
	int	 fd;

	fd = open(FMU_DEVICE_PATH, O_RDWR);

	if (fd < 0) {
		errx(1, "open fail");
	}

	if (ioctl(fd, GPIO_SENSOR_RAIL_RESET, ms) < 0) {
		warnx("sensor rail reset failed");
	}

	close(fd);
}

void
peripheral_reset(int ms)
{
	int	 fd;

	fd = open(FMU_DEVICE_PATH, O_RDWR);

	if (fd < 0) {
		errx(1, "open fail");
	}

	if (ioctl(fd, GPIO_PERIPHERAL_RAIL_RESET, ms) < 0) {
		warnx("peripheral rail reset failed");
	}

	close(fd);
}

void
test(void)
{
	int	 fd;
	unsigned servo_count = 0;
	unsigned capture_count = 0;
	unsigned pwm_value = 1000;
	int	 direction = 1;
	int	 ret;
	uint32_t rate_limit = 0;
	struct input_capture_t {
		bool valid;
		input_capture_config_t  chan;
	} capture_conf[INPUT_CAPTURE_MAX_CHANNELS];

	fd = open(FMU_DEVICE_PATH, O_RDWR);

	if (fd < 0) {
		errx(1, "open fail");
	}

	if (ioctl(fd, PWM_SERVO_ARM, 0) < 0) { err(1, "servo arm failed"); }

	if (ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count) != 0) {
		err(1, "Unable to get servo count\n");
	}

	if (ioctl(fd, INPUT_CAP_GET_COUNT, (unsigned long)&capture_count) != 0) {
		fprintf(stdout, "Not in a capture mode\n");
	}

	warnx("Testing %u servos and %u input captures", (unsigned)servo_count, capture_count);
	memset(capture_conf, 0, sizeof(capture_conf));

	if (capture_count != 0) {
		for (unsigned i = 0; i < capture_count; i++) {
			// Map to channel number
			capture_conf[i].chan.channel = i + servo_count;

			/* Save handler */
			if (ioctl(fd, INPUT_CAP_GET_CALLBACK, (unsigned long)&capture_conf[i].chan.channel) != 0) {
				err(1, "Unable to get capture callback for chan %u\n", capture_conf[i].chan.channel);

			} else {
				input_capture_config_t conf = capture_conf[i].chan;
				conf.callback = &FMU::capture_trampoline;
				conf.context = g_fmu;

				if (ioctl(fd, INPUT_CAP_SET_CALLBACK, (unsigned long)&conf) == 0) {
					capture_conf[i].valid = true;

				} else {
					err(1, "Unable to set capture callback for chan %u\n", capture_conf[i].chan.channel);
				}
			}

		}
	}

	struct pollfd fds;

	fds.fd = 0; /* stdin */

	fds.events = POLLIN;

	warnx("Press CTRL-C or 'c' to abort.");

	for (;;) {
		/* sweep all servos between 1000..2000 */
		servo_position_t servos[servo_count];

		for (unsigned i = 0; i < servo_count; i++) {
			servos[i] = pwm_value;
		}

		if (direction == 1) {
			// use ioctl interface for one direction
			for (unsigned i = 0; i < servo_count;	i++) {
				if (ioctl(fd, PWM_SERVO_SET(i), servos[i]) < 0) {
					err(1, "servo %u set failed", i);
				}
			}

		} else {
			// and use write interface for the other direction
			ret = write(fd, servos, sizeof(servos));

			if (ret != (int)sizeof(servos)) {
				err(1, "error writing PWM servo data, wrote %u got %d", sizeof(servos), ret);
			}
		}

		if (direction > 0) {
			if (pwm_value < 2000) {
				pwm_value++;

			} else {
				direction = -1;
			}

		} else {
			if (pwm_value > 1000) {
				pwm_value--;

			} else {
				direction = 1;
			}
		}

		/* readback servo values */
		for (unsigned i = 0; i < servo_count; i++) {
			servo_position_t value;

			if (ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&value)) {
				err(1, "error reading PWM servo %d", i);
			}

			if (value != servos[i]) {
				errx(1, "servo %d readback error, got %u expected %u", i, value, servos[i]);
			}
		}

		if (capture_count != 0 && (++rate_limit % 500 == 0)) {
			for (unsigned i = 0; i < capture_count; i++) {
				if (capture_conf[i].valid) {
					input_capture_stats_t stats;
					stats.chan_in_edges_out = capture_conf[i].chan.channel;

					if (ioctl(fd, INPUT_CAP_GET_STATS, (unsigned long)&stats) != 0) {
						err(1, "Unable to get stats for chan %u\n", capture_conf[i].chan.channel);

					} else {
						fprintf(stdout, "FMU: Status chan:%u edges: %d last time:%lld last state:%d overflows:%d lantency:%u\n",
							capture_conf[i].chan.channel,
							stats.chan_in_edges_out,
							stats.last_time,
							stats.last_edge,
							stats.overflows,
							stats.latnecy);
					}
				}
			}

		}

		/* Check if user wants to quit */
		char c;
		ret = poll(&fds, 1, 0);

		if (ret > 0) {

			read(0, &c, 1);

			if (c == 0x03 || c == 0x63 || c == 'q') {
				warnx("User abort\n");
				break;
			}
		}
	}

	if (capture_count != 0) {
		for (unsigned i = 0; i < capture_count; i++) {
			// Map to channel number
			if (capture_conf[i].valid) {
				/* Save handler */
				if (ioctl(fd, INPUT_CAP_SET_CALLBACK, (unsigned long)&capture_conf[i].chan) != 0) {
					err(1, "Unable to set capture callback for chan %u\n", capture_conf[i].chan.channel);
				}
			}
		}
	}

	close(fd);


	exit(0);
}

void
fake(int argc, char *argv[])
{
	if (argc < 5) {
		errx(1, "fmu fake <roll> <pitch> <yaw> <thrust> (values -100 .. 100)");
	}

	actuator_controls_s ac;

	ac.control[0] = strtol(argv[1], 0, 0) / 100.0f;

	ac.control[1] = strtol(argv[2], 0, 0) / 100.0f;

	ac.control[2] = strtol(argv[3], 0, 0) / 100.0f;

	ac.control[3] = strtol(argv[4], 0, 0) / 100.0f;

	orb_advert_t handle = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &ac);

	if (handle == nullptr) {
		errx(1, "advertise failed");
	}

	actuator_armed_s aa;

	aa.armed = true;
	aa.lockdown = false;

	handle = orb_advertise(ORB_ID(actuator_armed), &aa);

	if (handle == nullptr) {
		errx(1, "advertise failed 2");
	}

	exit(0);
}

} // namespace

extern "C" __EXPORT int fmu_main(int argc, char *argv[]);

int
fmu_main(int argc, char *argv[])
{
	PortMode new_mode = PORT_MODE_UNSET;
	const char *verb = argv[1];

	/* does not operate on a FMU instance */
	if (!strcmp(verb, "i2c")) {
		if (argc > 3) {
			int bus = strtol(argv[2], 0, 0);
			int clock_hz = strtol(argv[3], 0, 0);
			int ret = fmu_new_i2c_speed(bus, clock_hz);

			if (ret) {
				errx(ret, "setting I2C clock failed");
			}

			exit(0);

		} else {
			warnx("i2c cmd args: <bus id> <clock Hz>");
		}
	}

	if (!strcmp(verb, "stop")) {
		fmu_stop();
		errx(0, "FMU driver stopped");
	}

	if (!strcmp(verb, "id")) {
		uint8_t id[12];
		(void)get_board_serial(id);

		errx(0, "Board serial:\n %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
		     (unsigned)id[0], (unsigned)id[1], (unsigned)id[2], (unsigned)id[3], (unsigned)id[4], (unsigned)id[5],
		     (unsigned)id[6], (unsigned)id[7], (unsigned)id[8], (unsigned)id[9], (unsigned)id[10], (unsigned)id[11]);
	}

	if (fmu_start() != OK) {
		errx(1, "failed to start the FMU driver");
	}

	/*
	 * Mode switches.
	 */
	if (!strcmp(verb, "mode_gpio")) {
		new_mode = PORT_FULL_GPIO;

	} else if (!strcmp(verb, "mode_rcin")) {
		exit(0);

	} else if (!strcmp(verb, "mode_pwm")) {
		new_mode = PORT_FULL_PWM;

	} else if (!strcmp(verb, "mode_pwm4")) {
		new_mode = PORT_PWM4;

	} else if (!strcmp(verb, "mode_pwm2")) {
		new_mode = PORT_PWM2;

	} else if (!strcmp(verb, "mode_pwm3")) {
		new_mode = PORT_PWM3;

	} else if (!strcmp(verb, "mode_pwm3cap1")) {
		new_mode = PORT_PWM3CAP1;

	} else if (!strcmp(verb, "mode_pwm2cap2")) {
		new_mode = PORT_PWM2CAP2;
	}

	/* was a new mode set? */
	if (new_mode != PORT_MODE_UNSET) {

		/* yes but it's the same mode */
		if (new_mode == g_port_mode) {
			return OK;
		}

		/* switch modes */
		int ret = fmu_new_mode(new_mode);
		exit(ret == OK ? 0 : 1);
	}

	if (!strcmp(verb, "test")) {
		test();
	}

	if (!strcmp(verb, "info")) {
#ifdef RC_SERIAL_PORT
		warnx("frame drops: %u", sbus_dropped_frames());
#endif
		return 0;
	}

	if (!strcmp(verb, "fake")) {
		fake(argc - 1, argv + 1);
	}

	if (!strcmp(verb, "sensor_reset")) {
		if (argc > 2) {
			int reset_time = strtol(argv[2], 0, 0);
			sensor_reset(reset_time);

		} else {
			sensor_reset(0);
			warnx("resettet default time");
		}

		exit(0);
	}

	if (!strcmp(verb, "peripheral_reset")) {
		if (argc > 2) {
			int reset_time = strtol(argv[2], 0, 0);
			peripheral_reset(reset_time);

		} else {
			peripheral_reset(0);
			warnx("resettet default time");
		}

		exit(0);
	}

	if (!strcmp(verb, "status")) {
		if (!g_fmu) {
			errx(1, "not started");
		}

		g_fmu->status();
		exit(0);
	}

	fprintf(stderr, "FMU: unrecognised command %s, try:\n", verb);
	fprintf(stderr, "  mode_gpio, mode_pwm, mode_pwm4, test, sensor_reset [milliseconds], i2c <bus> <hz>\n");
	exit(1);
}
