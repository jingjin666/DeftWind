#pragma once

#include "AP_HAL_UAVRS.h"
#include <systemlib/perf_counter.h>

#include <AP_HAL/utility/RingBuffer.h>


class UAVRS::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(const char *devpath, const char *perf_name);
    /* UAVRS implementations of UARTDriver virtual methods */
    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    /* UAVRS implementations of Stream virtual methods */
    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;
    uint32_t read_bytes(uint8_t *data, uint32_t len) override;

    /* UAVRS implementations of Print virtual methods */
    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    void set_device_path(const char *path) {
	    _devpath = path;
    }

    void _timer_tick(void);

    int _get_fd(void) {
	    return _fd;
    }

    void set_flow_control(enum flow_control flow_control);
    enum flow_control get_flow_control(void) { return _flow_control; }

private:
    const char *_devpath;
    int _fd;
    uint32_t _baudrate;
    volatile bool _initialised;
    volatile bool _in_timer;

    bool _nonblocking_writes;

    // we use in-task ring buffers to reduce the system call cost
    // of ::read() and ::write() in the main loop
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    perf_counter_t  _perf_uart;

    int _write_fd(const uint8_t *buf, uint16_t n);
    int _read_fd(uint8_t *buf, uint16_t n);
    uint64_t _first_write_time;
    uint64_t _last_write_time;

    void try_initialise(void);
    uint32_t _last_initialise_attempt_ms;

    uint32_t _os_start_auto_space;
    uint32_t _total_read;
    uint32_t _total_written;
    enum flow_control _flow_control;

    pid_t _uart_owner_pid;
};
