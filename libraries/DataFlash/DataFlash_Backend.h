#pragma once

#include "DataFlash.h"

class DFMessageWriter_DFLogStart;

class DataFlash_Backend
{

public:
    FUNCTOR_TYPEDEF(print_mode_fn, void, AP_HAL::BetterStream*, uint8_t);
    FUNCTOR_TYPEDEF(vehicle_startup_message_Log_Writer, void);

    DataFlash_Backend(DataFlash_Class &front,
                      class DFMessageWriter_DFLogStart *writer);

    vehicle_startup_message_Log_Writer vehicle_message_writer();

    void internal_error();

    virtual bool CardInserted(void) const = 0;

    // erase handling
    virtual void EraseAll() = 0;
	virtual void EraseAllRawData() = 0;
	virtual void EraseAllPosData() = 0;

    virtual bool NeedPrep() = 0;
    virtual void Prep() = 0;

    /* Write a block of data at current offset */
    bool WriteBlock(const void *pBuffer, uint16_t size) {
        return WritePrioritisedBlock(pBuffer, size, false);
    }

    bool WriteCriticalBlock(const void *pBuffer, uint16_t size) {
        return WritePrioritisedBlock(pBuffer, size, true);
    }

    bool WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical);
	
    /* Write a block of data at current offset */
    bool WriteRawData(const void *pBuffer, uint16_t size) {
        return WriteRawData(pBuffer, size, false);
	}

    bool WriteRawData(const void *pBuffer, uint16_t size, bool is_critical);

    /* Write a block of data at current offset */
    bool WritePosData(const void *pBuffer, uint16_t size) {
        return WritePosData(pBuffer, size, false);
	}

    bool WritePosData(const void *pBuffer, uint16_t size, bool is_critical);

    // high level interface
    virtual uint16_t find_last_log() = 0;
	virtual uint16_t find_last_raw_data() = 0;
	virtual uint16_t find_last_pos_data() = 0;
    virtual void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) = 0;
    virtual void get_raw_data_boundaries(uint16_t raw_num, uint16_t & start_page, uint16_t & end_page) = 0;
    virtual void get_pos_data_boundaries(uint16_t pos_num, uint16_t & start_page, uint16_t & end_page) = 0;
    virtual void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) = 0;
    virtual void get_raw_data_info(uint16_t raw_num, uint32_t &size, uint32_t &time_utc) = 0;
    virtual void get_pos_data_info(uint16_t pos_num, uint32_t &size, uint32_t &time_utc) = 0;
    virtual int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) = 0;
    virtual int16_t get_raw_data(uint16_t raw_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) = 0;
    virtual int16_t get_pos_data(uint16_t pos_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) = 0;
    virtual uint16_t get_num_logs() = 0;
    virtual uint16_t get_num_raw_data() = 0;
    virtual uint16_t get_num_pos_data() = 0;
    virtual void LogReadProcess(const uint16_t list_entry,
                                uint16_t start_page, uint16_t end_page,
                                print_mode_fn printMode,
                                AP_HAL::BetterStream *port) = 0;
    virtual void DumpPageInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ShowDeviceInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ListAvailableLogs(AP_HAL::BetterStream *port) = 0;

    virtual bool logging_started(void) const = 0;
	
    virtual bool raw_data_started(void) const = 0;
	
    virtual bool pos_data_started(void) const = 0;

    virtual void Init() { }

    void set_mission(const AP_Mission *mission);

    virtual uint32_t bufferspace_available() = 0;

    virtual void PrepForArming() { }

    virtual uint16_t start_new_log(void) = 0;

	virtual uint16_t start_new_raw_data(void) = 0;

	virtual uint16_t start_new_pos_data(void) = 0;

    /* stop logging - close output files etc etc.
     *
     * note that this doesn't stop logging from starting up again
     * immediately - e.g. DataFlash_MAVLink might get another start
     * packet from a client.
     */
    virtual void stop_logging(void) = 0;

	virtual void stop_raw_data(void) = 0;
	
	virtual void stop_pos_data(void) = 0;
	
    void Log_Fill_Format(const struct LogStructure *structure, struct log_Format &pkt);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // currently only DataFlash_File support this:
    virtual void flush(void) { }
#endif

     // for Dataflash_MAVlink
    virtual void remote_log_block_status_msg(mavlink_channel_t chan,
                                             mavlink_message_t* msg) { }
    // end for Dataflash_MAVlink

   virtual void periodic_tasks();

    uint8_t num_types() const;
    const struct LogStructure *structure(uint8_t structure) const;

    void Log_Write_EntireMission(const AP_Mission &mission);
    bool Log_Write_Format(const struct LogStructure *structure);
    bool Log_Write_MavCmd(uint16_t cmd_total, const mavlink_mission_item_t& mav_cmd);
    bool Log_Write_Message(const char *message);
    bool Log_Write_Mission_Cmd(const AP_Mission &mission,
                               const AP_Mission::Mission_Command &cmd);
    bool Log_Write_Mode(uint8_t mode, uint8_t reason = 0);
    bool Log_Write_Parameter(const char *name, float value);
    bool Log_Write_Parameter(const AP_Param *ap,
                             const AP_Param::ParamToken &token,
                             enum ap_var_type type);

    uint32_t num_dropped(void) const {
        return _dropped;
    }

    /*
     * Log_Write support
     */
    // write a FMT message out (if it hasn't been done already).
    // Returns true if the FMT message has ever been written.
    bool Log_Write_Emit_FMT(uint8_t msg_type);

    // write a log message out to the log of msg_type type, with
    // values contained in arg_list:
    bool Log_Write(uint8_t msg_type, va_list arg_list, bool is_critical=false);

    // these methods are used when reporting system status over mavlink
    virtual bool logging_enabled() const = 0;
    virtual bool logging_failed() const = 0;

    virtual void vehicle_was_disarmed() { };

protected:
    uint32_t dropped;
    uint8_t internal_errors; // uint8_t - wishful thinking?

    DataFlash_Class &_front;

    virtual void periodic_10Hz(const uint32_t now);
    virtual void periodic_1Hz(const uint32_t now);
    virtual void periodic_fullrate(const uint32_t now);

    /*
    read and print a log entry using the format strings from the given structure
    */
    void _print_log_entry(uint8_t msg_type,
                          print_mode_fn print_mode,
                          AP_HAL::BetterStream *port);

    bool ShouldLog(bool is_critical);	
    bool ShouldPosData();
    virtual bool WritesOK() const = 0;
    virtual bool RawDataWritesOK() const = 0;
    virtual bool PosDataWritesOK() const = 0;
    virtual bool StartNewLogOK() const;
    virtual bool StartNewRawDataOK() const;
    virtual bool StartNewPosDataOK() const;

    /*
      read a block
    */
    virtual bool ReadBlock(void *pkt, uint16_t size) = 0;

    virtual bool WriteBlockCheckStartupMessages();
    virtual void WriteMoreStartupMessages();
    virtual void push_log_blocks();

    DFMessageWriter_DFLogStart *_startup_messagewriter;
    bool _writing_startup_messages;

    uint32_t _internal_errors;
    uint32_t _dropped;

    // must be called when a new log is being started:
    virtual void start_new_log_reset_variables();

    virtual bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) = 0;

    virtual bool _WriteRawData(const void *pBuffer, uint16_t size, bool is_critical) = 0;
	
    virtual bool ReadRawData(void *pkt, uint16_t size) = 0;

    virtual bool _WritePosData(const void *pBuffer, uint16_t size, bool is_critical) = 0;
	
    virtual bool ReadPosData(void *pkt, uint16_t size) = 0;

    bool _initialised;
    bool _initialised_advance;

private:

    uint32_t _last_periodic_1Hz;
    uint32_t _last_periodic_10Hz;
    bool have_logged_armed;
};
