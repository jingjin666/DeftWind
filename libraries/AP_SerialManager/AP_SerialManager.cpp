/*
   Please contribute your ideas! See http://dev.ardupilot.org for details

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
  SerialManager allows defining the protocol and baud rates for the available
  serial ports and provides helper functions so objects (like a gimbal) can
  find which serial port they should use
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_SerialManager.h"
#include <stdio.h>
extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define SERIAL0_PROTOCOL SerialProtocol_MAVLink2
#define SERIAL0_BAUD AP_SERIALMANAGER_MAVLINK_BAUD/1000

#define SERIAL1_PROTOCOL SerialProtocol_MAVLink2
#define SERIAL1_BAUD AP_SERIALMANAGER_MAVLINK_BAUD/1000

#define SERIAL2_PROTOCOL SerialProtocol_GPS
#define SERIAL2_BAUD AP_SERIALMANAGER_GPS_BAUD/1000

#define SERIAL3_PROTOCOL SerialProtocol_GPS
#define SERIAL3_BAUD AP_SERIALMANAGER_GPS_BAUD/1000

#define SERIAL4_PROTOCOL SerialProtocol_GPS
#define SERIAL4_BAUD AP_SERIALMANAGER_GPS_BAUD/1000

#define SERIAL5_PROTOCOL SerialProtocol_None
#define SERIAL5_BAUD AP_SERIALMANAGER_BACKUP_BAUD/1000

#define SERIAL6_PROTOCOL SerialProtocol_None
#define SERIAL6_BAUD AP_SERIALMANAGER_BACKUP_BAUD/1000
#elif CONFIG_HAL_BOARD == HAL_BOARD_UAVRS
#if defined(CONFIG_ARCH_BOARD_UAVRS_V1)
#define SERIAL0_PROTOCOL SerialProtocol_MAVLink2
#define SERIAL0_BAUD AP_SERIALMANAGER_MAVLINK_BAUD/1000

#define SERIAL1_PROTOCOL SerialProtocol_MAVLink2
#define SERIAL1_BAUD AP_SERIALMANAGER_MAVLINK_BAUD/1000

#define SERIAL2_PROTOCOL SerialProtocol_GPS
#define SERIAL2_BAUD AP_SERIALMANAGER_GPS_BAUD/1000

#define SERIAL3_PROTOCOL SerialProtocol_Nova_Rtcm
#define SERIAL3_BAUD AP_SERIALMANAGER_RTCM_BAUD/1000

#define SERIAL4_PROTOCOL SerialProtocol_GPS
#define SERIAL4_BAUD AP_SERIALMANAGER_GPS_BAUD/1000

#define SERIAL5_PROTOCOL SerialProtocol_None
#define SERIAL5_BAUD AP_SERIALMANAGER_BACKUP_BAUD/1000

#define SERIAL6_PROTOCOL SerialProtocol_None
#define SERIAL6_BAUD AP_SERIALMANAGER_BACKUP_BAUD/1000
#elif defined(CONFIG_ARCH_BOARD_UAVRS_V2)
#define SERIAL0_PROTOCOL SerialProtocol_MAVLink2
#define SERIAL0_BAUD AP_SERIALMANAGER_MAVLINK_BAUD/1000

#define SERIAL1_PROTOCOL SerialProtocol_Nova_Rtcm
#define SERIAL1_BAUD AP_SERIALMANAGER_RTCM_BAUD/1000

#define SERIAL2_PROTOCOL SerialProtocol_GPS
#define SERIAL2_BAUD AP_SERIALMANAGER_GPS_BAUD/1000

#define SERIAL3_PROTOCOL SerialProtocol_None
#define SERIAL3_BAUD AP_SERIALMANAGER_BACKUP_BAUD/1000

#define SERIAL4_PROTOCOL SerialProtocol_None
#define SERIAL4_BAUD AP_SERIALMANAGER_MAVLINK_BAUD/1000

#define SERIAL5_PROTOCOL SerialProtocol_MAVLink2
#define SERIAL5_BAUD AP_SERIALMANAGER_MAVLINK_BAUD/1000

#define SERIAL6_PROTOCOL SerialProtocol_GPS
#define SERIAL6_BAUD AP_SERIALMANAGER_GPS_BAUD/1000
#else
#error "Unknown board type"
#endif
#endif

const AP_Param::GroupInfo AP_SerialManager::var_info[] = {
    // @Param: 0_PROTOCOL
    // @DisplayName: Console protocol selection
    // @Description: Control what protocol to use on the console. 
    // @Values: 1:MAVlink1, 2:MAVLink2
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("0_PROTOCOL",  0, AP_SerialManager, state[0].protocol, SERIAL0_PROTOCOL),

    // @Param: 0_BAUD
    // @DisplayName: Serial0 baud rate
    // @Description: The baud rate used on the USB console. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,460:460800,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("0_BAUD",  1, AP_SerialManager, state[0].baud, SERIAL0_BAUD),
    
    // @Param: 1_PROTOCOL
    // @DisplayName: Telem1 protocol selection
    // @Description: Control what protocol to use on the Telem1 port. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @Values: -1:None, 1:MAVLink1, 2:MAVLink2, 3:Frsky D, 4:Frsky SPort, 5:GPS, 7:Alexmos Gimbal Serial, 8:SToRM32 Gimbal Serial, 9:Lidar, 10:FrSky SPort Passthrough (OpenTX), 11:Lidar360, 12:Aerotenna uLanding, 13:Beacon
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("1_PROTOCOL",  2, AP_SerialManager, state[1].protocol, SERIAL1_PROTOCOL),

    // @Param: 1_BAUD
    // @DisplayName: Telem1 Baud Rate
    // @Description: The baud rate used on the Telem1 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("1_BAUD", 3, AP_SerialManager, state[1].baud, SERIAL1_BAUD),

    // @Param: 2_PROTOCOL
    // @DisplayName: Telemetry 2 protocol selection
    // @Description: Control what protocol to use on the Telem2 port. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @Values: -1:None, 1:MAVLink1, 2:MAVLink2, 3:Frsky D, 4:Frsky SPort, 5:GPS, 7:Alexmos Gimbal Serial, 8:SToRM32 Gimbal Serial, 9:Lidar, 10:FrSky SPort Passthrough (OpenTX), 11:Lidar360, 12:Aerotenna uLanding, 13:Beacon
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("2_PROTOCOL",  4, AP_SerialManager, state[2].protocol, SERIAL2_PROTOCOL),

    // @Param: 2_BAUD
    // @DisplayName: Telemetry 2 Baud Rate
    // @Description: The baud rate of the Telem2 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("2_BAUD", 5, AP_SerialManager, state[2].baud, SERIAL2_BAUD),

    // @Param: 3_PROTOCOL
    // @DisplayName: Serial 3 (GPS) protocol selection
    // @Description: Control what protocol Serial 3 (GPS) should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @Values: -1:None, 1:MAVLink1, 2:MAVLink2, 3:Frsky D, 4:Frsky SPort, 5:GPS, 7:Alexmos Gimbal Serial, 8:SToRM32 Gimbal Serial, 9:Lidar, 10:FrSky SPort Passthrough (OpenTX), 11:Lidar360, 12:Aerotenna uLanding, 13:Beacon
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("3_PROTOCOL",  6, AP_SerialManager, state[3].protocol, SERIAL3_PROTOCOL),

    // @Param: 3_BAUD
    // @DisplayName: Serial 3 (GPS) Baud Rate
    // @Description: The baud rate used for the Serial 3 (GPS). The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("3_BAUD", 7, AP_SerialManager, state[3].baud, SERIAL3_BAUD),

    // @Param: 4_PROTOCOL
    // @DisplayName: Serial4 protocol selection
    // @Description: Control what protocol Serial4 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @Values: -1:None, 1:MAVLink1, 2:MAVLink2, 3:Frsky D, 4:Frsky SPort, 5:GPS, 7:Alexmos Gimbal Serial, 8:SToRM32 Gimbal Serial, 9:Lidar, 10:FrSky SPort Passthrough (OpenTX), 11:Lidar360, 12:Aerotenna uLanding, 13:Beacon
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("4_PROTOCOL",  8, AP_SerialManager, state[4].protocol, SERIAL4_PROTOCOL),

    // @Param: 4_BAUD
    // @DisplayName: Serial 4 Baud Rate
    // @Description: The baud rate used for Serial4. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("4_BAUD", 9, AP_SerialManager, state[4].baud, SERIAL4_BAUD),

    // @Param: 5_PROTOCOL
    // @DisplayName: Serial5 protocol selection
    // @Description: Control what protocol Serial5 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @Values: -1:None, 1:MAVLink1, 2:MAVLink2, 3:Frsky D, 4:Frsky SPort, 5:GPS, 7:Alexmos Gimbal Serial, 8:SToRM32 Gimbal Serial, 9:Lidar, 10:FrSky SPort Passthrough (OpenTX), 11:Lidar360, 12:Aerotenna uLanding, 13:Beacon
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("5_PROTOCOL",  10, AP_SerialManager, state[5].protocol, SERIAL5_PROTOCOL),

    // @Param: 5_BAUD
    // @DisplayName: Serial 5 Baud Rate
    // @Description: The baud rate used for Serial5. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("5_BAUD", 11, AP_SerialManager, state[5].baud, SERIAL5_BAUD),

    // @Param: 6_PROTOCOL
    // @DisplayName: Serial6 protocol selection
    // @Description: Control what protocol Serial5 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @Values: -1:None, 1:MAVLink1, 2:MAVLink2, 3:Frsky D, 4:Frsky SPort, 5:GPS, 7:Alexmos Gimbal Serial, 8:SToRM32 Gimbal Serial, 9:Lidar, 10:FrSky SPort Passthrough (OpenTX), 11:Lidar360, 12:Aerotenna uLanding, 13:Beacon
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("6_PROTOCOL",  12, AP_SerialManager, state[6].protocol, SERIAL6_PROTOCOL),

    // @Param: 6_BAUD
    // @DisplayName: Serial 6 Baud Rate
    // @Description: The baud rate used for Serial5. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("6_BAUD", 13, AP_SerialManager, state[6].baud, SERIAL6_BAUD),

    AP_GROUPEND
};

// Constructor
AP_SerialManager::AP_SerialManager()
{
    // setup parameter defaults
    AP_Param::setup_object_defaults(this, var_info);
}

// init_console - initialise console at default baud rate
void AP_SerialManager::init_console()
{
    // initialise console immediately at default size and baud
    state[0].uart = hal.uartA;  // serial0, uartA, always console
    state[0].uart->begin(AP_SERIALMANAGER_CONSOLE_BAUD,
                         AP_SERIALMANAGER_CONSOLE_BUFSIZE_RX,
                         AP_SERIALMANAGER_CONSOLE_BUFSIZE_TX);
}

extern bool g_nsh_should_exit;

// init - // init - initialise serial ports
void AP_SerialManager::init()
{
    // initialise pointers to serial ports
    state[1].uart = hal.uartB;
    state[2].uart = hal.uartC;
    state[3].uart = hal.uartD;
    state[4].uart = hal.uartE;
    state[5].uart = hal.uartF;
    state[6].uart = hal.uartG;

    if (state[0].uart == nullptr) {
        init_console();
    }
    
    // initialise serial ports
    for (uint8_t i=1; i<SERIALMANAGER_NUM_PORTS; i++) {        
        if (state[i].uart != nullptr) {
            switch (state[i].protocol) {
                case SerialProtocol_None:
                    break;
                case SerialProtocol_Console:
                case SerialProtocol_MAVLink:
                case SerialProtocol_MAVLink2:
                    state[i].uart->begin(map_baudrate(state[i].baud), 
                                         AP_SERIALMANAGER_MAVLINK_BUFSIZE_RX,
                                         AP_SERIALMANAGER_MAVLINK_BUFSIZE_TX);
                    break;
                case SerialProtocol_GPS:
                case SerialProtocol_GPS2:
                    state[i].uart->begin(map_baudrate(state[i].baud), 
                                         AP_SERIALMANAGER_GPS_BUFSIZE_RX,
                                         AP_SERIALMANAGER_GPS_BUFSIZE_TX);
                    break;
				case SerialProtocol_Nova_Rtcm:
                    state[i].uart->begin(map_baudrate(state[i].baud),
                                         AP_SERIALMANAGER_RTCM_BUFSIZE_RX,
                                         AP_SERIALMANAGER_RTCM_BUFSIZE_TX);
                    break;
            }
        }
    }
}

// find_serial - searches available serial ports for the first instance that allows the given protocol
//  instance should be zero if searching for the first instance, 1 for the second, etc
//  returns uart on success, nullptr if a serial port cannot be found
AP_HAL::UARTDriver *AP_SerialManager::find_serial(enum SerialProtocol protocol, uint8_t instance) const
{
    uint8_t found_instance = 0;

    // search for matching protocol
    for(uint8_t i=0; i<SERIALMANAGER_NUM_PORTS; i++) {
        if (protocol_match(protocol, (enum SerialProtocol)state[i].protocol.get())) {
            if (found_instance == instance) {
                return state[i].uart;
            }
            found_instance++;
        }
    }

    // if we got this far we did not find the uart
    return nullptr;
}

// find_baudrate - searches available serial ports for the first instance that allows the given protocol
//  instance should be zero if searching for the first instance, 1 for the second, etc
//  returns baudrate on success, 0 if a serial port cannot be found
uint32_t AP_SerialManager::find_baudrate(enum SerialProtocol protocol, uint8_t instance) const
{
    uint8_t found_instance = 0;

    // search for matching protocol
    for(uint8_t i=0; i<SERIALMANAGER_NUM_PORTS; i++) {
        if (protocol_match(protocol, (enum SerialProtocol)state[i].protocol.get())) {
            if (found_instance == instance) {
                return map_baudrate(state[i].baud);
            }
            found_instance++;
        }
    }

    // if we got this far we did not find the uart
    return 0;
}

// get_mavlink_channel - provides the mavlink channel associated with a given protocol
//  instance should be zero if searching for the first instance, 1 for the second, etc
//  returns true if a channel is found, false if not
bool AP_SerialManager::get_mavlink_channel(enum SerialProtocol protocol, uint8_t instance, mavlink_channel_t &mav_chan) const
{
    // check for MAVLink
    if (protocol_match(protocol, SerialProtocol_MAVLink)) {
        if (instance < MAVLINK_COMM_NUM_BUFFERS) {
            mav_chan = (mavlink_channel_t)(MAVLINK_COMM_0 + instance);
            return true;
        }
    }
    // report failure
    return false;
}


// get_mavlink_protocol - provides the specific MAVLink protocol for a
// given channel, or SerialProtocol_None if not found
AP_SerialManager::SerialProtocol AP_SerialManager::get_mavlink_protocol(mavlink_channel_t mav_chan) const
{
    uint8_t instance = 0;
    uint8_t chan_idx = (uint8_t)(mav_chan - MAVLINK_COMM_0);
    for (uint8_t i=0; i<SERIALMANAGER_NUM_PORTS; i++) {
        if (state[i].protocol == SerialProtocol_MAVLink ||
            state[i].protocol == SerialProtocol_MAVLink2) {
            if (instance == chan_idx) {
                return (SerialProtocol)state[i].protocol.get();
            }
            instance++;
        }
    }
    return SerialProtocol_None;
}

// set_blocking_writes_all - sets block_writes on or off for all serial channels
void AP_SerialManager::set_blocking_writes_all(bool blocking)
{
    // set block_writes for all initialised serial ports
    for (uint8_t i=0; i<SERIALMANAGER_NUM_PORTS; i++) {
        if (state[i].uart != nullptr) {
            state[i].uart->set_blocking_writes(blocking);
        }
    }
}

// set_console_baud - sets the console's baud rate to the rate specified by the protocol
void AP_SerialManager::set_console_baud(enum SerialProtocol protocol, uint8_t instance) const
{
    uint8_t found_instance = 0;

    // find baud rate of this protocol
    for (uint8_t i=0; i<SERIALMANAGER_NUM_PORTS; i++) {
        if (protocol_match(protocol, (enum SerialProtocol)state[i].protocol.get())) {
            if (instance == found_instance) {
                // set console's baud rate
                state[0].uart->begin(map_baudrate(state[i].baud));
                return;
            }
            found_instance++;
        }
    }
}

/*
 *  map from a 16 bit EEPROM baud rate to a real baud rate.
 *  For PX4 we can do 1.5MBit, although 921600 is more reliable.
 */
uint32_t AP_SerialManager::map_baudrate(int32_t rate) const
{
    if (rate <= 0) {
        rate = 57;
    }
    switch (rate) {
    case 1:    return 1200;
    case 2:    return 2400;
    case 4:    return 4800;
    case 9:    return 9600;
    case 19:   return 19200;
    case 38:   return 38400;
    case 57:   return 57600;
    case 100:  return 100000;
    case 111:  return 111100;
    case 115:  return 115200;
    case 230:  return 230400;
    case 460:  return 460800;
    case 500:  return 500000;
    case 921:  return 921600;
    case 1500:  return 1500000;
    }

    if (rate > 2000) {
        // assume it is a direct baudrate. This allows for users to
        // set an exact baudrate as long as it is over 2000 baud
        return (uint32_t)rate;
    }

    // otherwise allow any other kbaud rate
    return rate*1000;
}

// protocol_match - returns true if the protocols match
bool AP_SerialManager::protocol_match(enum SerialProtocol protocol1, enum SerialProtocol protocol2) const
{
    // check for obvious match
    if (protocol1 == protocol2) {
        return true;
    }

    // mavlink match
    if (((protocol1 == SerialProtocol_MAVLink) || (protocol1 == SerialProtocol_MAVLink2)) &&
        ((protocol2 == SerialProtocol_MAVLink) || (protocol2 == SerialProtocol_MAVLink2))) {
        return true;
    }

    // gps match
    if (((protocol1 == SerialProtocol_GPS) || (protocol1 == SerialProtocol_GPS2)) &&
        ((protocol2 == SerialProtocol_GPS) || (protocol2 == SerialProtocol_GPS2))) {
        return true;
    }

    return false;
}
