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

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_Compass_UAVCAN.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

extern const AP_HAL::HAL& hal;

#define debug_mag_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)

/*
  constructor - registers instance at top Compass driver
 */
AP_Compass_UAVCAN::AP_Compass_UAVCAN(Compass &compass):
    AP_Compass_Backend(compass)
{
    _mag_baro = hal.util->new_semaphore();
}

AP_Compass_UAVCAN::~AP_Compass_UAVCAN()
{
    if (_initialized)
    {
        if (hal.can_mgr[_manager] != nullptr) {
            AP_UAVCAN *ap_uavcan = hal.can_mgr[_manager]->get_UAVCAN();
            if (ap_uavcan != nullptr) {
                ap_uavcan->remove_mag_listener(this);

                //debug_mag_uavcan(2, "AP_Compass_UAVCAN destructed\n");
            }
        }
    }
}

AP_Compass_Backend *AP_Compass_UAVCAN::probe(Compass &compass)
{
    AP_Compass_UAVCAN *sensor = nullptr;

    if (AP_BoardConfig_CAN::get_can_num_ifaces() != 0) {
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (hal.can_mgr[i] != nullptr) {
                AP_UAVCAN *uavcan = hal.can_mgr[i]->get_UAVCAN();
                if (uavcan != nullptr) {
                    uint8_t freemag = uavcan->find_smallest_free_mag_node();
                    if (freemag != UINT8_MAX) {
                        sensor = new AP_Compass_UAVCAN(compass);
                        if (sensor->register_uavcan_compass(i, freemag)) {
                            debug_mag_uavcan(2, "AP_Compass_UAVCAN probed, CAN_Driver[%d], registered node id: %d\n", i, freemag);
                            return sensor;
                        } else {
                            delete sensor;
                            sensor = nullptr;
                        }
                    }
                }
            }
        }
    }
    printf("AP_Compass_UAVCAN probe error!\n");
    return sensor;
}

bool AP_Compass_UAVCAN::register_uavcan_compass(uint8_t mgr, uint8_t node)
{
    if (hal.can_mgr[mgr] != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr[mgr]->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            _manager = mgr;

            if (ap_uavcan->register_mag_listener_to_node(this, node)) {
                _instance = register_compass();

                struct DeviceStructure {
                    uint8_t bus_type : 3;
                    uint8_t bus: 5;
                    uint8_t address;
                    uint8_t devtype;
                };
                union DeviceId {
                    struct DeviceStructure devid_s;
                    uint32_t devid;
                };
                union DeviceId d;

                d.devid_s.bus_type = 3;
                d.devid_s.bus = mgr;
                d.devid_s.address = node;
                d.devid_s.devtype = 1;

                set_dev_id(_instance, d.devid);
                set_external(_instance, true);

                _sum.zero();
                _count = 0;
							
                accumulate();
				
                //debug_mag_uavcan(2, "AP_Compass_UAVCAN loaded\n");

                return true;
            }
        }
    }

    return false;
}

void AP_Compass_UAVCAN::read(void)
{
    // avoid division by zero if we haven't received any mag reports
    if (_count == 0) {
        return;
    }

    if (_mag_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
		#if 0
		_sum.x *= 0.89f;
		_sum.y *= 0.89f;
		_sum.z *= 0.84f;
		#endif
        _sum /= _count;

        publish_filtered_field(_sum, _instance);

        _sum.zero();
        _count = 0;
        _mag_baro->give();
    }
}

void AP_Compass_UAVCAN::handle_mag_msg(Vector3f &mag)
{	
	float _gain_scale = (1.0f / 1090) * 1000;

    Vector3f raw_field = mag * _gain_scale;
	//printf("raw_field [%.1f] [%.1f] [%.1f]\n", raw_field.x, raw_field.y, raw_field.z);

	// rotate to the desired orientation
    if (is_external(_instance)) {
        raw_field.rotate(ROTATION_YAW_90);
    }

    // rotate raw_field from sensor frame to body frame
    rotate_field(raw_field, _instance);

    _last_timestamp = AP_HAL::micros64();
    // publish raw_field (uncorrected point sample) for calibration use
    publish_raw_field(raw_field, (uint32_t) _last_timestamp, _instance);

    // correct raw_field for known errors
    correct_field(raw_field, _instance);

    if (_mag_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        // accumulate into averaging filter
        _sum += raw_field;
        _count++;

		if (_count == 14) {
            _sum /= 2;
            _count = 7;
        }
        _mag_baro->give();
    }
}

#endif
