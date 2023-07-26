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
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_CTRLPOS_ENABLED
#define AP_CTRLPOS_ENABLED 1
#endif

#ifndef HAL_MSP_CTRLPOS_ENABLED
#define HAL_MSP_CTRLPOS_ENABLED (AP_CTRLPOS_ENABLED && (HAL_MSP_ENABLED && !HAL_MINIMIZE_FEATURES))
#endif

#ifndef AP_CTRLPOS_SITL_ENABLED
#define AP_CTRLPOS_SITL_ENABLED AP_SIM_ENABLED
#endif

#if AP_CTRLPOS_ENABLED

/*
 * AP_OpticalFlow.h - OpticalFlow Base Class for ArduPilot
 */

#include <AP_MSP/msp.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_CtrlPos_Calibrator.h"
#include "AP_CtrlPos_PX4Flow.h"

class CtrlPos_backend;

class AP_CtrlPos
{
    friend class CtrlPos_backend;

public:
    AP_CtrlPos();

    CLASS_NO_COPY(AP_CtrlPos);

    // get singleton instance
    static AP_CtrlPos *get_singleton() {
        return _singleton;
    }

    enum class Type {
        NONE = 0,
        PX4FLOW = 1,
    };

    // init - initialise sensor
    void init(uint32_t log_bit);

    // enabled - returns true if optical flow is enabled
    bool enabled() const { return _type != Type::NONE; }

    // healthy - return true if the sensor is healthy
    bool healthy() const { return backend != nullptr && _flags.healthy; }

    // read latest values from sensor and fill in x,y and totals.
    void update(void);

    // handle optical flow mavlink messages
    void handle_msg(const mavlink_message_t &msg);
/*
#if HAL_MSP_CTRLPOS_ENABLED
    // handle optical flow msp messages
    void handle_msp(const MSP::msp_opflow_data_message_t &pkt);
#endif
*/
    // quality - returns the surface quality as a measure from 0 ~ 255
    uint8_t quality() const { return _state.surface_quality; }

    // raw - returns the raw movement from the sensor
    const Vector2f& flowRate() const { return _state.flowRate; }

    // velocity - returns the velocity in m/s
    const Vector2f& bodyRate() const { return _state.bodyRate; }

    // last_update() - returns system time of last sensor update
    uint32_t last_update() const { return _last_update_ms; }

    // get_height_override() - returns the user-specified height of sensor above ground
    float get_height_override() const { return 2.0f; }

    // including our collected numbers in the log packets...
    float getFirstNum() const {return _state.first_number; }
    float getSecondNum() const {return _state.second_number; }
    float getThirdNum() const {return _state.third_number; }
    struct CtrlPos_state {
        uint8_t  surface_quality;   // image quality (below TBD you can't trust the dx,dy values returned)
        Vector2f flowRate;          // optical flow angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.
        Vector2f bodyRate;          // body inertial angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.

        // added this in
        float first_number;
        float second_number;
        float third_number;
    
    };

    // return a 3D vector defining the position offset of the sensors focal point in metres relative to the body frame origin
    const Vector3f & get_pos_offset(void) const {
        return _pos_offset;
    }

    // start or stop calibration
    void start_calibration();
    void stop_calibration();

    // parameter var info table
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AP_CtrlPos *_singleton;

    CtrlPos_backend *backend;

    struct AP_CtrlPos_Flags {
        uint8_t healthy     : 1;    // true if sensor is healthy
    } _flags;

    // parameters
    AP_Enum<Type>  _type;           // user configurable sensor type
    AP_Int16 _flowScalerX;          // X axis flow scale factor correction - parts per thousand
    AP_Int16 _flowScalerY;          // Y axis flow scale factor correction - parts per thousand
    AP_Int16 _yawAngle_cd;          // yaw angle of sensor X axis with respect to vehicle X axis - centi degrees
    AP_Vector3f _pos_offset;        // position offset of the flow sensor in the body frame
    AP_Int8  _address;              // address on the bus (allows selecting between 8 possible I2C addresses for px4flow)

    // method called by backend to update frontend state:
    void update_state(const CtrlPos_state &state);

    // state filled in by backend
    struct CtrlPos_state _state;

    uint32_t _last_update_ms;        // millis() time of last update

    void Log_Write_CtrlPos();
    uint32_t _log_bit = -1;     // bitmask bit which indicates if we should log.  -1 means we always log

    // calibrator
    AP_CtrlPos_Calibrator *_calibrator;

};

namespace AP {
    AP_CtrlPos *ctrlpos();
}

#include "AP_CtrlPos_Backend.h"

#endif // AP_CTRLPOS_ENABLED
