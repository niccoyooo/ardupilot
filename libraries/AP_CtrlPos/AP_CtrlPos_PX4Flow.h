#pragma once

#include "AP_CtrlPos.h"



#ifndef AP_CTRLPOS_PX4FLOW_ENABLED
#define AP_CTRLPOS_PX4FLOW_ENABLED 1
#endif

#if AP_CTRLPOS_PX4FLOW_ENABLED

#include <AP_HAL/utility/OwnPtr.h>
#include "AP_CtrlPos_Backend.h"

class AP_CtrlPos_PX4Flow : public CtrlPos_backend {
    
    public:
        /// constructor
        using CtrlPos_backend::CtrlPos_backend;

        CLASS_NO_COPY(AP_CtrlPos_PX4Flow);

        // init - initialise the sensor
        void init() override {};

        // update - read latest values from sensor and fill in x,y and totals.
        void update(void) override;

        // detect if the sensor is available
        static AP_CtrlPos_PX4Flow *detect(AP_CtrlPos &_frontend);
        
        // access numbers from th 
        float getFirstNum(void);
        float getSecondNum(void);
        float getThirdNum(void);
        

    private:
        AP_HAL::OwnPtr<AP_HAL::Device> dev;

        static const uint8_t REG_INTEGRAL_FRAME = 0x16;
        
        // I2C data on register REG_INTEGRAL_FRAME
        struct PACKED i2c_integral_frame {
            uint16_t frame_count_since_last_readout;
            int16_t pixel_flow_x_integral;
            int16_t pixel_flow_y_integral;
            int16_t gyro_x_rate_integral;
            int16_t gyro_y_rate_integral;
            int16_t gyro_z_rate_integral;
            uint32_t integration_timespan;
            uint32_t sonar_timestamp;
            uint16_t ground_distance;
            int16_t gyro_temperature;
            uint8_t qual;
        };
        
        // scan I2C bus addresses and buses
        bool scan_buses(void);

        // setup sensor
        bool setup_sensor(void);

        void timer(void);
    
};

#endif // AP_CTRLPOS_PX4FLOW_ENABLED
