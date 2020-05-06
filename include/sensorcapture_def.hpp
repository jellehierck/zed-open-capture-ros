#ifndef SENSORCAPTURE_DEF_HPP
#define SENSORCAPTURE_DEF_HPP

#define DEFAULT_GRAVITY (9.8189f)
#define ACC_SCALE       (DEFAULT_GRAVITY*(8.0f/32768.0f))
#define GYRO_SCALE      (1000.0f/32768.0f)
#define MAG_SCALE       (1.0f/16.0f)
#define TEMP_SCALE      (0.01f)
#define PRESS_SCALE_NEW (0.0001f)       // FM >= V3.9
#define PRESS_SCALE_OLD (0.01f)         // FW < v3.9
#define HUMID_SCALE_NEW (0.01f)         // FM >= V3.9
#define HUMID_SCALE_OLD (1.0f/1024.0f)  // FW < v3.9

#define TS_SCALE        (39062.5f)
#define TEMP_NOT_VALID  (-27315)

namespace sl_drv {

// ----> Command to be used with the REPORT ID "REP_ID_REQUEST_SET"
// Command to ping the MCU to communicate that host is alive
#define RQ_CMD_PING 0xF2
// <---- Command to be used with the REPORT ID "REP_ID_REQUEST_SET"

/*!
 * \brief USB HID communication report IDs
 */
typedef enum CUSTOMHID_REPORT_ID {
    // Input Reports
    REP_ID_SENSOR_DATA          = 0x01,  //!< Sensor data report ID

    // Generic commands
    REP_ID_REQUEST_SET          = 0x21, //!< USB Request report ID

    // Features Reports
    REP_ID_SENSOR_STREAM_STATUS = 0x32, //!< Stream Status report ID

} CUSTOMHID_REPORT_ID;

#pragma pack(push)  // push current alignment to stack
#pragma pack(1)     // set alignment to 1 byte boundary

/*!
 * \brief The RAW sensor data structure retrieved from camera MCU by USB
 */
typedef struct SensData {
    uint8_t struct_id;		//!< Struct identifier, it matches the USB HID Report ID
    uint8_t imu_not_valid; 	//!< Indicate if IMU data are valid [0->valid, 1->not_valid
    uint64_t timestamp;		//!< Data timestamp (from IMU sensor) [usec/39]
    int16_t gX;				//!< Raw Gyroscope X
    int16_t gY;				//!< Raw Gyroscope Y
    int16_t gZ;				//!< Raw Gyroscope Z
    int16_t aX;				//!< Raw Accelerometer X
    int16_t aY;				//!< Raw Accelerometer Y
    int16_t aZ;				//!< Raw Accelerometer Z
    uint8_t frame_sync;		//!< Indicates if data are synced to a camera frame
    uint8_t sync_capabilities; //!< Indicates if frame synchronization is active
    uint32_t frame_sync_count; //!< Counts the number of synced frames
    int16_t imu_temp;		//!< Temperature from the IMU sensor [0.01 °C]
    uint8_t mag_valid;		//!< Indicates if Magnetometer data are valid (put to 0 if no magnetometer is present)
    int16_t mX;				//!< Raw Magnetometer X
    int16_t mY;				//!< Raw Magnetometer Y
    int16_t mZ;				//!< Raw Magnetometer Z
    uint8_t camera_moving;	//!< Indicate if the camera is moving (uses BMI internal HW)
    uint32_t camera_moving_count; //!< Counts the number of camera moving interrupts
    uint8_t camera_falling;	//!< Indicate if the camera is free falling (uses BMI internal HW)
    uint32_t camera_falling_count; //!< Counts the number of camera falling interrupts
    uint8_t env_valid;		//!< Indicate if Environmental data are valid (put to `ENV_SENS_NOT_PRESENT` if no environmental sensor is present)
    int16_t temp;			//!< Temperature [0.01 °C]
    uint32_t press;			//!< Pressure [0.01 hPa]
    uint32_t humid;			//!< Relative humidity [1.0/1024.0 %rH]
    int16_t temp_cam_left;	//!< Temperature of the left camera sensor [0.01 °C]
    int16_t temp_cam_right; //!< Temperature of the right camera sensor [0.01 °C]
} SensData;

/*!
 * Status of the data streaming
 */
typedef struct SensStreamStatus {
    uint8_t struct_id;		//!< Struct identifier, it matches the USB HID Report ID
    uint8_t stream_status;	//!< Status of the USB streaming
} SensStreamStatus;

#pragma pack(pop) // Restore previous saved alignment

// ----> FW versions
enum class ZED_M_FW {
    FW_2_2 = 514, //!< ZED Mini v2.2
    FW_2_3 = 515, //!< ZED Mini v2.3
    FW_2_4 = 516, //!< ZED Mini v2.4 (not released)
    FW_2_5 = 517, //!< ZED Mini v2.5
    LAST
};

enum class ZED_2_FW {
    FW_3_4 = 772,  //!< ZED2 v3.4
    FW_3_5 = 773,  //!< ZED2 v3.5
    FW_3_6 = 774,  //!< ZED2 v3.6
    FW_3_7 = 775,  //!< ZED2 v3.7
    FW_3_8 = 776,  //!< ZED2 v3.8
    FW_3_9 = 777   //!< ZED2 v3.9
};

/*!
 * \brief Check firmware version for ZED2 camera
 * \param version_current the current FW version
 * \param version_required the FW version to compare
 * \return
 */
inline bool atLeast(const int &version_current, const ZED_2_FW &version_required) {
    return (version_current >= static_cast<int>(version_required));
}

/*!
 * \brief Check firmware version for ZED Mini camera
 * \param version_current the current FW version
 * \param version_required the FW version to compare
 * \return
 */
inline bool atLeast(const int &version_current, const ZED_M_FW &version_required) {
    return (version_current >= static_cast<int>(version_required));
}
// <---- FW versions


#define NTP_ADJUST_CT 1
const size_t TS_SHIFT_VAL_COUNT = 50; //!< Number of sensor data to use to update timestamp scaling

}

#endif // SENSORCAPTURE_DEF_HPP