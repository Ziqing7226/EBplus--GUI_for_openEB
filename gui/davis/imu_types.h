// gui/davis/imu_types.h — shared IMU sample type for the inivation device
// layer (DAVIS + DVXplorer). The wire protocol (16-bit word stream, IMU
// start/end specials, misc8 tag/data sequence) is ported from dv-processing
// 2.0.4 parsers (Apache-2.0); the scales arrive in-band via the stream's
// IMU Scale Config event, so no range assumptions are needed here.

#ifndef GUI_DAVIS_IMU_TYPES_H
#define GUI_DAVIS_IMU_TYPES_H

#include <cstdint>
#include <functional>

namespace gui::davis {

/// One completed IMU6 sample (accelerometer + gyroscope + temperature).
/// Units follow the reference ecosystem: accelerations in g, angular rates
/// in °/s, temperature in °C.
struct ImuSample {
    std::int64_t t{0};  ///< Stream time (rebased µs, same clock as events).
    float accel_x{0};   ///< Accelerometer X (g).
    float accel_y{0};   ///< Accelerometer Y (g).
    float accel_z{0};   ///< Accelerometer Z (g).
    float gyro_x{0};    ///< Gyroscope X (°/s).
    float gyro_y{0};    ///< Gyroscope Y (°/s).
    float gyro_z{0};    ///< Gyroscope Z (°/s).
    float temperature{0};  ///< IMU die temperature (°C).
    bool valid{false};     ///< Set by the decoder when a full sample lands.
};

using ImuSink = std::function<void(const ImuSample&)>;

/// IMU chip models reported by the MODULE_IMU / IMU_TYPE register. Only the
/// InvenSense 6500/9250 family needs a distinct temperature formula.
enum class ImuModel : std::uint8_t {
    None = 0,
    InvenSense6050_6150 = 1,
    InvenSense6500_9250 = 2,
    BoschBMI160 = 3,
    BoschBMI270 = 4,
};

} // namespace gui::davis

#endif // GUI_DAVIS_IMU_TYPES_H
