#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>

#define PINNACLE_READ 0xA0
#define PINNACLE_WRITE 0x80

#define PINNACLE_AUTOINC 0xFC
#define PINNACLE_FILLER 0xFB

// Registers
#define PINNACLE_FW_ID 0x00   // ASIC ID.
#define PINNACLE_FW_VER 0x01  // Firmware Version Firmware revision number.
#define PINNACLE_STATUS1 0x02 // Contains status flags about the state of Pinnacle.
#define PINNACLE_STATUS1_SW_DR BIT(2)
#define PINNACLE_STATUS1_SW_CC BIT(3)
#define PINNACLE_SYS_CFG 0x03 // Contains system operation and configuration bits.
#define PINNACLE_SYS_CFG_EN_SLEEP_BIT 2
#define PINNACLE_SYS_CFG_EN_SLEEP BIT(2)
#define PINNACLE_SYS_CFG_SHUTDOWN BIT(1)
#define PINNACLE_SYS_CFG_RESET BIT(0)

#define PINNACLE_FEED_CFG1 0x04 // Contains feed operation and configuration bits.
#define PINNACLE_FEED_CFG1_EN_FEED BIT(0)
#define PINNACLE_FEED_CFG1_ABS_MODE BIT(1)
#define PINNACLE_FEED_CFG1_DIS_FILT BIT(2)
#define PINNACLE_FEED_CFG1_DIS_X BIT(3)
#define PINNACLE_FEED_CFG1_DIS_Y BIT(4)
#define PINNACLE_FEED_CFG1_INV_X BIT(6)
#define PINNACLE_FEED_CFG1_INV_Y BIT(7)
#define PINNACLE_FEED_CFG2 0x05               // Contains feed operation and configuration bits.
#define PINNACLE_FEED_CFG2_EN_IM BIT(0)       // Intellimouse
#define PINNACLE_FEED_CFG2_DIS_TAP BIT(1)     // Disable all taps
#define PINNACLE_FEED_CFG2_DIS_SEC BIT(2)     // Disable secondary tap
#define PINNACLE_FEED_CFG2_DIS_SCRL BIT(3)    // Disable scroll
#define PINNACLE_FEED_CFG2_DIS_GE BIT(4)      // Disable GlideExtend
#define PINNACLE_FEED_CFG2_EN_BTN_SCRL BIT(6) // Enable Button Scroll
#define PINNACLE_FEED_CFG2_ROTATE_90 BIT(7)   // Swap X & Y
#define PINNACLE_CAL_CFG 0x07                 // Contains calibration configuration bits.
#define PINNACLE_PS2_AUX 0x08                 // Contains Data register for PS/2 Aux Control.
#define PINNACLE_SAMPLE 0x09                  // Sample Rate Number of samples generated per second.
#define PINNACLE_Z_IDLE 0x0A         // Number of Z=0 packets sent when Z goes from >0 to 0.
#define PINNACLE_Z_SCALER 0x0B       // Contains the pen Z_On threshold.
#define PINNACLE_SLEEP_INTERVAL 0x0C // Sleep Interval
#define PINNACLE_SLEEP_TIMER 0x0D    // Sleep Timer
#define PINNACLE_AG_PACKET0 0x10     // trackpad Data (Pinnacle AG)
#define PINNACLE_2_2_PACKET0 0x12    // trackpad Data
#define PINNACLE_REG_COUNT 0x18

#define PINNACLE_REG_ERA_VALUE 0x1B
#define PINNACLE_REG_ERA_HIGH_BYTE 0x1C
#define PINNACLE_REG_ERA_LOW_BYTE 0x1D
#define PINNACLE_REG_ERA_CONTROL 0x1E

#define PINNACLE_ERA_CONTROL_READ 0x01
#define PINNACLE_ERA_CONTROL_WRITE 0x02

#define PINNACLE_ERA_REG_X_AXIS_WIDE_Z_MIN 0x0149
#define PINNACLE_ERA_REG_Y_AXIS_WIDE_Z_MIN 0x0168
#define PINNACLE_ERA_REG_TRACKING_ADC_CONFIG 0x0187

#define PINNACLE_TRACKING_ADC_CONFIG_1X 0x00
#define PINNACLE_TRACKING_ADC_CONFIG_2X 0x40
#define PINNACLE_TRACKING_ADC_CONFIG_3X 0x80
#define PINNACLE_TRACKING_ADC_CONFIG_4X 0xC0

#define PINNACLE_PACKET0_BTN_PRIM BIT(0) // Primary button
#define PINNACLE_PACKET0_BTN_SEC BIT(1)  // Secondary button
#define PINNACLE_PACKET0_BTN_AUX BIT(2)  // Auxiliary (middle?) button
#define PINNACLE_PACKET0_X_SIGN BIT(4)   // X delta sign
#define PINNACLE_PACKET0_Y_SIGN BIT(5)   // Y delta sign

struct pinnacle_inertia_axis {
    int32_t velocity_x1000;     // Velocity scaled by 1000 for precision
    int64_t last_event_time;    // Last time this axis had movement
    bool active;                // Whether this axis has active inertia
    int32_t remainder_x1000;    // Sub-pixel remainder for smooth movement
};

enum pinnacle_tap_state {
    TAP_IDLE,           // No touch detected
    TAP_DOWN            // Touch down, checking for tap
};

struct pinnacle_tap_detector {
    enum pinnacle_tap_state state;
    int64_t touch_start_time;   // When current touch started
    uint16_t initial_x, initial_y;  // Position where touch started
    uint16_t max_movement;      // Maximum movement during current touch
    bool tap_pending;           // Tap event ready to be sent
};

struct pinnacle_data {
    bool in_int;
    bool touch_active;
    uint16_t last_x, last_y;  // Previous absolute position for delta calculation
    bool position_valid;      // Whether we have a valid previous position
    const struct device *dev;
    struct gpio_callback gpio_cb;
    struct k_work work;
    // Inertia state
    struct pinnacle_inertia_axis inertia_x, inertia_y;
    struct k_work_delayable inertia_work;
    struct k_mutex inertia_lock;
    // Tap detection state
    struct pinnacle_tap_detector tap_detector;
    struct k_work_delayable tap_release_work;  // For delayed button release
};

enum pinnacle_sensitivity {
    PINNACLE_SENSITIVITY_1X,
    PINNACLE_SENSITIVITY_2X,
    PINNACLE_SENSITIVITY_3X,
    PINNACLE_SENSITIVITY_4X,
};

typedef int (*pinnacle_seq_read_t)(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                   const uint8_t len);
typedef int (*pinnacle_write_t)(const struct device *dev, const uint8_t addr, const uint8_t val);

struct pinnacle_config {
    union {
        struct i2c_dt_spec i2c;
        struct spi_dt_spec spi;
    } bus;

    pinnacle_seq_read_t seq_read;
    pinnacle_write_t write;

    bool rotate_90, sleep_en, x_invert, y_invert, z_touch_detection;
    enum pinnacle_sensitivity sensitivity;
    uint8_t x_axis_z_min, y_axis_z_min;
    uint8_t z_threshold_touch, z_threshold_release;
    const struct gpio_dt_spec dr;
    // Inertia configuration
    bool inertia_enable;
    uint16_t inertia_start_velocity;
    uint16_t inertia_stop_velocity;
    uint16_t inertia_decay_rate;
    uint16_t inertia_update_interval_ms;
    // Tap detection configuration
    bool no_taps;
    uint16_t tap_timeout_ms;                // Maximum tap duration
    uint16_t tap_movement_threshold;        // Maximum movement for valid tap
};

int pinnacle_set_sleep(const struct device *dev, bool enabled);
