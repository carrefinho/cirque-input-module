#define DT_DRV_COMPAT cirque_pinnacle

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>

#include "input_pinnacle.h"

LOG_MODULE_REGISTER(pinnacle, CONFIG_INPUT_LOG_LEVEL);

static int pinnacle_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                             const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    return config->seq_read(dev, addr, buf, len);
}
static int pinnacle_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    return config->write(dev, addr, val);
}

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int pinnacle_i2c_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    return i2c_burst_read_dt(&config->bus.i2c, PINNACLE_READ | addr, buf, len);
}

static int pinnacle_i2c_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    return i2c_reg_write_byte_dt(&config->bus.i2c, PINNACLE_WRITE | addr, val);
}

#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int pinnacle_spi_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    uint8_t tx_buffer[len + 3], rx_dummy[3];
    tx_buffer[0] = PINNACLE_READ | addr;
    memset(&tx_buffer[1], PINNACLE_AUTOINC, len + 2);

    const struct spi_buf tx_buf[2] = {
        {
            .buf = tx_buffer,
            .len = len + 3,
        },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = 1,
    };
    struct spi_buf rx_buf[2] = {
        {
            .buf = rx_dummy,
            .len = 3,
        },
        {
            .buf = buf,
            .len = len,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = 2,
    };
    int ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);

    return ret;
}

static int pinnacle_spi_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    uint8_t tx_buffer[2] = {PINNACLE_WRITE | addr, val};
    uint8_t rx_buffer[2];

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = 2,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    const struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = 2,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    const int ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);

    if (ret < 0) {
        LOG_ERR("spi ret: %d", ret);
    }

    if (rx_buffer[1] != PINNACLE_FILLER) {
        LOG_ERR("bad ret val %d - %d", rx_buffer[0], rx_buffer[1]);
        return -EIO;
    }

    k_usleep(50);

    return ret;
}
#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int set_int(const struct device *dev, const bool en) {
    const struct pinnacle_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->dr,
                                              en ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }

    return ret;
}

static int pinnacle_clear_status(const struct device *dev) {
    int ret = pinnacle_write(dev, PINNACLE_STATUS1, 0);
    if (ret < 0) {
        LOG_ERR("Failed to clear STATUS1 register: %d", ret);
    }

    return ret;
}

static int pinnacle_era_read(const struct device *dev, const uint16_t addr, uint8_t *val) {
    int ret;

    set_int(dev, false);

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_CONTROL, PINNACLE_ERA_CONTROL_READ);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_VALUE, val, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read ERA value (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static int pinnacle_era_write(const struct device *dev, const uint16_t addr, uint8_t val) {
    int ret;

    set_int(dev, false);

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_VALUE, val);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA value (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_CONTROL, PINNACLE_ERA_CONTROL_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = pinnacle_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static uint16_t calculate_distance(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    int32_t dx = (int32_t)x1 - (int32_t)x2;
    int32_t dy = (int32_t)y1 - (int32_t)y2;
    // Simple Manhattan distance for efficiency
    return abs(dx) + abs(dy);
}

static void pinnacle_tap_release_work_handler(struct k_work *work) {
    struct k_work_delayable *delayable_work = k_work_delayable_from_work(work);
    struct pinnacle_data *data = CONTAINER_OF(delayable_work, struct pinnacle_data, tap_release_work);
    
    // Generate button release
    input_report_key(data->dev, INPUT_BTN_0, 0, false, K_FOREVER);
    LOG_DBG("Synthetic button release generated");
}

static void pinnacle_process_tap_detection(const struct device *dev, uint16_t x, uint16_t y, uint8_t z_level) {
    const struct pinnacle_config *config = dev->config;
    struct pinnacle_data *data = dev->data;
    struct pinnacle_tap_detector *tap = &data->tap_detector;
    
    if (config->no_taps) {
        return;
    }
    
    int64_t current_time = k_uptime_get();
    bool touch_detected = z_level >= config->z_threshold_touch;
    bool touch_released = z_level < config->z_threshold_release;
    
    switch (tap->state) {
    case TAP_IDLE:
        if (touch_detected) {
            // Touch down - start potential tap
            tap->state = TAP_DOWN;
            tap->touch_start_time = current_time;
            tap->initial_x = x;
            tap->initial_y = y;
            tap->max_movement = 0;
            LOG_DBG("Touch down at (%d,%d)", x, y);
        }
        break;
        
    case TAP_DOWN:
        if (touch_detected) {
            // Update movement tracking
            uint16_t movement = calculate_distance(x, y, tap->initial_x, tap->initial_y);
            if (movement > tap->max_movement) {
                tap->max_movement = movement;
            }
        } else if (touch_released) {
            // Touch up - check if it was a valid tap
            int64_t touch_duration = current_time - tap->touch_start_time;
            
            if (touch_duration <= config->tap_timeout_ms && 
                tap->max_movement <= config->tap_movement_threshold) {
                // Valid tap - mark for button event generation
                tap->tap_pending = true;
                LOG_DBG("Tap detected: duration=%lldms, movement=%d", touch_duration, tap->max_movement);
            } else {
                // Invalid tap (too long or too much movement)
                LOG_DBG("Invalid tap: duration=%lldms, movement=%d", touch_duration, tap->max_movement);
            }
            tap->state = TAP_IDLE;
        }
        break;
    }
}

static void pinnacle_report_data(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;
    uint8_t packet[6]; // 6-byte absolute packet
    int ret;
    ret = pinnacle_seq_read(dev, PINNACLE_STATUS1, packet, 1);
    if (ret < 0) {
        LOG_ERR("read status: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 1, "Pinnacle Status1");

    // Ignore 0xFF packets that indicate communcation failure, or if SW_DR isn't asserted
    if (packet[0] == 0xFF || !(packet[0] & PINNACLE_STATUS1_SW_DR)) {
        return;
    }

    // Read 6-byte absolute packet: 0x12-0x17 for Pinnacle 2.2
    ret = pinnacle_seq_read(dev, 0x12, packet, 6);
    if (ret < 0) {
        LOG_ERR("read packet: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 6, "Pinnacle Absolute Packets");

    struct pinnacle_data *data = dev->data;

    // Note: In absolute mode, packet[0] contains physical button states only
    // Tap detection will be implemented synthetically below

    // Extract absolute X/Y position according to datasheet Table 8
    // X: packet[2] (0x14 - X Position Low Byte) + lower nibble of packet[4] (0x16 - X11-X8)
    // Y: packet[3] (0x15 - Y Position Low Byte) + upper nibble of packet[4] (0x16 - Y11-Y8)
    uint16_t abs_x =
        packet[2] | ((packet[4] & 0x0F) << 8); // X = packet[2] + low 4 bits of packet[4]
    uint16_t abs_y =
        packet[3] | (((packet[4] & 0xF0) >> 4) << 8); // Y = packet[3] + high 4 bits of packet[4]

    // Extract Z-level from packet[5] (register 0x17)
    uint8_t z_level = packet[5] & 0x3F; // Z-level is 6-bit value

    if (data->in_int) {
        LOG_DBG("Clearing status bit");
        ret = pinnacle_clear_status(dev);
        data->in_int = true;
    }

    // Process tap detection using position and Z-level data
    pinnacle_process_tap_detection(dev, abs_x, abs_y, z_level);
    
    // Handle synthetic button events from tap detection
    if (!config->no_taps) {
        // Check if we have a pending tap to generate a button click
        if (data->tap_detector.tap_pending) {
            // Generate immediate button press
            input_report_key(dev, INPUT_BTN_0, 1, false, K_FOREVER);
            LOG_DBG("Synthetic button press generated");
            
            // Schedule button release after a short delay (10ms)
            k_work_schedule(&data->tap_release_work, K_MSEC(10));
            
            data->tap_detector.tap_pending = false;
        }
    }

    LOG_DBG("Absolute position: X=%d, Y=%d, Z=%d", abs_x, abs_y, z_level);

    // Validate coordinates are in expected range (12-bit: 0-4095)
    if (abs_x > 4095 || abs_y > 4095) {
        LOG_WRN("Invalid coordinates: X=%d, Y=%d - ignoring", abs_x, abs_y);
        return;
    }

    // Check for "no valid position" condition - trackpad reports (0,0) with Z=0 during finger lift
    bool position_invalid = (abs_x == 0 && abs_y == 0 && z_level == 0);
    if (position_invalid) {
        LOG_DBG("No valid position detected (0,0 + Z=0) - skipping position update");
        // Skip movement calculation and position updates, but still handle touch state for inertia
        goto handle_touch_detection;
    }

    // Check if this is the first touch after finger was lifted
    if (!data->touch_active && z_level >= config->z_threshold_touch) {
        LOG_DBG("First touch after lift at (%d,%d) - setting as new origin", abs_x, abs_y);
        // Set new origin without sending any movement
        data->last_x = abs_x;
        data->last_y = abs_y;
        data->position_valid = true;
        goto handle_touch_detection;  // Handle touch state but skip movement calculation
    }

    // Calculate relative movement deltas from absolute positions
    int16_t dx = 0, dy = 0;
    if (data->position_valid) {
        // Calculate deltas from previous position
        int32_t raw_dx = (int32_t)abs_x - (int32_t)data->last_x;
        int32_t raw_dy = (int32_t)abs_y - (int32_t)data->last_y;

        // Filter out large jumps that indicate position errors or palm detection
        const int32_t MAX_JUMP = 500; // Adjust based on testing
        if (abs(raw_dx) > MAX_JUMP || abs(raw_dy) > MAX_JUMP) {
            LOG_WRN("Large position jump detected: dx=%d, dy=%d - ignoring", (int)raw_dx,
                    (int)raw_dy);
            // Update position but don't report movement
            data->last_x = abs_x;
            data->last_y = abs_y;
            return;
        }

        // Use raw deltas without scaling for more responsive tracking
        dx = (int16_t)raw_dx;
        dy = (int16_t)raw_dy;

        // Add deadzone to prevent jitter from small movements
        const int16_t DEADZONE = 1;
        if (abs(dx) < DEADZONE)
            dx = 0;
        if (abs(dy) < DEADZONE)
            dy = 0;

        LOG_DBG("Raw delta: dx=%d, dy=%d, Scaled delta: dx=%d, dy=%d", (int)raw_dx, (int)raw_dy, dx,
                dy);
        LOG_DBG("Delta calculation: (%d,%d) - (%d,%d) = (%d,%d)",
                abs_x, abs_y, data->last_x, data->last_y, dx, dy);

        // Only report movement if there's actual movement
        if (dx != 0 || dy != 0) {
            input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, dy, false, K_FOREVER);

            // Update inertia velocities if inertia is enabled
            if (config->inertia_enable) {
                k_mutex_lock(&data->inertia_lock, K_FOREVER);

                int64_t current_time = k_uptime_get();

                // Update X velocity
                if (dx != 0) {
                    if (data->inertia_x.last_event_time > 0) {
                        int64_t time_diff_ms = current_time - data->inertia_x.last_event_time;
                        if (time_diff_ms <= 0) time_diff_ms = 1;

                        int32_t new_velocity = (dx * 1000) / time_diff_ms;
                        // Apply smoothing - weighted average with previous velocity
                        data->inertia_x.velocity_x1000 = (data->inertia_x.velocity_x1000 * 7 + new_velocity * 3) / 10;
                    }
                    data->inertia_x.last_event_time = current_time;
                }

                // Update Y velocity
                if (dy != 0) {
                    if (data->inertia_y.last_event_time > 0) {
                        int64_t time_diff_ms = current_time - data->inertia_y.last_event_time;
                        if (time_diff_ms <= 0) time_diff_ms = 1;

                        int32_t new_velocity = (dy * 1000) / time_diff_ms;
                        // Apply smoothing - weighted average with previous velocity
                        data->inertia_y.velocity_x1000 = (data->inertia_y.velocity_x1000 * 7 + new_velocity * 3) / 10;
                    }
                    data->inertia_y.last_event_time = current_time;
                }

                // Clear velocities for axes that haven't moved recently (>50ms)
                if (dx == 0 && data->inertia_x.last_event_time > 0 && (current_time - data->inertia_x.last_event_time) > 50) {
                    data->inertia_x.velocity_x1000 = 0;
                }
                if (dy == 0 && data->inertia_y.last_event_time > 0 && (current_time - data->inertia_y.last_event_time) > 50) {
                    data->inertia_y.velocity_x1000 = 0;
                }

                k_mutex_unlock(&data->inertia_lock);
            }
        }
    } else {
        // First valid position - no movement to report yet
        data->position_valid = true;
        LOG_DBG("First valid position recorded");
    }

    // Update previous position for next delta calculation
    data->last_x = abs_x;
    data->last_y = abs_y;

handle_touch_detection:
    // Z-level touch detection (if enabled)
    if (config->z_touch_detection) {
        LOG_DBG("Z-level: %d, touch_thresh: %d, release_thresh: %d", z_level,
                config->z_threshold_touch, config->z_threshold_release);

        // Determine touch state based on Z-level thresholds
        bool current_touch = z_level >= config->z_threshold_touch;
        bool release_touch = z_level < config->z_threshold_release;

        // Apply hysteresis to prevent jitter
        if (current_touch && !data->touch_active) {
            // Touch down - stop inertia immediately
            data->touch_active = true;
            LOG_DBG("Touch down (Z=%d)", z_level);

            if (config->inertia_enable) {
                k_mutex_lock(&data->inertia_lock, K_FOREVER);
                // Stop inertia and clear all state
                if (k_work_delayable_is_pending(&data->inertia_work)) {
                    k_work_cancel_delayable(&data->inertia_work);
                }
                data->inertia_x.active = false;
                data->inertia_y.active = false;
                data->inertia_x.remainder_x1000 = 0;
                data->inertia_y.remainder_x1000 = 0;
                LOG_DBG("Inertia stopped on touch down");
                k_mutex_unlock(&data->inertia_lock);
            }
        } else if (release_touch && data->touch_active) {
            // Touch up - potentially start inertia
            data->touch_active = false;
            LOG_DBG("Touch up (Z=%d)", z_level);

            if (config->inertia_enable) {
                k_mutex_lock(&data->inertia_lock, K_FOREVER);
                bool start_inertia = false;

                // Check if X velocity is sufficient for inertia
                int32_t abs_vel_x = data->inertia_x.velocity_x1000 < 0 ? -data->inertia_x.velocity_x1000 : data->inertia_x.velocity_x1000;
                if (abs_vel_x >= config->inertia_start_velocity) {
                    data->inertia_x.active = true;
                    start_inertia = true;
                    LOG_DBG("X axis inertia started with velocity %d", data->inertia_x.velocity_x1000);
                }

                // Check if Y velocity is sufficient for inertia
                int32_t abs_vel_y = data->inertia_y.velocity_x1000 < 0 ? -data->inertia_y.velocity_x1000 : data->inertia_y.velocity_x1000;
                if (abs_vel_y >= config->inertia_start_velocity) {
                    data->inertia_y.active = true;
                    start_inertia = true;
                    LOG_DBG("Y axis inertia started with velocity %d", data->inertia_y.velocity_x1000);
                }

                // Start inertia timer if any axis is active
                if (start_inertia && !k_work_delayable_is_pending(&data->inertia_work)) {
                    k_work_reschedule(&data->inertia_work, K_MSEC(config->inertia_update_interval_ms));
                    LOG_DBG("Inertia timer started");
                }

                k_mutex_unlock(&data->inertia_lock);
            }
        }
    }

    // Send sync event without touch events (touch state handled internally for inertia)
    input_report_rel(dev, INPUT_REL_X, 0, true, K_FOREVER);

    return;
}

static void pinnacle_work_cb(struct k_work *work) {
    struct pinnacle_data *data = CONTAINER_OF(work, struct pinnacle_data, work);
    pinnacle_report_data(data->dev);
}

static void pinnacle_inertia_work_cb(struct k_work *work) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    struct pinnacle_data *data = CONTAINER_OF(d_work, struct pinnacle_data, inertia_work);
    const struct pinnacle_config *config = data->dev->config;

    if (!config->inertia_enable) {
        return;
    }

    k_mutex_lock(&data->inertia_lock, K_FOREVER);

    bool any_active = false;
    int16_t synthetic_dx = 0, synthetic_dy = 0;

    // Process X axis
    if (data->inertia_x.active) {
        // Apply decay
        data->inertia_x.velocity_x1000 = (data->inertia_x.velocity_x1000 * config->inertia_decay_rate) / 1000;

        int32_t abs_velocity = data->inertia_x.velocity_x1000 < 0 ? -data->inertia_x.velocity_x1000 : data->inertia_x.velocity_x1000;

        if (abs_velocity < config->inertia_stop_velocity) {
            data->inertia_x.active = false;
            data->inertia_x.velocity_x1000 = 0;
            data->inertia_x.remainder_x1000 = 0;
        } else {
            // Calculate synthetic movement with remainder tracking
            int32_t movement_x1000 = (data->inertia_x.velocity_x1000 * config->inertia_update_interval_ms) + data->inertia_x.remainder_x1000;
            synthetic_dx = movement_x1000 / 1000;
            data->inertia_x.remainder_x1000 = movement_x1000 % 1000;
            any_active = true;
        }
    }

    // Process Y axis
    if (data->inertia_y.active) {
        // Apply decay
        data->inertia_y.velocity_x1000 = (data->inertia_y.velocity_x1000 * config->inertia_decay_rate) / 1000;

        int32_t abs_velocity = data->inertia_y.velocity_x1000 < 0 ? -data->inertia_y.velocity_x1000 : data->inertia_y.velocity_x1000;

        if (abs_velocity < config->inertia_stop_velocity) {
            data->inertia_y.active = false;
            data->inertia_y.velocity_x1000 = 0;
            data->inertia_y.remainder_x1000 = 0;
        } else {
            // Calculate synthetic movement with remainder tracking
            int32_t movement_x1000 = (data->inertia_y.velocity_x1000 * config->inertia_update_interval_ms) + data->inertia_y.remainder_x1000;
            synthetic_dy = movement_x1000 / 1000;
            data->inertia_y.remainder_x1000 = movement_x1000 % 1000;
            any_active = true;
        }
    }

    // Send synthetic movement events if we have any movement
    if (synthetic_dx != 0 || synthetic_dy != 0) {
        LOG_DBG("Inertia synthetic movement: dx=%d, dy=%d", synthetic_dx, synthetic_dy);

        if (synthetic_dx != 0) {
            input_report_rel(data->dev, INPUT_REL_X, synthetic_dx, false, K_FOREVER);
        }
        if (synthetic_dy != 0) {
            input_report_rel(data->dev, INPUT_REL_Y, synthetic_dy, false, K_FOREVER);
        }
        input_report_rel(data->dev, INPUT_REL_X, 0, true, K_FOREVER); // Send sync
    }

    // Continue inertia if any axis is still active
    if (any_active) {
        k_work_reschedule(&data->inertia_work, K_MSEC(config->inertia_update_interval_ms));
    }

    k_mutex_unlock(&data->inertia_lock);
}

static void pinnacle_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct pinnacle_data *data = CONTAINER_OF(cb, struct pinnacle_data, gpio_cb);

    LOG_DBG("HW DR asserted");
    data->in_int = true;
    k_work_submit(&data->work);
}

static int pinnacle_adc_sensitivity_reg_value(enum pinnacle_sensitivity sensitivity) {
    switch (sensitivity) {
    case PINNACLE_SENSITIVITY_1X:
        return PINNACLE_TRACKING_ADC_CONFIG_1X;
    case PINNACLE_SENSITIVITY_2X:
        return PINNACLE_TRACKING_ADC_CONFIG_2X;
    case PINNACLE_SENSITIVITY_3X:
        return PINNACLE_TRACKING_ADC_CONFIG_3X;
    case PINNACLE_SENSITIVITY_4X:
        return PINNACLE_TRACKING_ADC_CONFIG_4X;
    default:
        return PINNACLE_TRACKING_ADC_CONFIG_1X;
    }
}

static int pinnacle_tune_edge_sensitivity(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;
    int ret;

    uint8_t x_val;
    ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_X_AXIS_WIDE_Z_MIN, &x_val);
    if (ret < 0) {
        LOG_WRN("Failed to read X val: %d", ret);
        return ret;
    }

    LOG_WRN("X val: %d", x_val);

    uint8_t y_val;
    ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_Y_AXIS_WIDE_Z_MIN, &y_val);
    if (ret < 0) {
        LOG_WRN("Failed to read Y val: %d", ret);
        return ret;
    }

    LOG_WRN("Y val: %d", y_val);

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_X_AXIS_WIDE_Z_MIN, config->x_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set X-Axis Min-Z %d", ret);
        return ret;
    }
    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_Y_AXIS_WIDE_Z_MIN, config->y_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set Y-Axis Min-Z %d", ret);
        return ret;
    }
    return 0;
}

static int pinnacle_set_adc_tracking_sensitivity(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;

    uint8_t val;
    int ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    val &= 0x3F;
    val |= pinnacle_adc_sensitivity_reg_value(config->sensitivity);

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, val);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
    }
    ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    return ret;
}

static int pinnacle_force_recalibrate(const struct device *dev) {
    uint8_t val;
    int ret = pinnacle_seq_read(dev, PINNACLE_CAL_CFG, &val, 1);
    if (ret < 0) {
        LOG_ERR("Failed to get cal config %d", ret);
    }

    val |= 0x01;
    ret = pinnacle_write(dev, PINNACLE_CAL_CFG, val);
    if (ret < 0) {
        LOG_ERR("Failed to force calibration %d", ret);
    }

    do {
        pinnacle_seq_read(dev, PINNACLE_CAL_CFG, &val, 1);
    } while (val & 0x01);

    return ret;
}

int pinnacle_set_sleep(const struct device *dev, bool enabled) {
    uint8_t sys_cfg;
    int ret = pinnacle_seq_read(dev, PINNACLE_SYS_CFG, &sys_cfg, 1);
    if (ret < 0) {
        LOG_ERR("can't read sys config %d", ret);
        return ret;
    }

    if (((sys_cfg & PINNACLE_SYS_CFG_EN_SLEEP) != 0) == enabled) {
        return 0;
    }

    LOG_DBG("Setting sleep: %s", (enabled ? "on" : "off"));
    WRITE_BIT(sys_cfg, PINNACLE_SYS_CFG_EN_SLEEP_BIT, enabled ? 1 : 0);

    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, sys_cfg);
    if (ret < 0) {
        LOG_ERR("can't write sleep config %d", ret);
        return ret;
    }

    return ret;
}

static int pinnacle_init(const struct device *dev) {
    struct pinnacle_data *data = dev->data;
    const struct pinnacle_config *config = dev->config;
    int ret;

    uint8_t fw_id[2];
    ret = pinnacle_seq_read(dev, PINNACLE_FW_ID, fw_id, 2);
    if (ret < 0) {
        LOG_ERR("Failed to get the FW ID %d", ret);
    }

    LOG_DBG("Found device with FW ID: 0x%02x, Version: 0x%02x", fw_id[0], fw_id[1]);

    LOG_INF("Z-touch detection: %s, thresholds: touch=%d, release=%d",
            config->z_touch_detection ? "enabled" : "disabled", config->z_threshold_touch,
            config->z_threshold_release);

    data->in_int = false;
    data->touch_active = false;
    data->position_valid = false;
    data->last_x = 0;
    data->last_y = 0;

    // Initialize inertia system
    if (config->inertia_enable) {
        k_mutex_init(&data->inertia_lock);
        k_work_init_delayable(&data->inertia_work, pinnacle_inertia_work_cb);

        // Initialize inertia axis states
        data->inertia_x.velocity_x1000 = 0;
        data->inertia_x.last_event_time = 0;
        data->inertia_x.active = false;
        data->inertia_x.remainder_x1000 = 0;

        data->inertia_y.velocity_x1000 = 0;
        data->inertia_y.last_event_time = 0;
        data->inertia_y.active = false;
        data->inertia_y.remainder_x1000 = 0;

        LOG_DBG("Inertia system initialized: start_vel=%d, stop_vel=%d, decay=%d, interval=%dms",
                config->inertia_start_velocity, config->inertia_stop_velocity,
                config->inertia_decay_rate, config->inertia_update_interval_ms);
    }
    
    // Initialize tap detection system
    if (!config->no_taps) {
        data->tap_detector.state = TAP_IDLE;
        data->tap_detector.touch_start_time = 0;
        data->tap_detector.initial_x = 0;
        data->tap_detector.initial_y = 0;
        data->tap_detector.max_movement = 0;
        data->tap_detector.tap_pending = false;
        
        // Initialize tap release work
        k_work_init_delayable(&data->tap_release_work, pinnacle_tap_release_work_handler);
        
        LOG_INF("Tap detection enabled: tap_timeout=%dms, movement_threshold=%d",
                config->tap_timeout_ms, config->tap_movement_threshold);
    }
    k_msleep(10);
    ret = pinnacle_write(dev, PINNACLE_STATUS1, 0); // Clear CC
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    k_usleep(50);
    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, PINNACLE_SYS_CFG_RESET);
    if (ret < 0) {
        LOG_ERR("can't reset %d", ret);
        return ret;
    }
    k_msleep(20);
    ret = pinnacle_write(dev, PINNACLE_Z_IDLE, 0x05); // No Z-Idle packets
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    ret = pinnacle_set_adc_tracking_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
        return ret;
    }

    ret = pinnacle_tune_edge_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to tune edge sensitivity %d", ret);
        return ret;
    }
    ret = pinnacle_force_recalibrate(dev);
    if (ret < 0) {
        LOG_ERR("Failed to force recalibration %d", ret);
        return ret;
    }

    if (config->sleep_en) {
        ret = pinnacle_set_sleep(dev, true);
        if (ret < 0) {
            return ret;
        }
    }

    uint8_t packet[1];
    ret = pinnacle_seq_read(dev, PINNACLE_SLEEP_INTERVAL, packet, 1);

    if (ret >= 0) {
        LOG_DBG("Default sleep interval %d", packet[0]);
    }

    ret = pinnacle_write(dev, PINNACLE_SLEEP_INTERVAL, 255);
    if (ret <= 0) {
        LOG_DBG("Failed to update sleep interaval %d", ret);
    }

    uint8_t feed_cfg2 = PINNACLE_FEED_CFG2_EN_IM | PINNACLE_FEED_CFG2_EN_BTN_SCRL;

    if (config->rotate_90) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_ROTATE_90;
    }
    ret = pinnacle_write(dev, PINNACLE_FEED_CFG2, feed_cfg2);
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    uint8_t feed_cfg1 =
        PINNACLE_FEED_CFG1_EN_FEED | PINNACLE_FEED_CFG1_ABS_MODE; // Enable absolute mode
    if (config->x_invert) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_INV_X;
    }

    if (config->y_invert) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_INV_Y;
    }
    if (feed_cfg1) {
        ret = pinnacle_write(dev, PINNACLE_FEED_CFG1, feed_cfg1);
    }
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    data->dev = dev;

    pinnacle_clear_status(dev);

    gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
    gpio_init_callback(&data->gpio_cb, pinnacle_gpio_cb, BIT(config->dr.pin));
    ret = gpio_add_callback(config->dr.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    k_work_init(&data->work, pinnacle_work_cb);

    pinnacle_write(dev, PINNACLE_FEED_CFG1, feed_cfg1);

    set_int(dev, true);

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int pinnacle_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        return set_int(dev, false);
    case PM_DEVICE_ACTION_RESUME:
        return set_int(dev, true);
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

#define PINNACLE_INST(n)                                                                           \
    static struct pinnacle_data pinnacle_data_##n;                                                 \
    static const struct pinnacle_config pinnacle_config_##n = {                                    \
        COND_CODE_1(DT_INST_ON_BUS(n, i2c),                                                        \
                    (.bus = {.i2c = I2C_DT_SPEC_INST_GET(n)}, .seq_read = pinnacle_i2c_seq_read,   \
                     .write = pinnacle_i2c_write),                                                 \
                    (.bus = {.spi = SPI_DT_SPEC_INST_GET(n,                                        \
                                                         SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |    \
                                                             SPI_TRANSFER_MSB | SPI_MODE_CPHA,     \
                                                         0)},                                      \
                     .seq_read = pinnacle_spi_seq_read, .write = pinnacle_spi_write)),             \
        .rotate_90 = DT_INST_PROP(n, rotate_90),                                                   \
        .x_invert = DT_INST_PROP(n, x_invert),                                                     \
        .y_invert = DT_INST_PROP(n, y_invert),                                                     \
        .sleep_en = DT_INST_PROP(n, sleep),                                                        \
        .z_touch_detection = DT_INST_PROP_OR(n, z_touch_detection, false),                         \
        .x_axis_z_min = DT_INST_PROP_OR(n, x_axis_z_min, 5),                                       \
        .y_axis_z_min = DT_INST_PROP_OR(n, y_axis_z_min, 4),                                       \
        .z_threshold_touch = DT_INST_PROP_OR(n, z_threshold_touch, 5),                             \
        .z_threshold_release = DT_INST_PROP_OR(n, z_threshold_release, 2),                         \
        .sensitivity = DT_INST_ENUM_IDX_OR(n, sensitivity, PINNACLE_SENSITIVITY_1X),               \
        .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), dr_gpios, {}),                                   \
        .inertia_enable = DT_INST_PROP_OR(n, inertia, false),                                     \
        .inertia_start_velocity = DT_INST_PROP_OR(n, inertia_start_velocity, 15),                  \
        .inertia_stop_velocity = DT_INST_PROP_OR(n, inertia_stop_velocity, 5),                     \
        .inertia_decay_rate = DT_INST_PROP_OR(n, inertia_decay_rate, 950),                         \
        .inertia_update_interval_ms = DT_INST_PROP_OR(n, inertia_update_interval_ms, 16),          \
        .no_taps = DT_INST_PROP(n, no_taps),                                       \
        .tap_timeout_ms = DT_INST_PROP_OR(n, tap_timeout_ms, 200),                                 \
        .tap_movement_threshold = DT_INST_PROP_OR(n, tap_movement_threshold, 15),                  \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(n, pinnacle_pm_action);                                               \
    DEVICE_DT_INST_DEFINE(n, pinnacle_init, PM_DEVICE_DT_INST_GET(n), &pinnacle_data_##n,          \
                          &pinnacle_config_##n, POST_KERNEL, CONFIG_INPUT_PINNACLE_INIT_PRIORITY,  \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(PINNACLE_INST)
