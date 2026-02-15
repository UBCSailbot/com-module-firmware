#ifndef CANSERVO_H
#define CANSERVO_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @file CANSERVO.h
 * @brief Public API + register/encoding definitions for CAN-controlled servo.
 *
 * Notes:
 * - Many functions accept float but ultimately send 16-bit values over CAN.
 * - Register addresses, ranges, and units should be validated against the datasheet.
 * - REG_POWER_CONFIG is treated as write-only in your design (forced ES mode + SW reset).
 */

// ============================================================================
// Register definitions (servo register map)
// ============================================================================

/** @name Position / limits */
//@{
#define REG_POSITION_NEW        0x1E  /**< Command / position reference (raw position encoding). */
#define REG_POSITION_MID        0xC2  /**< Midpoint register (marked "does not work as intended" in notes). */

#define REG_POSITION_MAX_LIMIT  0xB0  /**< Soft max position limit (0..16383, 4096=90deg per notes). */
#define REG_POSITION_MIN_LIMIT  0xB2  /**< Soft min position limit (0..16383, 4096=90deg per notes). */

#define REG_POS_MAX             0x50  /**< Hard max limit (may trigger emergency stop mode). */
#define REG_POS_MIN             0x52  /**< Hard min limit (may trigger emergency stop mode). */
//@}

/** @name Velocity / acceleration */
//@{
#define REG_VELOCITY_MAX        0x54  /**< Max velocity in normal state (notes: 0..4095). */
#define REG_SPEED_UP            0xDC  /**< Acceleration ramp time (ms per notes). */
#define REG_SPEED_DN            0xDE  /**< Deceleration ramp time (ms per notes). */
#define REG_SPEED_ES            0xE0  /**< Emergency-mode speed (datasheet-specific; you use forced ES mode via power config). */
//@}

/** @name Temperature */
//@{
#define REG_TEMP                0xD2  /**< Temperature readback (raw -> degC conversion in CANSERVO.c). */
#define REG_TEMPER_MAX          0x5C  /**< Max temperature limit (triggers emergency mode). */
#define REG_TEMPER_MIN          0x6C  /**< Min temperature limit (triggers emergency mode). */
//@}

/** @name Current / voltage */
//@{
#define REG_CURRENT             0x16  /**< Current readback (units depend on datasheet). */
#define REG_CURRENT_MAX         0xD8  /**< Max current limit (mA per notes). */

#define REG_VOLTAGE_MAX         0x58  /**< Max voltage limit (units depend on datasheet). */
#define REG_VOLTAGE_MIN         0x5A  /**< Min voltage limit (units depend on datasheet). */
//@}

/** @name Status / control */
//@{
#define REG_EMERGENCY_STOP      0x48  /**< Emergency status flags (bitmask). */
#define REG_POWER_CONFIG        0x46  /**< Power config (forced ES mode + SW reset bit). */
//@}

// ============================================================================
// Servo protocol / addressing definitions
// ============================================================================

/**
 * @name CAN protocol fields
 * These constants define how messages are built for your servo protocol.
 */
//@{
#define SERVO_CAN_ID        0x000  /**< CAN frame ID used for the servo (extended ID in your code). */
#define SERVO_ID            0      /**< Servo node/device ID placed in data1. */

#define SERVO_WRITE_DLC     7      /**< DLC used for register write frames. */
#define SERVO_READ_DLC      5      /**< DLC used for register read frames. */

#define SERVO_REG_LENGTH    2      /**< Register length in bytes (16-bit registers). */
//@}

// ============================================================================
// Servo configuration definitions (project-level defaults)
// ============================================================================

/**
 * Soft servo limits for -90deg to +90deg behavior (per your notes).
 * Encoding note from your code:
 * - 0x2000 is "center"
 * - 4096 counts corresponds to 90 degrees
 */
#define SERVO_MAX_POSITION_LIMIT     12288   /**< Soft max limit (check against datasheet). */
#define SERVO_MIN_POSITION_LIMIT     4096       /**< Soft min limit (check against datasheet). */
#define SERVO_INITIAL_MIDPOINT		 8192
extern uint16_t servo_midpoint;

/**
 * Hard servo limits used if the servo goes beyond soft range.
 * @warning Comments indicate hard limits may trigger emergency-stop mode and be hard to recover from.
 */
#define SERVO_MAX_POSITION           12288+0.1*4096  /**< +10% error margin around +90deg region (float expression). */
#define SERVO_MIN_POSITION           4096-0.1*4096   /**< -10% error margin around -90deg region (float expression). */

#define SERVO_MAX_CURRENT            6800            /**< Max stall current (mA per notes). */
#define SERVO_MAX_VOLTAGE            1200+1200*0.01  /**< 12V +1% (units depend on datasheet encoding). */
#define SERVO_MIN_VOLTAGE            1200-1200*0.01  /**< 12V -1% (units depend on datasheet encoding). */

// ============================================================================
// Encoding helper macros
// ============================================================================

/**
 * @brief Convert degrees to raw 16-bit servo position encoding.
 * @param angle_deg Angle in degrees.
 * @return Raw position (uint16_t) using: raw = angle*(4096/90) + 0x2000.
 */
#define ANGLE_TO_SERVO_U16(angle_deg) \
    ((uint16_t)((angle_deg) * (4096.0f / 90.0f) + 0x2000))

/**
 * @brief Convert raw 16-bit servo position encoding to degrees.
 * @param servo_u16 Raw position encoding.
 * @return Angle in degrees using: deg = (raw-0x2000)*(90/4096).
 */
#define SERVO_U16_TO_ANGLE(servo_u16) \
    (((float)(servo_u16) - 0x2000f) * (90.0f / 4096.0f))

// ============================================================================
// Emergency status flags (REG_EMERGENCY_STOP bitmasks)
// ============================================================================

/**
 * @name Emergency status bits
 * These match how SERVO_OK() tests servo_status in your CANSERVO.c.
 */
//@{
#define ERR_POS_MIN      (1 << 8)   /**< Position min limit exceeded. */
#define ERR_POS_MAX      (1 << 9)   /**< Position max limit exceeded. */
#define ERR_MCU_TEMP_UND (1 << 10)  /**< MCU temperature too low. */
#define ERR_MCU_TEMP_OVR (1 << 11)  /**< MCU temperature too high. */
#define ERR_VOLT_UND     (1 << 13)  /**< Voltage too low. */
#define ERR_VOLT_OVR     (1 << 14)  /**< Voltage too high. */
//@}

/** @deprecated Prefer PWR_SW_RESET_BIT for REG_POWER_CONFIG SW reset. */
#define POWER_CFG_SW_RESET (1U << 0)

// ============================================================================
// Power config modes (REG_POWER_CONFIG)
// ============================================================================

/**
 * @name Forced emergency-stop (forced ES) field
 * Typically bits [10:9] based on your earlier notes/screenshot.
 *
 * 00 = OFF
 * 01 = Motor_Free
 * 10 = Speed_Down
 * 11 = Motor_Hold
 */
//@{
#define PWR_FORCED_ES_SHIFT  9
#define PWR_FORCED_ES_MASK   (0x3u << PWR_FORCED_ES_SHIFT)

#define PWR_FORCED_ES_OFF         (0u << PWR_FORCED_ES_SHIFT)
#define PWR_FORCED_ES_MOTOR_FREE  (1u << PWR_FORCED_ES_SHIFT)
#define PWR_FORCED_ES_SPEED_DOWN  (2u << PWR_FORCED_ES_SHIFT)  /**< Speed_Down (0x0400). */
#define PWR_FORCED_ES_MOTOR_HOLD  (3u << PWR_FORCED_ES_SHIFT)
//@}

/** @brief Software reset request bit written via REG_POWER_CONFIG. */
#define PWR_SW_RESET_BIT          (1u << 0)  /**< 0x0001 */

// ============================================================================
// Timer / step token used by your Servo_Handler throttling
// ============================================================================

/**
 * @brief Step token set by a periodic timer ISR to rate-limit servo recovery actions.
 *
 * Typical pattern:
 * - ISR sets g_servo_step = 1 when an action is allowed.
 * - Servo handler consumes token (sets back to 0).
 */
extern volatile uint8_t g_servo_step;

// ============================================================================
// Function prototypes
// ============================================================================

/**
 * @name CAN helpers
 * Low-level CAN message construction and register read/write primitives.
 */
//@{

/**
 * @brief Calculate write checksum used in send_can_msg().
 * @param ID         Servo device ID.
 * @param address    Register address.
 * @param reg_length Register length in bytes (typically 2).
 * @param lo_byte    Payload low byte.
 * @param hi_byte    Payload high byte.
 * @return 8-bit checksum.
 */
uint8_t calculate_write_checksum(uint8_t ID, uint8_t address, uint8_t reg_length,
                                 uint8_t lo_byte, uint8_t hi_byte);

/**
 * @brief Calculate read checksum used in receive_can_msg().
 * @param ID         Servo device ID.
 * @param address    Register address.
 * @param reg_length Register length in bytes (typically 2).
 * @return 8-bit checksum.
 */
uint8_t calculate_read_checksum(uint8_t ID, uint8_t address, uint8_t reg_length);

/**
 * @brief Write a 16-bit value to a servo register over CAN.
 * @param val 16-bit raw value to write.
 * @param reg Register address.
 */
void send_can_msg(uint16_t val, uint8_t reg);

/**
 * @brief Read a 16-bit value from a servo register over CAN.
 * @param reg Register address.
 * @return 16-bit raw register value.
 */
uint16_t receive_can_msg(uint8_t reg);

//@}

// ----------------------------------------------------------------------------

/**
 * @name Servo high-level functionality
 * Wrapper functions that convert application-level units/ideas into register writes/reads.
 */
//@{

/**
 * @brief Initialize servo configuration at startup using project macros.
 * Writes soft position limits and max current limit.
 */
void servo_init(void);

/**
 * @brief Command a servo angle.
 * @param angle Angle in degrees (notes suggest -180..+180; no clamp is applied in implementation).
 */
void set_servo_angle(float angle);

/**
 * @brief Read back servo angle.
 * @return Angle in degrees using SERVO_U16_TO_ANGLE conversion.
 */
float check_servo_angle(void);

/**
 * @brief Set servo midpoint (marked as not working as intended in your notes).
 * @param servo_midpoint Raw midpoint value (encoding per datasheet/firmware).
 */
void set_servo_midpoint(void);

/**
 * @brief Read servo midpoint (marked DO NOT USE in your notes).
 * @return Midpoint converted using SERVO_U16_TO_ANGLE.
 */
float check_servo_midpoint(void);

/**
 * @brief Set acceleration ramp-up time.
 * @param servo_speed_up_time Time in ms (sent as raw 16-bit value).
 */
void set_servo_speed_up(float servo_speed_up_time);

/**
 * @brief Set deceleration ramp-down time.
 * @param servo_speed_down_time Time in ms (sent as raw 16-bit value).
 */
void set_servo_speed_down(float servo_speed_down_time);

/**
 * @brief Set maximum velocity in normal mode.
 * @param servo_max_velocity Raw value (notes suggest 0..4095; no clamp is applied).
 */
void set_servo_max_velocity(float servo_max_velocity);

/**
 * @brief Read servo internal temperature.
 * @return Temperature in degrees C using conversion in implementation.
 */
float check_servo_temp(void);

/**
 * @brief Set maximum temperature limit (triggers emergency mode if exceeded).
 * @param servo_max_temperature Raw limit value (encoding per datasheet).
 */
void set_servo_max_temperature(float servo_max_temperature);

/**
 * @brief Set minimum temperature limit (triggers emergency mode if undercut).
 * @param servo_min_temperature Raw limit value (encoding per datasheet).
 */
void set_servo_min_temperature(float servo_min_temperature);

/**
 * @brief Set maximum current limit.
 * @param max_servo_current Current limit (mA per notes; encoding per datasheet).
 */
void set_servo_max_current(float max_servo_current);

/**
 * @brief Read current draw.
 * @return Raw current reading (units per datasheet).
 */
float check_servo_current(void);

/**
 * @brief Read configured max current limit.
 * @return Raw max current limit (units per datasheet).
 */
float check_servo_max_current(void);

/**
 * @brief Set maximum voltage limit.
 * @param servo_max_voltage Raw max voltage value (units per datasheet).
 */
void set_servo_max_voltage(float servo_max_voltage);

/**
 * @brief Set minimum voltage limit.
 * @param servo_min_voltage Raw min voltage value (units per datasheet).
 */
void set_servo_min_voltage(float servo_min_voltage);

/**
 * @brief Set soft maximum position limit.
 * @param servo_position_max_limit Raw position limit value (encoding per datasheet).
 */
void set_servo_max_position_limit(float servo_position_max_limit);

/**
 * @brief Set soft minimum position limit.
 * @param servo_position_min_limit Raw position limit value (encoding per datasheet).
 */
void set_servo_min_position_limit(float servo_position_min_limit);

/**
 * @brief Set hard maximum position limit (may trigger emergency stop; use with caution).
 * @param servo_position_max Raw hard limit value (encoding per datasheet).
 */
void set_servo_max_position(float servo_position_max);

/**
 * @brief Set hard minimum position limit (may trigger emergency stop; use with caution).
 * @param servo_position_min Raw hard limit value (encoding per datasheet).
 */
void set_servo_min_position(float servo_position_min);

/**
 * @brief Check if servo has no emergency status flags set (based on REG_EMERGENCY_STOP).
 * @return true if no checked error bits are set, false otherwise.
 */
bool SERVO_OK(void);

/**
 * @brief Attempt recovery from a detected servo fault.
 *
 * Performs forced speed-down, software reset, clears emergency mode,
 * and reapplies initial configuration.
 */
void servo_error_handler (void);

/**
 * @brief Force servo into "Speed_Down" mode via REG_POWER_CONFIG forced-ES field.
 * @note Writes PWR_FORCED_ES_SPEED_DOWN to REG_POWER_CONFIG.
 */
void servo_set_speed_down_emergeny_mode(void);

/**
 * @brief Clear forced-ES mode via REG_POWER_CONFIG (forced-ES field set to OFF).
 * @note Writes PWR_FORCED_ES_OFF to REG_POWER_CONFIG.
 */
void servo_clear_forced_es(void);

/**
 * @brief Request a servo software reset via REG_POWER_CONFIG.
 * @note Writes PWR_SW_RESET_BIT to REG_POWER_CONFIG.
 */
void servo_soft_reset_only(void);

//@}

#endif
