//
// MT6835 21-bit magnetic encoder driver.
//


#ifndef LEO_FOC_MT6835_H
#define LEO_FOC_MT6835_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

/* MT6835 24-bit command field. Arm GNU GCC accepts 0b... constants. */
#define MT6835_OP_READ                 0b0011U
#define MT6835_OP_WRITE                0b0110U
#define MT6835_OP_PROGRAM_EEPROM       0b1100U
#define MT6835_OP_SET_ZERO             0b0101U
#define MT6835_OP_BURST_ANGLE          0b1010U

#define MT6835_REG_ANGLE1              0x003U
#define MT6835_REG_ANGLE2              0x004U
#define MT6835_REG_ANGLE3              0x005U
#define MT6835_REG_ANGLE_CRC           0x006U

#define MT6835_COUNTS_PER_REVOLUTION   (1UL << 21U)
#define MT6835_MAX_SPI_CLOCK_HZ        16000000UL
#define MT6835_DEFAULT_VELOCITY_FILTER_HZ 200.0f
#define MotorRadius                    0.025f /* m */

#define MT6835_STATUS_OVERSPEED        (1U << 0U)
#define MT6835_STATUS_WEAK_FIELD       (1U << 1U)
#define MT6835_STATUS_UNDERVOLTAGE     (1U << 2U)

/* MT6835_Reading.config_error bits. */
#define MT6835_CONFIG_ERROR_SPI        (1UL << 0U)
#define MT6835_CONFIG_ERROR_DMA        (1UL << 1U)
#define MT6835_CONFIG_ERROR_CLOCK      (1UL << 2U)

typedef struct {
    uint32_t raw_angle;             /* 0 .. 2^21 - 1. */
    float angle_radians;            /* 0 .. 2*pi. */
    uint32_t sample_count;          /* Completed 48-bit angle frames. */
    uint32_t crc_error_count;
    uint32_t transfer_error_count;
    uint32_t config_error;
    uint8_t status;                 /* MT6835_STATUS_* bits. */
    uint8_t received_crc;
    uint8_t calculated_crc;
    uint8_t valid;                  /* One when the latest frame passed CRC. */
} MT6835_Reading_t;

typedef struct {
    float radians_per_second;       /* Positive when raw_angle increases. */
    float revolutions_per_minute;
    uint32_t sample_count;          /* Angle frame used for this estimate. */
    uint8_t valid;                  /* Zero until two valid frames exist. */
} MT6835_Velocity_t;

/* Updated by the SPI1 RX-DMA completion interrupt. */
extern volatile MT6835_Reading_t MT6835_Reading;

/* Call once after MX_GPIO_Init(), MX_DMA_Init(), and MX_SPI1_Init(). */
HAL_StatusTypeDef MT6835_Init(void);

/*
 * Starts one non-blocking 48-bit burst-angle transaction.
 * The result is published in MT6835_Reading from the DMA callback.
 */
HAL_StatusTypeDef MT6835_StartAngleRead_DMA(void);

/* Stops any active MT6835 DMA transaction and releases SPI1. */
void MT6835_DeInit(void);

bool MT6835_IsBusy(void);

/* Copies a coherent snapshot; returns true when its CRC is valid. */
bool MT6835_GetLatestReading(MT6835_Reading_t *destination);

/*
 * Copies the latest filtered velocity estimate. The estimate is updated after
 * every valid angle DMA frame and is positive when raw_angle increases.
 */
bool MT6835_GetLatestVelocity(MT6835_Velocity_t *destination);

/* Convenience accessors for the filtered velocity estimate. */
bool MT6835_GetVelocityRadPerSecond(float *radians_per_second);
bool MT6835_GetVelocityRPM(float *revolutions_per_minute);

/*
 * Returns the signed multi-turn position accumulated from valid angle frames.
 * The first valid frame defines zero and one revolution is exactly
 * MT6835_COUNTS_PER_REVOLUTION counts. Positive counts follow increasing
 * raw_angle. The 64-bit value is copied atomically with respect to the DMA ISR.
 */
bool MT6835_GetTotalAngleCounts(int64_t *total_angle_counts);

/* Returns signed mechanical revolutions relative to the position zero. */
bool MT6835_GetTotalRevolutions(float *total_revolutions);

/*
 * Returns signed direct-drive travel distance in meters using
 * MotorRadius.
 */
bool MT6835_GetDistanceMeters(float *distance_meters);

/* Zeros the accumulated position at the latest accepted angle sample. */
void MT6835_ResetTotalAngleCounts(void);

/*
 * Sets the first-order velocity low-pass cutoff. The default is 200 Hz.
 * Passing 0 disables filtering and publishes the differentiated raw angle.
 */
void MT6835_SetVelocityFilterCutoff(float cutoff_hz);

/* Invalidates velocity until two new valid angle frames have been received. */
void MT6835_ResetVelocityEstimator(void);

/* CRC-8: polynomial x^8 + x^2 + x + 1 (0x07), initial value zero. */
uint8_t MT6835_CalculateCRC(uint32_t raw_angle, uint8_t status);

#ifdef __cplusplus
}
#endif

#endif // LEO_FOC_MT6835_H
