//
// MT6835 21-bit magnetic encoder driver.
//


#include "mt6835.h"

#include "main.h"
#include "spi.h"

#define MT6835_ANGLE_FRAME_SIZE        6U
#define MT6835_SPI_IDLE_TIMEOUT        1000U
#define MT6835_CS_DELAY_ITERATIONS     32U
#define MT6835_TWO_PI                  6.28318530717958647692f
#define MT6835_RPM_PER_RADIAN_SECOND   9.54929658551372014613f
#define MT6835_MAX_VELOCITY_GAP_MS     100U

volatile MT6835_Reading_t MT6835_Reading = {0};

static volatile MT6835_Velocity_t MT6835_Velocity = {0};

static uint8_t MT6835_TxBuffer[MT6835_ANGLE_FRAME_SIZE] = {
    (uint8_t)(MT6835_OP_BURST_ANGLE << 4U),
    (uint8_t)MT6835_REG_ANGLE1,
    0U,
    0U,
    0U,
    0U
};
static uint8_t MT6835_RxBuffer[MT6835_ANGLE_FRAME_SIZE] = {0};

static volatile uint8_t MT6835_Initialized = 0U;
static volatile uint8_t MT6835_TransferBusy = 0U;
static volatile float MT6835_VelocityFilterCutoffHz =
    MT6835_DEFAULT_VELOCITY_FILTER_HZ;

static uint32_t MT6835_PreviousVelocityAngle = 0U;
static uint32_t MT6835_PreviousVelocityCycles = 0U;
static uint32_t MT6835_PreviousVelocityTickMs = 0U;
static volatile uint32_t MT6835_CurrentSampleCycles = 0U;
static volatile uint32_t MT6835_CurrentSampleTickMs = 0U;
static uint32_t MT6835_CycleClockHz = 0U;
static uint8_t MT6835_HavePreviousVelocityAngle = 0U;
static uint8_t MT6835_VelocityFilterInitialized = 0U;

static void MT6835_RxDMAComplete(DMA_HandleTypeDef *hdma);
static void MT6835_DMAError(DMA_HandleTypeDef *hdma);
static void MT6835_ResetVelocityState(void);
static void MT6835_UpdateVelocity(uint32_t raw_angle,
                                  uint32_t sample_count,
                                  uint32_t sample_cycles,
                                  uint32_t sample_tick_ms);

static void MT6835_ResetVelocityState(void)
{
    MT6835_Velocity.radians_per_second = 0.0f;
    MT6835_Velocity.revolutions_per_minute = 0.0f;
    MT6835_Velocity.sample_count = 0U;
    MT6835_Velocity.valid = 0U;

    MT6835_PreviousVelocityAngle = 0U;
    MT6835_PreviousVelocityCycles = 0U;
    MT6835_PreviousVelocityTickMs = 0U;
    MT6835_CurrentSampleCycles = 0U;
    MT6835_CurrentSampleTickMs = 0U;
    MT6835_HavePreviousVelocityAngle = 0U;
    MT6835_VelocityFilterInitialized = 0U;
}

static void MT6835_UpdateVelocity(uint32_t raw_angle,
                                  uint32_t sample_count,
                                  uint32_t sample_cycles,
                                  uint32_t sample_tick_ms)
{
    if ((MT6835_HavePreviousVelocityAngle == 0U) ||
        (MT6835_CycleClockHz == 0U)) {
        MT6835_PreviousVelocityAngle = raw_angle;
        MT6835_PreviousVelocityCycles = sample_cycles;
        MT6835_PreviousVelocityTickMs = sample_tick_ms;
        MT6835_HavePreviousVelocityAngle = 1U;
        MT6835_Velocity.valid = 0U;
        return;
    }

    const uint32_t previous_angle = MT6835_PreviousVelocityAngle;
    const uint32_t elapsed_cycles =
        sample_cycles - MT6835_PreviousVelocityCycles;
    const uint32_t elapsed_ms =
        sample_tick_ms - MT6835_PreviousVelocityTickMs;

    MT6835_PreviousVelocityAngle = raw_angle;
    MT6835_PreviousVelocityCycles = sample_cycles;
    MT6835_PreviousVelocityTickMs = sample_tick_ms;

    if ((elapsed_cycles == 0U) ||
        (elapsed_ms > MT6835_MAX_VELOCITY_GAP_MS)) {
        MT6835_Velocity.valid = 0U;
        MT6835_VelocityFilterInitialized = 0U;
        return;
    }

    int32_t delta_counts =
        (int32_t)raw_angle - (int32_t)previous_angle;

    if (delta_counts > (int32_t)(MT6835_COUNTS_PER_REVOLUTION / 2U)) {
        delta_counts -= (int32_t)MT6835_COUNTS_PER_REVOLUTION;
    } else if (delta_counts <
               -(int32_t)(MT6835_COUNTS_PER_REVOLUTION / 2U)) {
        delta_counts += (int32_t)MT6835_COUNTS_PER_REVOLUTION;
    }

    const float elapsed_seconds =
        (float)elapsed_cycles / (float)MT6835_CycleClockHz;
    const float instantaneous_radians_per_second =
        ((float)delta_counts *
         (MT6835_TWO_PI / (float)MT6835_COUNTS_PER_REVOLUTION)) /
        elapsed_seconds;
    float filtered_radians_per_second =
        instantaneous_radians_per_second;
    const float cutoff_hz = MT6835_VelocityFilterCutoffHz;

    if ((cutoff_hz > 0.0f) &&
        (MT6835_VelocityFilterInitialized != 0U)) {
        const float rc_seconds = 1.0f / (MT6835_TWO_PI * cutoff_hz);
        const float alpha = elapsed_seconds / (rc_seconds + elapsed_seconds);

        filtered_radians_per_second =
            MT6835_Velocity.radians_per_second +
            alpha * (instantaneous_radians_per_second -
                     MT6835_Velocity.radians_per_second);
    }

    MT6835_Velocity.valid = 0U;
    MT6835_Velocity.radians_per_second = filtered_radians_per_second;
    MT6835_Velocity.revolutions_per_minute =
        filtered_radians_per_second * MT6835_RPM_PER_RADIAN_SECOND;
    MT6835_Velocity.sample_count = sample_count;
    MT6835_VelocityFilterInitialized = 1U;
    __DMB();
    MT6835_Velocity.valid = 1U;
}

static void MT6835_CSDelay(void)
{
    for (volatile uint32_t i = 0U; i < MT6835_CS_DELAY_ITERATIONS; ++i) {
        __NOP();
    }
}

static uint32_t MT6835_SPIClockDivider(void)
{
    const uint32_t divider_index =
        (hspi1.Init.BaudRatePrescaler & SPI_CR1_BR_Msk) >> SPI_CR1_BR_Pos;

    return 2UL << divider_index;
}

static uint32_t MT6835_ValidateConfiguration(void)
{
    uint32_t error = 0U;

    if ((hspi1.Instance != SPI1) ||
        (hspi1.Init.Mode != SPI_MODE_MASTER) ||
        (hspi1.Init.Direction != SPI_DIRECTION_2LINES) ||
        (hspi1.Init.DataSize != SPI_DATASIZE_8BIT) ||
        (hspi1.Init.CLKPolarity != SPI_POLARITY_HIGH) ||
        (hspi1.Init.CLKPhase != SPI_PHASE_2EDGE) ||
        (hspi1.Init.NSS != SPI_NSS_SOFT) ||
        (hspi1.Init.FirstBit != SPI_FIRSTBIT_MSB)) {
        error |= MT6835_CONFIG_ERROR_SPI;
    }

    if ((hspi1.hdmatx == NULL) || (hspi1.hdmarx == NULL)) {
        error |= MT6835_CONFIG_ERROR_DMA;
    } else {
        if ((hspi1.hdmatx->Init.Mode != DMA_NORMAL) ||
            (hspi1.hdmatx->Init.Direction != DMA_MEMORY_TO_PERIPH) ||
            (hspi1.hdmatx->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE) ||
            (hspi1.hdmatx->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE) ||
            (hspi1.hdmatx->Init.MemInc != DMA_MINC_ENABLE) ||
            (hspi1.hdmarx->Init.Direction != DMA_PERIPH_TO_MEMORY) ||
            (hspi1.hdmarx->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE) ||
            (hspi1.hdmarx->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE) ||
            (hspi1.hdmarx->Init.MemInc != DMA_MINC_ENABLE) ||
            ((hspi1.hdmarx->Init.Mode != DMA_CIRCULAR) &&
             (hspi1.hdmarx->Init.Mode != DMA_NORMAL))) {
            error |= MT6835_CONFIG_ERROR_DMA;
        }
    }

    if ((HAL_RCC_GetPCLK2Freq() / MT6835_SPIClockDivider()) >
        MT6835_MAX_SPI_CLOCK_HZ) {
        error |= MT6835_CONFIG_ERROR_CLOCK;
    }

    return error;
}

static void MT6835_StopDMA(void)
{
    CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    if ((hspi1.hdmatx != NULL) &&
        (hspi1.hdmatx->State == HAL_DMA_STATE_BUSY)) {
        (void)HAL_DMA_Abort(hspi1.hdmatx);
    }

    if ((hspi1.hdmarx != NULL) &&
        (hspi1.hdmarx->State == HAL_DMA_STATE_BUSY)) {
        (void)HAL_DMA_Abort(hspi1.hdmarx);
    }
}

static HAL_StatusTypeDef MT6835_StartDMA(void)
{
    HAL_StatusTypeDef status;

    hspi1.hdmarx->XferHalfCpltCallback = NULL;
    hspi1.hdmarx->XferCpltCallback = MT6835_RxDMAComplete;
    hspi1.hdmarx->XferErrorCallback = MT6835_DMAError;
    hspi1.hdmarx->XferAbortCallback = NULL;

    hspi1.hdmatx->XferHalfCpltCallback = NULL;
    hspi1.hdmatx->XferCpltCallback = NULL;
    hspi1.hdmatx->XferErrorCallback = MT6835_DMAError;
    hspi1.hdmatx->XferAbortCallback = NULL;

    CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);

    status = HAL_DMA_Start_IT(
        hspi1.hdmarx,
        (uint32_t)(uintptr_t)&hspi1.Instance->DR,
        (uint32_t)(uintptr_t)MT6835_RxBuffer,
        MT6835_ANGLE_FRAME_SIZE);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_DMA_Start_IT(
        hspi1.hdmatx,
        (uint32_t)(uintptr_t)MT6835_TxBuffer,
        (uint32_t)(uintptr_t)&hspi1.Instance->DR,
        MT6835_ANGLE_FRAME_SIZE);
    if (status != HAL_OK) {
        (void)HAL_DMA_Abort(hspi1.hdmarx);
        return status;
    }

    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET);
    MT6835_CSDelay();

    MT6835_CurrentSampleCycles = DWT->CYCCNT;
    MT6835_CurrentSampleTickMs = HAL_GetTick();
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    return HAL_OK;
}

HAL_StatusTypeDef MT6835_Init(void)
{
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    MT6835_Initialized = 0U;
    MT6835_TransferBusy = 1U;
    __set_PRIMASK(interrupt_state);

    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);
    MT6835_StopDMA();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    MT6835_CycleClockHz = HAL_RCC_GetHCLKFreq();
    MT6835_VelocityFilterCutoffHz =
        MT6835_DEFAULT_VELOCITY_FILTER_HZ;
    MT6835_ResetVelocityState();

    MT6835_Reading.raw_angle = 0U;
    MT6835_Reading.angle_radians = 0.0f;
    MT6835_Reading.sample_count = 0U;
    MT6835_Reading.crc_error_count = 0U;
    MT6835_Reading.transfer_error_count = 0U;
    MT6835_Reading.status = 0U;
    MT6835_Reading.received_crc = 0U;
    MT6835_Reading.calculated_crc = 0U;
    MT6835_Reading.valid = 0U;
    MT6835_Reading.config_error = MT6835_ValidateConfiguration();

    if (MT6835_Reading.config_error != 0U) {
        MT6835_TransferBusy = 0U;
        return HAL_ERROR;
    }

    CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_LDMATX | SPI_CR2_LDMARX);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    __HAL_SPI_ENABLE(&hspi1);

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    MT6835_Initialized = 1U;
    MT6835_TransferBusy = 0U;
    __set_PRIMASK(interrupt_state);

    return HAL_OK;
}

HAL_StatusTypeDef MT6835_StartAngleRead_DMA(void)
{
    HAL_StatusTypeDef status;
    uint32_t interrupt_state;

    if (MT6835_Initialized == 0U) {
        return HAL_ERROR;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (MT6835_TransferBusy != 0U) {
        __set_PRIMASK(interrupt_state);
        return HAL_BUSY;
    }
    MT6835_TransferBusy = 1U;
    __set_PRIMASK(interrupt_state);

    if ((hspi1.hdmatx->State != HAL_DMA_STATE_READY) ||
        (hspi1.hdmarx->State != HAL_DMA_STATE_READY)) {
        MT6835_TransferBusy = 0U;
        return HAL_BUSY;
    }

    status = MT6835_StartDMA();
    if (status != HAL_OK) {
        HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);
        MT6835_StopDMA();
        ++MT6835_Reading.transfer_error_count;
        MT6835_TransferBusy = 0U;
    }

    return status;
}

void MT6835_DeInit(void)
{
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    MT6835_Initialized = 0U;
    MT6835_TransferBusy = 1U;
    __set_PRIMASK(interrupt_state);

    MT6835_StopDMA();
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);
    __HAL_SPI_DISABLE(&hspi1);

    MT6835_Reading.valid = 0U;
    MT6835_ResetVelocityState();
    MT6835_TransferBusy = 0U;
}

bool MT6835_IsBusy(void)
{
    return MT6835_TransferBusy != 0U;
}

bool MT6835_GetLatestReading(MT6835_Reading_t *destination)
{
    uint32_t interrupt_state;

    if (destination == NULL) {
        return false;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    *destination = MT6835_Reading;
    __set_PRIMASK(interrupt_state);

    return destination->valid != 0U;
}

bool MT6835_GetLatestVelocity(MT6835_Velocity_t *destination)
{
    uint32_t interrupt_state;

    if (destination == NULL) {
        return false;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    *destination = MT6835_Velocity;
    __set_PRIMASK(interrupt_state);

    return destination->valid != 0U;
}

bool MT6835_GetVelocityRadPerSecond(float *radians_per_second)
{
    MT6835_Velocity_t velocity;

    if (radians_per_second == NULL) {
        return false;
    }

    if (!MT6835_GetLatestVelocity(&velocity)) {
        return false;
    }

    *radians_per_second = velocity.radians_per_second;
    return true;
}

bool MT6835_GetVelocityRPM(float *revolutions_per_minute)
{
    MT6835_Velocity_t velocity;

    if (revolutions_per_minute == NULL) {
        return false;
    }

    if (!MT6835_GetLatestVelocity(&velocity)) {
        return false;
    }

    *revolutions_per_minute = velocity.revolutions_per_minute;
    return true;
}

void MT6835_SetVelocityFilterCutoff(float cutoff_hz)
{
    uint32_t interrupt_state;

    if (cutoff_hz < 0.0f) {
        cutoff_hz = 0.0f;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    MT6835_VelocityFilterCutoffHz = cutoff_hz;
    __set_PRIMASK(interrupt_state);
}

void MT6835_ResetVelocityEstimator(void)
{
    const uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    MT6835_ResetVelocityState();
    __set_PRIMASK(interrupt_state);
}

uint8_t MT6835_CalculateCRC(uint32_t raw_angle, uint8_t status)
{
    uint8_t crc = 0U;
    const uint8_t data[3] = {
        (uint8_t)(raw_angle >> 13U),
        (uint8_t)(raw_angle >> 5U),
        (uint8_t)((raw_angle << 3U) | (status & 0x07U))
    };

    for (uint32_t byte_index = 0U; byte_index < 3U; ++byte_index) {
        crc ^= data[byte_index];
        for (uint32_t bit_index = 0U; bit_index < 8U; ++bit_index) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            } else {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

static void MT6835_RxDMAComplete(DMA_HandleTypeDef *hdma)
{
    uint32_t raw_angle;
    uint32_t timeout = MT6835_SPI_IDLE_TIMEOUT;
    uint8_t status;
    uint8_t calculated_crc;

    if ((hspi1.hdmarx == NULL) || (hdma != hspi1.hdmarx)) {
        return;
    }

    CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    while ((__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET) &&
           (timeout > 0U)) {
        --timeout;
    }

    MT6835_CSDelay();
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);

    /* Circular RX must be stopped after one CS-framed transaction. */
    if (hspi1.hdmarx->State == HAL_DMA_STATE_BUSY) {
        (void)HAL_DMA_Abort(hspi1.hdmarx);
    }

    if (timeout == 0U) {
        MT6835_Reading.valid = 0U;
        MT6835_Velocity.valid = 0U;
        ++MT6835_Reading.transfer_error_count;
        MT6835_TransferBusy = 0U;
        return;
    }

    raw_angle = ((uint32_t)MT6835_RxBuffer[2] << 13U) |
                ((uint32_t)MT6835_RxBuffer[3] << 5U) |
                ((uint32_t)MT6835_RxBuffer[4] >> 3U);
    status = MT6835_RxBuffer[4] & 0x07U;
    calculated_crc = MT6835_CalculateCRC(raw_angle, status);

    MT6835_Reading.valid = 0U;
    MT6835_Reading.raw_angle = raw_angle;
    MT6835_Reading.angle_radians =
        (float)raw_angle * (MT6835_TWO_PI /
                            (float)MT6835_COUNTS_PER_REVOLUTION);
    MT6835_Reading.status = status;
    MT6835_Reading.received_crc = MT6835_RxBuffer[5];
    MT6835_Reading.calculated_crc = calculated_crc;
    ++MT6835_Reading.sample_count;

    if (calculated_crc == MT6835_RxBuffer[5]) {
        MT6835_UpdateVelocity(raw_angle,
                              MT6835_Reading.sample_count,
                              MT6835_CurrentSampleCycles,
                              MT6835_CurrentSampleTickMs);
        __DMB();
        MT6835_Reading.valid = 1U;
    } else {
        MT6835_Velocity.valid = 0U;
        ++MT6835_Reading.crc_error_count;
    }

    __DMB();
    MT6835_TransferBusy = 0U;
}

static void MT6835_DMAError(DMA_HandleTypeDef *hdma)
{
    if ((hdma != hspi1.hdmatx) && (hdma != hspi1.hdmarx)) {
        return;
    }

    MT6835_StopDMA();
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);

    MT6835_Reading.valid = 0U;
    MT6835_ResetVelocityState();
    ++MT6835_Reading.transfer_error_count;
    MT6835_Initialized = 0U;
    MT6835_TransferBusy = 0U;
}
