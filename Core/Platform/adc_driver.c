/**
 * @file adc_driver.c
 * @brief ADC 초기화/정지, 3상 완료 수집, DC 전압 읽기와 SI 단위 환산 구현.
 * @ingroup platform_adc_driver
 *
 * 보드 매핑은 설정으로 받고, 상위 제어 실행은 완료 결과를 받은 App이 결정한다.
 * 공개 API의 호출 조건과 오류 의미는 adc_driver.h 에서 정의한다.
 * @see @ref platform_adc_driver "ADC driver 사용 안내"
 */

#include "adc_driver.h"

#include <math.h>

#include "stm32g4xx_ll_adc.h"

#define ADC_DRIVER_PHASE_COUNT     3U     /**< 수집할 논리적 전류 상의 개수. */
#define ADC_DRIVER_CURRENT_MASK    0x07U  /**< a/b/c상 완료 bit 0/1/2가 모두 설정된 값. */
#define ADC_DRIVER_VOLTAGE_MASK    0x08U  /**< active_mask에서 regular 전압 그룹을 나타내는 bit. */
#define ADC_DRIVER_MAX_CODE        4095U  /**< 지원하는 12-bit 결과의 최댓값 [count]. */

/**
 * @brief 논리적 상 index에 해당하는 설정 필드를 반환한다.
 * @param[in] config 유효한 driver 설정.
 * @param[in] index a/b/c상에 대응하는 0/1/2.
 * @return config 내부의 해당 상 설정 주소.
 * @pre index는 [0, 2]여야 한다.
 * @note 구조체 필드 사이의 pointer 산술에 의존하지 않고 명시적으로 매핑한다.
 */
static const adc_driver_phase_config_t *adc_driver_get_phase(
    const adc_driver_config_t *config,
    uint32_t index
)
{
    switch (index) {
    case 0U:
        return &config->phase_a;
    case 1U:
        return &config->phase_b;
    default:
        return &config->phase_c;
    }
}

/**
 * @brief 센서 영점과 환산 이득의 설정 범위를 확인한다.
 * @param[in] offset_counts 0 A 또는 0 V의 ADC code [count].
 * @param[in] gain 전류 [A/count] 또는 전압 [V/count] 이득.
 * @return 영점이 유한한 [0, 4095]이고 이득이 유한한 0 이외 값이면 true.
 */
static bool adc_driver_is_valid_scale(float offset_counts, float gain)
{
    return isfinite(offset_counts) && (offset_counts >= 0.0f) &&
           (offset_counts <= (float)ADC_DRIVER_MAX_CODE) &&
           isfinite(gain) && (gain != 0.0f);
}

/**
 * @brief ADC 입력과 공통 지원 조건을 hardware 변경 없이 확인한다.
 * @param[in] adc 검사할 ADC handle. NULL이면 유효하지 않은 입력으로 처리함.
 * @param[in] channel 해당 ADC에서 사용할 ADC_CHANNEL_* 값.
 * @return ADC/channel이 유효하고 12-bit, right-aligned, independent mode이면 true.
 */
static bool adc_driver_is_valid_input(ADC_HandleTypeDef *adc, uint32_t channel)
{
    if ((adc == NULL) || !IS_ADC_ALL_INSTANCE(adc->Instance)) {
        return false;
    }

    return IS_ADC_CHANNEL(adc, channel) &&
           (adc->Init.Resolution == ADC_RESOLUTION_12B) &&
           (adc->Init.DataAlign == ADC_DATAALIGN_RIGHT) &&
           (LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(adc->Instance)) ==
            LL_ADC_MULTI_INDEPENDENT);
}

/**
 * @brief LL에서 읽은 channel과 설정 channel의 번호를 비교한다.
 * @param[in] configured Sequencer에서 읽은 channel 표현.
 * @param[in] expected 설정에 지정한 ADC_CHANNEL_* 값.
 * @return Channel 번호 부분이 일치하면 true.
 * @note LL 조회 결과에는 부가 식별 bit가 생략될 수 있어 전체 값으로 비교하지 않는다.
 */
static bool adc_driver_is_same_channel(uint32_t configured, uint32_t expected)
{
    return (configured & ADC_CHANNEL_ID_NUMBER_MASK) ==
           (expected & ADC_CHANNEL_ID_NUMBER_MASK);
}

/**
 * @brief 매핑, 환산 계수, 실제 sequencer와 trigger 구성이 지원 범위 안인지 검사한다.
 * @param[in] config 검사할 driver 설정. NULL이 아니어야 함.
 * @return 세 전류 ADC가 서로 다르고, 설정과 hardware 구성이 지원 조건에 맞으면 true.
 * @note 실제 sampling 시점, ISR deadline 및 oversampling의 최종 결과 범위까지
 *       증명하는 검사는 아니다. 전압/전류 공유 시 동일한 HAL handle을 요구한다.
 */
static bool adc_driver_is_valid_config(const adc_driver_config_t *config)
{
    for (uint32_t i = 0U; i < ADC_DRIVER_PHASE_COUNT; ++i) {
        const adc_driver_phase_config_t *phase = adc_driver_get_phase(config, i);
        if (!adc_driver_is_valid_input(phase->adc, phase->channel) ||
            !adc_driver_is_valid_scale(phase->offset_counts, phase->gain_a_per_count) ||
            (phase->injected_rank != ADC_INJECTED_RANK_1)) {
            return false;
        }

        ADC_TypeDef *adc = phase->adc->Instance;
        if ((LL_ADC_INJ_GetSequencerLength(adc) != LL_ADC_INJ_SEQ_SCAN_DISABLE) ||
            !adc_driver_is_same_channel(
                LL_ADC_INJ_GetSequencerRanks(adc, phase->injected_rank), phase->channel) ||
            (LL_ADC_INJ_GetTriggerSource(adc) == LL_ADC_INJ_TRIG_SOFTWARE) ||
            (READ_BIT(adc->CFGR, ADC_CFGR_JAUTO | ADC_CFGR_JDISCEN | ADC_CFGR_JQM) != 0U)) {
            return false;
        }

        for (uint32_t j = 0U; j < i; ++j) {
            if (adc == adc_driver_get_phase(config, j)->adc->Instance) {
                return false;
            }
        }
    }

    const adc_driver_voltage_config_t *voltage = &config->dc_link;
    if (!adc_driver_is_valid_input(voltage->adc, voltage->channel) ||
        !adc_driver_is_valid_scale(voltage->offset_counts, voltage->gain_v_per_count)) {
        return false;
    }

    ADC_TypeDef *adc = voltage->adc->Instance;
    if ((LL_ADC_REG_GetSequencerLength(adc) != LL_ADC_REG_SEQ_SCAN_DISABLE) ||
        !adc_driver_is_same_channel(
            LL_ADC_REG_GetSequencerRanks(adc, ADC_REGULAR_RANK_1), voltage->channel) ||
        (LL_ADC_REG_GetTriggerSource(adc) == LL_ADC_REG_TRIG_SOFTWARE) ||
        (READ_BIT(adc->CFGR, ADC_CFGR_DMAEN | ADC_CFGR_CONT | ADC_CFGR_AUTDLY) != 0U) ||
        (voltage->adc->Init.DMAContinuousRequests != DISABLE) ||
        (READ_BIT(adc->IER, ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR) != 0U)) {
        return false;
    }

    /* 같은 주변장치를 서로 다른 handle로 다루면 HAL 상태가 분리되므로 거절한다. */
    for (uint32_t i = 0U; i < ADC_DRIVER_PHASE_COUNT; ++i) {
        ADC_HandleTypeDef *phase_adc = adc_driver_get_phase(config, i)->adc;
        if ((adc == phase_adc->Instance) && (voltage->adc != phase_adc)) {
            return false;
        }
    }
    return true;
}

adc_driver_status_t adc_driver_init(
    adc_driver_t *self,
    const adc_driver_config_t *config
)
{
    if ((self == NULL) || (config == NULL)) {
        return ADC_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (self->is_running || (self->active_mask != 0U)) {
        return ADC_DRIVER_STATUS_INVALID_STATE;
    }
    if (!adc_driver_is_valid_config(config)) {
        return ADC_DRIVER_STATUS_INVALID_CONFIG;
    }

    ADC_HandleTypeDef *inputs[4] = {
        config->phase_a.adc, config->phase_b.adc, config->phase_c.adc, config->dc_link.adc
    };
    const uint32_t channels[4] = {
        config->phase_a.channel, config->phase_b.channel,
        config->phase_c.channel, config->dc_link.channel
    };
    uint32_t modes[4];

    /* 일부를 보정한 뒤 실행 중인 ADC를 발견하지 않도록 모든 입력을 먼저 확인한다. */
    for (uint32_t i = 0U; i < 4U; ++i) {
        if (READ_BIT(inputs[i]->Instance->CR, ADC_CR_ADSTART | ADC_CR_JADSTART) != 0U) {
            return ADC_DRIVER_STATUS_INVALID_STATE;
        }
        modes[i] = (LL_ADC_GetChannelSingleDiff(inputs[i]->Instance, channels[i]) != 0U)
                 ? ADC_DIFFERENTIAL_ENDED : ADC_SINGLE_ENDED;
    }

    /* config가 self->config를 가리키는 경우에도 설정을 보존한다. */
    const adc_driver_config_t saved_config = *config;
    *self = (adc_driver_t){0};
    self->config = saved_config;

    for (uint32_t i = 0U; i < 4U; ++i) {
        bool already_calibrated = false;
        for (uint32_t j = 0U; j < i; ++j) {
            if ((inputs[i] == inputs[j]) && (modes[i] == modes[j])) {
                already_calibrated = true;
                break;
            }
        }
        if (!already_calibrated &&
            (HAL_ADCEx_Calibration_Start(inputs[i], modes[i]) != HAL_OK)) {
            return ADC_DRIVER_STATUS_HAL_ERROR;
        }
    }

    self->is_initialized = true;
    return ADC_DRIVER_STATUS_OK;
}

adc_driver_status_t adc_driver_start(adc_driver_t *self)
{
    if ((self == NULL) || !self->is_initialized) {
        return ADC_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (self->has_sync_error) {
        return ADC_DRIVER_STATUS_SYNC_ERROR;
    }
    if (self->is_running) {
        return ADC_DRIVER_STATUS_OK;
    }
    if (self->active_mask != 0U) {
        return ADC_DRIVER_STATUS_INVALID_STATE;
    }

    self->complete_mask = 0U;
    self->is_sample_ready = false;

    /* HAL의 중간 실패로 ADC만 활성화된 경우도 정리 대상에 포함한다. */
    self->active_mask |= ADC_DRIVER_VOLTAGE_MASK;
    if (HAL_ADC_Start(self->config.dc_link.adc) != HAL_OK) {
        (void)adc_driver_stop(self);
        return ADC_DRIVER_STATUS_HAL_ERROR;
    }

    for (uint32_t i = 0U; i < ADC_DRIVER_PHASE_COUNT; ++i) {
        self->active_mask |= 1U << i;
        if (HAL_ADCEx_InjectedStart_IT(adc_driver_get_phase(&self->config, i)->adc) != HAL_OK) {
            (void)adc_driver_stop(self);
            return ADC_DRIVER_STATUS_HAL_ERROR;
        }
    }

    self->is_running = true;
    return ADC_DRIVER_STATUS_OK;
}

adc_driver_status_t adc_driver_stop(adc_driver_t *self)
{
    if ((self == NULL) || !self->is_initialized) {
        return ADC_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    self->is_running = false;
    self->complete_mask = 0U;
    self->is_sample_ready = false;
    adc_driver_status_t status = ADC_DRIVER_STATUS_OK;

    /* HAL_ADC_Stop은 같은 ADC의 injected도 정지하므로 injected IT를 먼저 정리한다. */
    for (uint32_t i = 0U; i < ADC_DRIVER_PHASE_COUNT; ++i) {
        const uint32_t bit = 1U << i;
        if ((self->active_mask & bit) != 0U) {
            if (HAL_ADCEx_InjectedStop_IT(adc_driver_get_phase(&self->config, i)->adc) == HAL_OK) {
                self->active_mask &= ~bit;
            } else {
                status = ADC_DRIVER_STATUS_HAL_ERROR;
            }
        }
    }
    if ((self->active_mask & ADC_DRIVER_VOLTAGE_MASK) != 0U) {
        if (HAL_ADC_Stop(self->config.dc_link.adc) == HAL_OK) {
            self->active_mask &= ~ADC_DRIVER_VOLTAGE_MASK;
        } else {
            status = ADC_DRIVER_STATUS_HAL_ERROR;
        }
    }
    if (self->active_mask == 0U) {
        self->has_sync_error = false;
    }
    return status;
}

adc_driver_status_t adc_driver_handle_injected_complete(
    adc_driver_t *self,
    ADC_HandleTypeDef *hadc,
    bool *is_complete
)
{
    if (is_complete == NULL) {
        return ADC_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    *is_complete = false;
    if ((self == NULL) || (hadc == NULL) || !self->is_initialized) {
        return ADC_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_running) {
        return ADC_DRIVER_STATUS_INVALID_STATE;
    }
    if (self->has_sync_error) {
        return ADC_DRIVER_STATUS_SYNC_ERROR;
    }

    for (uint32_t i = 0U; i < ADC_DRIVER_PHASE_COUNT; ++i) {
        const adc_driver_phase_config_t *phase = adc_driver_get_phase(&self->config, i);
        if (hadc != phase->adc) {
            continue;
        }

        const uint32_t bit = 1U << i;
        if (((self->complete_mask & bit) != 0U) || self->is_sample_ready) {
            self->has_sync_error = true;
            self->is_sample_ready = false;
            self->complete_mask = 0U;
            return ADC_DRIVER_STATUS_SYNC_ERROR;
        }

        const uint32_t code = HAL_ADCEx_InjectedGetValue(hadc, phase->injected_rank);
        if (code > ADC_DRIVER_MAX_CODE) {
            self->has_sync_error = true;
            self->complete_mask = 0U;
            return ADC_DRIVER_STATUS_INVALID_SAMPLE;
        }
        self->current_raw[i] = (uint16_t)code;
        self->complete_mask |= bit;
        if (self->complete_mask == ADC_DRIVER_CURRENT_MASK) {
            self->complete_mask = 0U;
            self->is_sample_ready = true;
            *is_complete = true;
        }
        return ADC_DRIVER_STATUS_OK;
    }
    return ADC_DRIVER_STATUS_INVALID_ARGUMENT;
}

adc_driver_status_t adc_driver_read_raw(
    adc_driver_t *self,
    adc_driver_raw_sample_t *sample
)
{
    if ((self == NULL) || (sample == NULL) || !self->is_initialized) {
        return ADC_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_running) {
        return ADC_DRIVER_STATUS_INVALID_STATE;
    }
    if (self->has_sync_error) {
        return ADC_DRIVER_STATUS_SYNC_ERROR;
    }
    if (!self->is_sample_ready) {
        return ADC_DRIVER_STATUS_NOT_READY;
    }

    self->is_sample_ready = false;
    ADC_HandleTypeDef *adc = self->config.dc_link.adc;
    const uint32_t flags = READ_REG(adc->Instance->ISR);
    if ((flags & ADC_FLAG_EOC) == 0U) {
        return ADC_DRIVER_STATUS_NOT_READY;
    }

    const uint32_t voltage = HAL_ADC_GetValue(adc);
    const bool overrun = ((flags | READ_REG(adc->Instance->ISR)) & ADC_FLAG_OVR) != 0U;
    /* DR 읽기는 EOC를 해제한다. 단일 regular의 EOS/OVR은 여기서 소비한다. */
    __HAL_ADC_CLEAR_FLAG(adc, ADC_FLAG_EOS | ADC_FLAG_OVR);
    if (overrun) {
        return ADC_DRIVER_STATUS_OVERRUN;
    }
    if (voltage > ADC_DRIVER_MAX_CODE) {
        return ADC_DRIVER_STATUS_INVALID_SAMPLE;
    }

    *sample = (adc_driver_raw_sample_t){
        .phase_a = self->current_raw[0],
        .phase_b = self->current_raw[1],
        .phase_c = self->current_raw[2],
        .dc_link = (uint16_t)voltage,
    };
    return ADC_DRIVER_STATUS_OK;
}

adc_driver_status_t adc_driver_convert(
    const adc_driver_t *self,
    const adc_driver_raw_sample_t *sample,
    abc_t *i_abc,
    float *v_dc
)
{
    if ((self == NULL) || (sample == NULL) || (i_abc == NULL) || (v_dc == NULL) ||
        !self->is_initialized || (sample->phase_a > ADC_DRIVER_MAX_CODE) ||
        (sample->phase_b > ADC_DRIVER_MAX_CODE) || (sample->phase_c > ADC_DRIVER_MAX_CODE) ||
        (sample->dc_link > ADC_DRIVER_MAX_CODE)) {
        return ADC_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    const adc_driver_config_t *config = &self->config;
    *i_abc = (abc_t){
        .a = ((float)sample->phase_a - config->phase_a.offset_counts)
           * config->phase_a.gain_a_per_count,
        .b = ((float)sample->phase_b - config->phase_b.offset_counts)
           * config->phase_b.gain_a_per_count,
        .c = ((float)sample->phase_c - config->phase_c.offset_counts)
           * config->phase_c.gain_a_per_count,
    };
    /* 제공된 전압식의 절편은 gain * 2048과 반올림 오차 범위에서 같다. */
    *v_dc = ((float)sample->dc_link - config->dc_link.offset_counts)
          * config->dc_link.gain_v_per_count;
    return ADC_DRIVER_STATUS_OK;
}
