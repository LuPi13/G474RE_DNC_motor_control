/**
 * @file adc_driver.c
 * @brief ADC 전압/전류 측정 드라이버 소스 파일
 * @date 2026-04-03
 * @author JWDNC
 * @version 1.0
 */

#include "adc_driver.h"

sADCRawData adcRawDataBuffer; // ADC DMA로 채워질 원시 데이터 버퍼

void ADC_Init(sADCHandle *pHandle) {
    pHandle->pRawData = &adcRawDataBuffer;

    HAL_ADCEx_Calibration_Start(pHandle->hadc, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(pHandle->hadc, ADC_DIFFERENTIAL_ENDED);
    HAL_ADC_Start_DMA(pHandle->hadc, (uint32_t*)pHandle->pRawData, 4);

    ADC_CalibrateOffsets(pHandle);

}

void ADC_CalibrateOffsets(sADCHandle *pHandle) {
    uint32_t sumA = 0, sumB = 0, sumC = 0;

    for (int i = 0; i < CURRENT_SENSOR_OFFSET_SAMPLES; i++) {
        // ADC 변환이 완료될 때까지 대기
        while (HAL_ADC_PollForConversion(pHandle->hadc, HAL_MAX_DELAY) != HAL_OK);

        // DMA로 채워진 원시 데이터 읽기
        sumA += pHandle->pRawData->RawIA;
        sumB += pHandle->pRawData->RawIB;
        sumC += pHandle->pRawData->RawIC;

        HAL_Delay(1); // 샘플 간 약간의 지연 (필요에 따라 조정 가능)
    }

    pHandle->OffsetIA = sumA / CURRENT_SENSOR_OFFSET_SAMPLES;
    pHandle->OffsetIB = sumB / CURRENT_SENSOR_OFFSET_SAMPLES;
    pHandle->OffsetIC = sumC / CURRENT_SENSOR_OFFSET_SAMPLES;

}

void ADC_GetPhaseCurrents(sADCHandle *pHandle, sPhaseCurrents *pCurrentsOut) {
    int32_t correctedA = (int32_t)pHandle->pRawData->RawIA - (int32_t)pHandle->OffsetIA;
    int32_t correctedB = (int32_t)pHandle->pRawData->RawIB - (int32_t)pHandle->OffsetIB;
    int32_t correctedC = (int32_t)pHandle->pRawData->RawIC - (int32_t)pHandle->OffsetIC;

    float scaledA = correctedA * pHandle->ScaleI;
    float scaledB = correctedB * pHandle->ScaleI;
    float scaledC = correctedC * pHandle->ScaleI;

    pCurrentsOut->IA = scaledA;
    pCurrentsOut->IB = scaledB;
    pCurrentsOut->IC = scaledC;
}

float ADC_GetVDC(sADCHandle *pHandle) {
    int32_t correctedVDC = (int32_t)pHandle->pRawData->RawVDC;

    return correctedVDC * pHandle->ScaleV;

}
