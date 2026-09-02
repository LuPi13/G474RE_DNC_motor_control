/**
 * @file adc_driver.h
 * @brief ADC 전압/전류 측정 드라이버 헤더 파일
 * @date 2026-04-03
 * @author JWDNC
 * @version 1.0
 */

#ifndef INC_ADC_DRIVER_H_
#define INC_ADC_DRIVER_H_

#define ADC_MAX_VALUE 4095.0f // 12비트 ADC의 최대값
#define ADC_REF_VOLTAGE 3.3f // ADC 참조 전압 (V)

#define CURRENT_SENSOR_OFFSET_SAMPLES 1000 // 전류 센서 오프셋 보정을 위한 샘플 수


#include "stm32g4xx_hal.h"
#include "q31.h"

typedef struct {
    uint16_t RawIA; // ADC 채널 1 원시값
    uint16_t RawIB; // ADC 채널 2 원시값
    uint16_t RawIC; // ADC 채널 3 원시값
    uint16_t RawVDC; // ADC 채널 4 원시값 (차동 입력)
} sADCRawData;

typedef struct {
    ADC_HandleTypeDef *hadc; // ADC 핸들러 포인터

    sADCRawData *pRawData; // 원시 ADC 데이터

    uint16_t OffsetIA; // 전류 센서 오프셋 (ADC 단위)
    uint16_t OffsetIB; // 전류 센서 오프셋 (ADC 단위)
    uint16_t OffsetIC; // 전류 센서 오프셋 (ADC 단위)
    uint16_t OffsetVDC; // 전압 센서 오프셋 (ADC 단위)

    float ScaleI; // 전류 센서 스케일링 팩터 (ADC 단위 * 스케일링 = 실제값(A))
    float ScaleV; // 전압 센서 스케일링 팩터 (ADC 단위 * 스케일링 = 실제값(V))
} sADCHandle;

typedef struct {
    float IA;
    float IB;
    float IC;
} sPhaseCurrents;


/**
 * @brief ADC 초기화 함수
 * @param pHandle ADC 핸들러 구조체 포인터
 * @note
 */
void ADC_Init(sADCHandle *pHandle);

/**
 * @brief ADC로부터 원시 데이터 읽기 및 보정/스케일링하여 pu 단위로 변환
 * @param pHandle ADC 핸들러 구조체 포인터
 * @note
 */
void ADC_CalibrateOffsets(sADCHandle *pHandle);

/**
 * @brief ADC로부터 원시 데이터 읽기 및 보정/스케일링하여 pu 단위로 변환
 * @param pHandle ADC 핸들러 구조체 포인터
 * @param pCurrentsOut 보정 및 스케일링된 전류가 저장될 sPhaseCurrents 구조체 포인터
 * @note
 */
void ADC_GetPhaseCurrents(sADCHandle *pHandle, sPhaseCurrents *pCurrentsOut);

/**
 * @brief ADC로부터 원시 데이터 읽기 및 보정/스케일링하여 실제값으로 변환
 * @param pHandle ADC 핸들러 구조체 포인터
 * @return 보정 및 스케일링된 DC 버스 전압 (pu)
 */
float ADC_GetVDC(sADCHandle *pHandle);

#endif /* INC_ADC_DRIVER_H_ */
