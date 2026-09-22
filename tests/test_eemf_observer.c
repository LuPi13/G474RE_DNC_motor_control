/**
 * @file test_eemf_observer.c
 * @brief EEMF observer + PLL의 alpha-beta voltage equation 부호/convention host test.
 *
 * 알려진 정상상태 IPMSM 동작점(고정 omega_e, i_d, i_q)에서 해석적으로 계산한
 * v_alpha_beta/i_alpha_beta를 observer+PLL에 주입하고, 추정 전기각/각속도가 참값에
 * 수렴하는지 확인한다. 이 test가 통과해야 실제 fast-loop ISR에 연결한다.
 *
 * PLL의 위상검출기는 `theta_hat = theta_true`와 `theta_hat = theta_true + pi` 양쪽에서
 * phase_error가 0이 되는 잘 알려진 180도 ambiguity를 갖는다(BEMF/EEMF PLL의 일반적 한계이며
 * voltage equation 부호 오류가 아니다 — pll.h 참고). 실측으로 확인한 결과 이 ambiguity는
 * 매 tick이 아니라 정착 뒤 한 번만 pll_resolve_polarity()로 바로잡아야 안정적이다(매 tick
 * 적용하면 경계 근처에서 진동하며 오히려 발산한다). 이 test는 그 순서(warm-start seed ->
 * 정착 -> 최종 1회 polarity correction)로 검증한다.
 */

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "eemf_observer.h"
#include "pll.h"
#include "sensorless_config.h"
#include "transform.h"

#define TEST_PI_F 3.14159265358979323846f
#define TEST_TWO_PI_F (2.0f * TEST_PI_F)
#define TEST_RS_OHM 0.059f
#define TEST_LD_H 147.5e-6f
#define TEST_LQ_H 155.5e-6f
#define TEST_PSI_F_WB 0.0167f
#define TEST_TS_S 25.0e-6f

static float test_wrap_angle(float theta_rad)
{
    float wrapped_rad = fmodf(theta_rad, TEST_TWO_PI_F);

    if (wrapped_rad < 0.0f) {
        wrapped_rad += TEST_TWO_PI_F;
    }

    return wrapped_rad;
}

static float test_wrap_angle_error(float theta_rad)
{
    float wrapped_rad = test_wrap_angle(theta_rad);

    if (wrapped_rad > TEST_PI_F) {
        wrapped_rad -= TEST_TWO_PI_F;
    }

    return wrapped_rad;
}

/**
 * @brief 정상상태(di_d/dt = di_q/dt = 0) IPMSM 동작점의 alpha-beta 전류/전압을 계산한다.
 */
static void test_plant_sample(
    float omega_e_rad_s,
    float i_d_a,
    float i_q_a,
    float theta_e_rad,
    alpha_beta_t *i_alpha_beta,
    alpha_beta_t *v_alpha_beta
)
{
    const dq_t i_dq = {i_d_a, i_q_a};
    const dq_t v_dq = {
        (TEST_RS_OHM * i_d_a) - (omega_e_rad_s * TEST_LQ_H * i_q_a),
        (TEST_RS_OHM * i_q_a) +
            (omega_e_rad_s * ((TEST_LD_H * i_d_a) + TEST_PSI_F_WB)),
    };
    const float sin_theta = sinf(theta_e_rad);
    const float cos_theta = cosf(theta_e_rad);

    transform_inverse_park(&i_dq, sin_theta, cos_theta, i_alpha_beta);
    transform_inverse_park(&v_dq, sin_theta, cos_theta, v_alpha_beta);
}

static void test_run_case(
    const char *label,
    float omega_e_rad_s,
    float i_d_a,
    float i_q_a,
    float settle_time_s,
    float theta_tolerance_rad,
    float omega_tolerance_rad_s
)
{
    eemf_observer_t observer;
    pll_t pll;
    eemf_observer_output_t observer_output;
    pll_output_t pll_output = {0};
    float theta_e_rad = 0.0f;
    const uint32_t step_count = (uint32_t)(settle_time_s / TEST_TS_S);
    /* 실기 shadow-mode 기동은 정지 상태의 큰 step이 아니라, Hall이 이미 추종 중인 회전체
     * 위에서 시작한다. Hall의 근사 각도(오프셋 존재)와 근사 속도(오차 존재)로 observer와
     * PLL을 함께 warm-start해 그 상황을 흉내낸다. 둘 중 하나만 seed하면 나머지가 수 ms
     * 동안 스스로 수렴하는 구간에서 방향이 어긋나 false-lock을 유발할 수 있다(실측 확인). */
    const float hall_theta_offset_rad = 0.3f;
    const float hall_omega_error_fraction = 0.1f;
    const float seed_theta_e_rad = theta_e_rad + hall_theta_offset_rad;
    const float seed_omega_e_rad_s =
        omega_e_rad_s * (1.0f - hall_omega_error_fraction);
    const float seed_emf_magnitude_v = fabsf(seed_omega_e_rad_s) * TEST_PSI_F_WB;
    const alpha_beta_t seed_i_hat = {0.0f, 0.0f};
    const alpha_beta_t seed_e_hat = {
        -sinf(seed_theta_e_rad) * seed_emf_magnitude_v,
        cosf(seed_theta_e_rad) * seed_emf_magnitude_v,
    };

    assert(eemf_observer_init(&observer, &sensorless_config_eemf_observer) ==
        EEMF_OBSERVER_STATUS_OK);
    assert(eemf_observer_seed(&observer, &seed_i_hat, &seed_e_hat) ==
        EEMF_OBSERVER_STATUS_OK);
    assert(pll_init(&pll, &sensorless_config_pll) == PLL_STATUS_OK);
    assert(pll_seed(&pll, seed_theta_e_rad, seed_omega_e_rad_s) ==
        PLL_STATUS_OK);
    pll_output.omega_e_rad_s = seed_omega_e_rad_s;

    for (uint32_t step = 0U; step < step_count; ++step) {
        alpha_beta_t i_alpha_beta;
        alpha_beta_t v_alpha_beta;

        test_plant_sample(
            omega_e_rad_s, i_d_a, i_q_a, theta_e_rad,
            &i_alpha_beta, &v_alpha_beta
        );

        assert(eemf_observer_update(
            &observer, &i_alpha_beta, &v_alpha_beta,
            pll_output.omega_e_rad_s, &observer_output
        ) == EEMF_OBSERVER_STATUS_OK);
        assert(pll_update(&pll, &observer_output.e_hat, &pll_output) ==
            PLL_STATUS_OK);

        theta_e_rad = test_wrap_angle(theta_e_rad + (omega_e_rad_s * TEST_TS_S));
    }

    assert(pll_resolve_polarity(&pll, cosf(theta_e_rad), sinf(theta_e_rad)) ==
        PLL_STATUS_OK);

    const float estimated_theta_e_rad = pll_get_theta_e_rad(&pll);
    const float angle_error_rad =
        test_wrap_angle_error(theta_e_rad - estimated_theta_e_rad);
    const float omega_error_rad_s = omega_e_rad_s - pll_output.omega_e_rad_s;

    printf(
        "%s: angle_error=%f rad, omega_error=%f rad/s, has_valid_speed=%d\n",
        label,
        angle_error_rad,
        omega_error_rad_s,
        pll_output.has_valid_speed
    );

    assert(pll_output.has_valid_speed);
    assert(fabsf(angle_error_rad) <= theta_tolerance_rad);
    assert(fabsf(omega_error_rad_s) <= omega_tolerance_rad_s);
}

int main(void)
{
    /* 500 rpm(4 pole-pair) forward, i_d=0: 순수 psi_f 기반 EEMF. */
    test_run_case("forward_500rpm_id0", 209.44f, 0.0f, 2.0f, 0.08f, 0.08f, 5.0f);
    /* 3000 rpm forward, i_d=0. */
    test_run_case("forward_3000rpm_id0", 1256.64f, 0.0f, 2.0f, 0.08f, 0.08f, 5.0f);
    /* 500 rpm reverse, i_d != 0로 (Ld-Lq) saliency cross term을 함께 검증. */
    test_run_case("reverse_500rpm_saliency", -209.44f, -1.0f, 2.0f, 0.08f, 0.08f, 5.0f);
    /* 3000 rpm reverse, i_d=0. */
    test_run_case("reverse_3000rpm_id0", -1256.64f, 0.0f, 2.0f, 0.08f, 0.08f, 5.0f);

    puts("eemf observer/pll tests passed");
    return 0;
}
