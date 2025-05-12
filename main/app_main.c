#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "foc/esp_foc.h"
#include "svpwm/esp_svpwm.h"

static const char *TAG = "dual_foc";

// tick rate of clock, 10MHz
#define MCPWM_TIMER_RESOLUTION_HZ 10000000

// length of pwm period, period = resolution / MCPWM_PERIOD = 10KHz
// MCPWM_PERIOD is how many ticks per period
// do not change, necessary in init setup
#define MCPWM_PERIOD        1000

// wave frequency, frequency in Hz
#define FOC_WAVE_FREQ    50     

// amplitude, should only affect torque not speed
// can't be greater than period/2, otherwise overlapping high/low = shootthrough
#define FOC_WAVE_AMPL    50        // Wave amplitude, Use up-down timer mode, max value should be (MCPWM_PERIOD/2) 
//The actual FOC wave frequency appears to be proportional to this value.

// freq = 50, amplt = 50 is same slow moving as before

// Inverter 1 GPIO definitions
#define FOC1_PWM_UH_GPIO 13
#define FOC1_PWM_UL_GPIO 14
#define FOC1_PWM_VH_GPIO 11
#define FOC1_PWM_VL_GPIO 12
#define FOC1_PWM_WH_GPIO 9
#define FOC1_PWM_WL_GPIO 10

// Inverter 2 GPIO definitions
#define FOC2_PWM_UH_GPIO 21
#define FOC2_PWM_UL_GPIO 47
#define FOC2_PWM_VH_GPIO 35
#define FOC2_PWM_VL_GPIO 48
#define FOC2_PWM_WH_GPIO 37
#define FOC2_PWM_WL_GPIO 36

SemaphoreHandle_t semaphore_inv1;
SemaphoreHandle_t semaphore_inv2;

bool inverter_update_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
{
    BaseType_t task_yield = pdFALSE;
    xSemaphoreGiveFromISR(*((SemaphoreHandle_t *)user_ctx), &task_yield);
    return task_yield;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Hello Dual FOC");

    inverter_config_t cfg1 = {
        .timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ, 
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN, // UP_DOWN mode will generate center align pwm wave, which can reduce MOSFET switch times on same effect, extend life
            .period_ticks = MCPWM_PERIOD,
        },
        .operator_config = {
            .group_id = 0,
        },
        .compare_config = {
            .flags.update_cmp_on_tez = true,
        },
        .gen_gpios = {
            {FOC1_PWM_UH_GPIO, FOC1_PWM_UL_GPIO}, 
            {FOC1_PWM_VH_GPIO, FOC1_PWM_VL_GPIO}, 
            {FOC1_PWM_WH_GPIO, FOC1_PWM_WL_GPIO},
        },
        .dt_config = {
            .posedge_delay_ticks = 5,
        },
        .inv_dt_config = {
            .negedge_delay_ticks = 5, 
            .flags.invert_output = true,
        },
    };

    inverter_config_t cfg2 = {
        .timer_config = {
            .group_id = 1,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ, 
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN, 
            .period_ticks = MCPWM_PERIOD,
        },
        .operator_config = {
            .group_id = 1,
        },
        .compare_config = {
            .flags.update_cmp_on_tez = true,
        },
        .gen_gpios = {
            {FOC2_PWM_UH_GPIO, FOC2_PWM_UL_GPIO}, 
            {FOC2_PWM_VH_GPIO, FOC2_PWM_VL_GPIO}, 
            {FOC2_PWM_WH_GPIO, FOC2_PWM_WL_GPIO},
        },
        .dt_config = {
            .posedge_delay_ticks = 5,
        },
        .inv_dt_config = {
            .negedge_delay_ticks = 5, 
            .flags.invert_output = true,
        },
    };

    inverter_handle_t inverter1, inverter2;
    ESP_ERROR_CHECK(svpwm_new_inverter(&cfg1, &inverter1));
    ESP_LOGI(TAG, "Inverter1 init OK");
    ESP_ERROR_CHECK(svpwm_new_inverter(&cfg2, &inverter2));
    ESP_LOGI(TAG, "Inverter2 init OK");

    // counting semaphore used to sync update foc calculation when mcpwm timer updated
    semaphore_inv1 = xSemaphoreCreateCounting(1, 0);
    semaphore_inv2 = xSemaphoreCreateCounting(1, 0);

    mcpwm_timer_event_callbacks_t cbs = {
        .on_full = inverter_update_cb,
    };

    ESP_ERROR_CHECK(svpwm_inverter_register_cbs(inverter1, &cbs, semaphore_inv1));
    ESP_ERROR_CHECK(svpwm_inverter_start(inverter1, MCPWM_TIMER_START_NO_STOP));
    ESP_LOGI(TAG, "Inverter1 init OK");
    
    ESP_ERROR_CHECK(svpwm_inverter_register_cbs(inverter2, &cbs, semaphore_inv2));
    ESP_ERROR_CHECK(svpwm_inverter_start(inverter2, MCPWM_TIMER_START_NO_STOP));
    ESP_LOGI(TAG, "Inverter2 init OK");

    foc_dq_coord_t dq_out1 = {_IQ(0), _IQ(0)};
    foc_dq_coord_t dq_out2 = {_IQ(0), _IQ(0)};

    foc_ab_coord_t ab_out1, ab_out2;
    foc_uvw_coord_t uvw_out1, uvw_out2;

    int uvw_duty1[3], uvw_duty2[3];
    float elec_theta_deg1 = 0, elec_theta_deg2 = 0;
    _iq elec_theta_rad1, elec_theta_rad2;

    ESP_LOGI(TAG, "Start FOC");
    while (true)
    {
        if (xSemaphoreTake(semaphore_inv1, portMAX_DELAY)) {
            // Calculate elec_theta_deg increase step of 50Hz output on 10000Hz call
            elec_theta_deg1 += (FOC_WAVE_FREQ * 360.f) / (MCPWM_TIMER_RESOLUTION_HZ / MCPWM_PERIOD);
            if (elec_theta_deg1 >= 360) {
                elec_theta_deg1 -= 360;
            }

            elec_theta_rad1 = _IQmpy(_IQ(elec_theta_deg1), _IQ(M_PI / 180.f));
            dq_out1.d = _IQ(FOC_WAVE_AMPL);

            foc_inverse_park_transform(elec_theta_rad1, &dq_out1, &ab_out1);
            foc_svpwm_duty_calculate(&ab_out1, &uvw_out1);

            // Regular uvw data to (0 ~ (MCPWM_PERIOD/2))
            uvw_duty1[0] = _IQtoF(_IQdiv2(uvw_out1.u)) + (MCPWM_PERIOD / 4);
            uvw_duty1[1] = _IQtoF(_IQdiv2(uvw_out1.v)) + (MCPWM_PERIOD / 4);    
            uvw_duty1[2] = _IQtoF(_IQdiv2(uvw_out1.w)) + (MCPWM_PERIOD / 4);

            // output pwm duty
            ESP_ERROR_CHECK(svpwm_inverter_set_duty(inverter1, uvw_duty1[0], uvw_duty1[1], uvw_duty1[2]));
        }

        if (xSemaphoreTake(semaphore_inv2, portMAX_DELAY)) {

            elec_theta_deg2 += (FOC_WAVE_AMPL * 360.f) / (MCPWM_TIMER_RESOLUTION_HZ / FOC_WAVE_FREQ);
            if (elec_theta_deg2 >= 360) {
                elec_theta_deg2 -= 360;
            }

            elec_theta_rad2 = _IQmpy(_IQ(elec_theta_deg2), _IQ(M_PI / 180.f));
            dq_out2.d = _IQ(FOC_WAVE_AMPL);

            foc_inverse_park_transform(elec_theta_rad2, &dq_out2, &ab_out2);
            foc_svpwm_duty_calculate(&ab_out2, &uvw_out2);


            uvw_duty2[0] = _IQtoF(_IQdiv2(uvw_out2.u)) + (MCPWM_PERIOD / 4);
            uvw_duty2[1] = _IQtoF(_IQdiv2(uvw_out2.v)) + (MCPWM_PERIOD / 4);
            uvw_duty2[2] = _IQtoF(_IQdiv2(uvw_out2.w)) + (MCPWM_PERIOD / 4);


            ESP_ERROR_CHECK(svpwm_inverter_set_duty(inverter2, uvw_duty2[0], uvw_duty2[1], uvw_duty2[2]));
        }
    }

    ESP_ERROR_CHECK(svpwm_inverter_start(inverter1, MCPWM_TIMER_STOP_EMPTY));
    ESP_ERROR_CHECK(svpwm_inverter_start(inverter2, MCPWM_TIMER_STOP_EMPTY));
    ESP_ERROR_CHECK(svpwm_del_inverter(inverter1));
    ESP_ERROR_CHECK(svpwm_del_inverter(inverter2));
}