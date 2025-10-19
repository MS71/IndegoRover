#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "adc.h"

static const char *TAG = "adc";

#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12
#define EXAMPLE_ADC2_CHAN0          ADC_CHANNEL_0 // BAT TEMP0
#define EXAMPLE_ADC2_CHAN1          ADC_CHANNEL_1 // BAT TEMP1
#define EXAMPLE_ADC2_CHAN2          ADC_CHANNEL_2 // JOY 0
#define EXAMPLE_ADC2_CHAN3          ADC_CHANNEL_3 // JOY 0
#define EXAMPLE_ADC2_CHAN4          ADC_CHANNEL_4 // JOY 0
#define EXAMPLE_ADC2_CHAN5          ADC_CHANNEL_5 // JOY 0
#define EXAMPLE_ADC2_CHAN6          ADC_CHANNEL_6 // U Charge
#define EXAMPLE_ADC2_CHAN7          ADC_CHANNEL_7 // U Bat

adc_oneshot_unit_handle_t adc2_handle;

void adc_init()
{
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

        //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN1, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN2, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN3, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN4, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN5, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN6, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN7, &config));

        //-------------ADC1 Calibration Init---------------//
    //adc_cali_handle_t adc2_cali_chan0_handle = NULL;
    //adc_cali_handle_t adc2_cali_chan1_handle = NULL;
    //bool do_calibration2_chan0 = example_adc_calibration_init(ADC_UNIT_2, EXAMPLE_ADC2_CHAN0, EXAMPLE_ADC_ATTEN, &adc2_cali_chan0_handle);
    //bool do_calibration2_chan1 = example_adc_calibration_init(ADC_UNIT_2, EXAMPLE_ADC2_CHAN1, EXAMPLE_ADC_ATTEN, &adc2_cali_chan1_handle);

}

void adc_handle()
{
    int adc_raw[8] = {};
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN0, &adc_raw[0]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN1, &adc_raw[1]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN2, &adc_raw[2]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN3, &adc_raw[3]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN4, &adc_raw[4]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN5, &adc_raw[5]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN6, &adc_raw[6]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN7, &adc_raw[7]));
#if 0
    ESP_LOGI(TAG, "adc_handle T1:%d T2:%d J:(%d,%d,%d,%d) UCh:%d (%.3fV) UBat:%d (%.3fV)",
        adc_raw[0],
        adc_raw[1],
        adc_raw[2],
        adc_raw[3],
        adc_raw[4],
        adc_raw[5],
        adc_raw[6],(3.3 * 340.0 * adc_raw[6]) / (4095.0 * 10.0),
        adc_raw[7],(3.3 * 340.0 * adc_raw[7]) / (4095.0 * 10.0) );
#endif        
}

void adc_exit()
{

}

