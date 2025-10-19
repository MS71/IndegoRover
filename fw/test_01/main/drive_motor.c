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

#include "drive_motor.h"

#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

static const char *TAG = "drive_motor";


#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

#define BDC_L_MCPWM_GPIO_A              17
#define BDC_L_MCPWM_GPIO_B              18
#define BDC_L_ENCODER_GPIO_A            3
#define BDC_L_ENCODER_GPIO_B            8

#define BDC_R_MCPWM_GPIO_A              9
#define BDC_R_MCPWM_GPIO_B              10
#define BDC_R_ENCODER_GPIO_A            11
#define BDC_R_ENCODER_GPIO_B            12

#define BDC_ENCODER_PCNT_HIGH_LIMIT    100
#define BDC_ENCODER_PCNT_LOW_LIMIT    -100

#define BDC_PID_LOOP_PERIOD_MS          20   // calculate the motor speed every 10ms
#define BDC_PID_EXPECT_SPEED            1    // expected motor speed, in the pulses counted by the rotary encoder

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_channel_handle_t pcnt_chan_a;
    pcnt_channel_handle_t pcnt_chan_b;
    pcnt_unit_handle_t pcnt_unit;
    pid_ctrl_block_handle_t pid_ctrl;
    esp_timer_handle_t pid_loop_timer;
    pid_ctrl_config_t pid_config;
    int report_pulses;
    int last_pulse_count;
} motor_control_context_t;

static motor_control_context_t motor_ctrl_ctx[2] = {};
#define DRIVE_MOTOR_L 0
#define DRIVE_MOTOR_R 1

static void pid_loop_cb(void *args)
{
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_unit;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - ctx->last_pulse_count;
    if(real_pulses < 0)
    {
        real_pulses = -real_pulses;
    }

    ctx->last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;

    // calculate the speed error
    float error = BDC_PID_EXPECT_SPEED - real_pulses;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    bdc_motor_set_speed(motor, (uint32_t)new_speed);

    //ESP_LOGI(TAG, "pid_loop_cb %p new_speed=%f error=%f cur_pulse_count=%d",ctx,new_speed,error,cur_pulse_count);
}

void motor_init(motor_control_context_t* p_motor_ctrl_ctx,int pin_pwm_a,int pin_pwm_b,int pin_enc_a,int pin_enc_b)
{
    memset(p_motor_ctrl_ctx,0,sizeof(motor_control_context_t));
    ESP_LOGI(TAG, "Create DC motor");

    gpio_set_direction(pin_enc_a,GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin_enc_a,GPIO_PULLUP_ONLY);
    gpio_set_direction(pin_enc_b,GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin_enc_b,GPIO_PULLUP_ONLY);


    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = pin_pwm_a,
        .pwmb_gpio_num = pin_pwm_b,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    //bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &p_motor_ctrl_ctx->motor));

    ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal");
    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &p_motor_ctrl_ctx->pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(p_motor_ctrl_ctx->pcnt_unit, &filter_config));
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = pin_enc_a,
        .level_gpio_num = pin_enc_b,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(p_motor_ctrl_ctx->pcnt_unit, &chan_a_config, &p_motor_ctrl_ctx->pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = pin_enc_b,
        .level_gpio_num = pin_enc_a,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(p_motor_ctrl_ctx->pcnt_unit, &chan_b_config, &p_motor_ctrl_ctx->pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(p_motor_ctrl_ctx->pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(p_motor_ctrl_ctx->pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(p_motor_ctrl_ctx->pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(p_motor_ctrl_ctx->pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(p_motor_ctrl_ctx->pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(p_motor_ctrl_ctx->pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(p_motor_ctrl_ctx->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(p_motor_ctrl_ctx->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(p_motor_ctrl_ctx->pcnt_unit));

    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = 0.6,
        .ki = 0.4,
        .kd = 0.2,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    p_motor_ctrl_ctx->pid_config.init_param = pid_runtime_param;
    ESP_ERROR_CHECK(pid_new_control_block(&p_motor_ctrl_ctx->pid_config, &p_motor_ctrl_ctx->pid_ctrl));

    ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = p_motor_ctrl_ctx,
        .name = "pid_loop"
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &p_motor_ctrl_ctx->pid_loop_timer));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(p_motor_ctrl_ctx->motor));

    ESP_LOGI(TAG, "Start motor speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(p_motor_ctrl_ctx->pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));
}

/*
 * initialize ...
 */
void drive_motor_init()
{
#if 1
    motor_init(&motor_ctrl_ctx[DRIVE_MOTOR_L],
        BDC_L_MCPWM_GPIO_A,BDC_L_MCPWM_GPIO_B,
        BDC_L_ENCODER_GPIO_A,BDC_L_ENCODER_GPIO_B);
    ESP_ERROR_CHECK(bdc_motor_forward(motor_ctrl_ctx[DRIVE_MOTOR_L].motor));
#endif
#if 1
    motor_init(&motor_ctrl_ctx[DRIVE_MOTOR_R],
        BDC_R_MCPWM_GPIO_A,BDC_R_MCPWM_GPIO_B,
        BDC_R_ENCODER_GPIO_A,BDC_R_ENCODER_GPIO_B);
    ESP_ERROR_CHECK(bdc_motor_forward(motor_ctrl_ctx[DRIVE_MOTOR_R].motor));
#endif    
}

/**
 * shutdown ...
 */
void drive_motor_exit()
{

}
