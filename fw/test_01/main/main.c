/*
 * SPDX-FileCopyrightText: 2020-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_wake_stub.h"
#include "driver/rtc_io.h"
#include "rtc_wake_stub_example.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <unistd.h>

#define CONFIG_MICRO_ROS_APP_STACK 16000
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5


#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "i2c.h"
#include "cam.h"
#include "adc.h"
#include "drive_motor.h"
#include "lawn_motor.h"

static const char *TAG = "example";

//#define GPIO_MAIN_PWR_ON 38

#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

uint8_t IP_ADDRESS[4];

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        IP_ADDRESS[0] = esp_ip4_addr_get_byte(&event->ip_info.ip, 0);
        IP_ADDRESS[1] = esp_ip4_addr_get_byte(&event->ip_info.ip, 1);
        IP_ADDRESS[2] = esp_ip4_addr_get_byte(&event->ip_info.ip, 2);
        IP_ADDRESS[3] = esp_ip4_addr_get_byte(&event->ip_info.ip, 3);
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,

#ifdef CONFIG_PM_ENABLE
            .listen_interval = 5,
            /* Listen interval for ESP32 station to receive beacon when WIFI_PS_MAX_MODEM is set.
            Units: AP beacon intervals. Defaults to 3 if set to 0. */
#endif //CONFIG_PM_ENABLE *
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

#ifdef CONFIG_PM_ENABLE
    ESP_LOGI(TAG, "esp_wifi_set_ps().");
    esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    /* Call esp_wifi_set_ps(WIFI_PS_MIN_MODEM) to enable Modem-sleep minimum power save mode
    or esp_wifi_set_ps(WIFI_PS_MAX_MODEM) to enable Modem-sleep maximum power save mode after
    calling esp_wifi_init(). When station connects to AP, Modem-sleep will start.
    When station disconnects from AP, Modem-sleep will stop. */
#endif /* CONFIG_PM_ENABLE */

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher_1;
rcl_publisher_t publisher_2;

void thread_1(void * arg)
{
	std_msgs__msg__Int32 msg;
	msg.data = 0;
	while(1){
		RCSOFTCHECK(rcl_publish(&publisher_1, &msg, NULL));
		msg.data++;
		usleep(1000000);
	}
}

void thread_2(void * arg)
{
	std_msgs__msg__Int32 msg;
	msg.data = 0;
	while(1){
		RCSOFTCHECK(rcl_publish(&publisher_2, &msg, NULL));
		msg.data--;
		usleep(500000);
	}
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "multithread_node", "", &support));

	// create two publishers
	RCCHECK(rclc_publisher_init_default(
		&publisher_1,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"multithread_publisher_1"));

	RCCHECK(rclc_publisher_init_default(
		&publisher_2,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"multithread_publisher_2"));


	xTaskCreate(thread_1,
		"thread_1",
		CONFIG_MICRO_ROS_APP_STACK,
		NULL,
		CONFIG_MICRO_ROS_APP_TASK_PRIO,
		NULL);

	xTaskCreate(thread_2,
		"thread_2",
		CONFIG_MICRO_ROS_APP_STACK,
		NULL,
		CONFIG_MICRO_ROS_APP_TASK_PRIO + 1,
		NULL);

	while(1){
		sleep(100);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher_1, &node));
	RCCHECK(rcl_publisher_fini(&publisher_2, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main(void)
{
    // sleep_enter_time stored in RTC memory
    static RTC_DATA_ATTR struct timeval sleep_enter_time;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
        printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
    }

#ifdef GPIO_MAIN_PWR_ON
    {
	    gpio_config_t io_conf = {};
	    //disable interrupt
	    io_conf.intr_type = GPIO_INTR_DISABLE;
	    //set as output mode
	    io_conf.mode = GPIO_MODE_OUTPUT;
	    //bit mask of the pins that you want to set,e.g.GPIO18/19
	    io_conf.pin_bit_mask = (1<<GPIO_MAIN_PWR_ON);
	    //disable pull-down mode
	    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
	    //disable pull-up mode
	    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	    //configure GPIO with the given settings
	    gpio_config(&io_conf);    
    }
#endif

    //gpio_hold_dis(GPIO_MAIN_PWR_ON);
#ifdef GPIO_MAIN_PWR_ON
    gpio_set_level(GPIO_MAIN_PWR_ON, 1);
#endif
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

 #if 0
    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL);
#endif
    //gpio_deep_sleep_hold_en();       

    camera_init();
    i2c_init();
    //adc_init();
    drive_motor_init();
    lawn_motor_init();

    int loopcnt = 300000;
    while( loopcnt-- > 0 )
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        //adc_handle();
        i2c_handle();
        camera_handle();
    }

    drive_motor_exit();
    lawn_motor_exit();
    //adc_exit();
    i2c_exit();
    camera_exit();
    
    vTaskDelay(100 / portTICK_PERIOD_MS);

#ifdef GPIO_MAIN_PWR_ON
    gpio_sleep_set_direction(GPIO_MAIN_PWR_ON,GPIO_MODE_OUTPUT);
    gpio_sleep_set_pull_mode(GPIO_MAIN_PWR_ON,GPIO_PULLDOWN_ONLY);
    gpio_set_level(GPIO_MAIN_PWR_ON, 0);
    //gpio_hold_en(GPIO_MAIN_PWR_ON);
#endif    

    const int wakeup_time_sec = 10;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

#if CONFIG_IDF_TARGET_ESP32
    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    rtc_gpio_isolate(GPIO_NUM_12);
#endif

    // Set the wake stub function
    esp_set_deep_sleep_wake_stub(&wake_stub_example);

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);

    esp_deep_sleep_start();
}
