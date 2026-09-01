#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include <unistd.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include "pote.h"
#include "as5600.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

static const char *TAG = "SENSORES";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define MICRO_ROS_APP_STACK      16000
#define MICRO_ROS_APP_TASK_PRIO  5

#define PUBLISH_PERIOD_MS 20

// Publicadores de micro-ROS
// =======================================

// Potenciometro
static rcl_publisher_t posicion_publisher;
static rcl_publisher_t voltaje_publisher;
// Encoder
static rcl_publisher_t angulo_publisher; 

// Mensajes de micro-ROS
// =======================================

// Potenciometro
static std_msgs__msg__Int32 posicion_msg;
static std_msgs__msg__Float32 voltaje_msg;
// Encoder
static std_msgs__msg__Float32 angulo_msg;

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time){
    
    (void) last_call_time;
    if (timer == NULL) {
        return;
    }

    // Leer el potenciometro y publicar posicion y voltaje
    pote_muestra_t muestra_pote;
    if (pote_leer(&muestra_pote) == ESP_OK) {
        
        
        posicion_msg.data = (int32_t)((muestra_pote.grados / 300.0f) * 100.0f);
        voltaje_msg.data = muestra_pote.mv_senal / 1000.0f;

        RCSOFTCHECK(rcl_publish(&posicion_publisher, &posicion_msg, NULL));
        RCSOFTCHECK(rcl_publish(&voltaje_publisher, &voltaje_msg, NULL));
    }

    // Leer el encoder y publicar angulo
    as5600_muestra_t muestra_encoder;
    if (as5600_leer(&muestra_encoder) == ESP_OK) { 

        angulo_msg.data = muestra_encoder.grados;
        RCSOFTCHECK(rcl_publish(&angulo_publisher, &angulo_msg, NULL));
        
    }

    ESP_LOGI(TAG, "Angulo Encoder: %.2f grados | Posicion Potenciometro: %.2f", angulo_msg.data, posicion_msg.data);
}

static void micro_ros_task(void *arg){
    while (1) {
        // 1. ESPERA / PING AL AGENTE
        ESP_LOGI(TAG, "Verificando Agente en IP: %s | Puerto: %s", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
        

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;

        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT,
                                             rmw_options));
                                             
#endif

        // Intentar conectar con el agente directamente vía support_init
        rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

        if (rc != RCL_RET_OK) {
            ESP_LOGW(TAG, "No se pudo conectar con el Agente (error %d). Reintentando en 2s...", (int)rc);
            rcl_init_options_fini(&init_options);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue; // Reintentar en el ciclo while
        }

        ESP_LOGI(TAG, "¡Conectado exitosamente al Agente!");

        rcl_node_t node = rcl_get_zero_initialized_node();
        RCCHECK(rclc_node_init_default(&node, "sensores_node", "", &support));
        ESP_LOGI(TAG, "Nodo creado correctamente");

        RCCHECK(rclc_publisher_init_default(
            &posicion_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "posicion"));

        RCCHECK(rclc_publisher_init_default(
            &voltaje_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "voltaje"));

        RCCHECK(rclc_publisher_init_default(
            &angulo_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "angulo"));

        rcl_timer_t timer = rcl_get_zero_initialized_timer();
        RCCHECK(rclc_timer_init_default2(
            &timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS), timer_callback, true));

        rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
        RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
        RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1)));
        RCCHECK(rclc_executor_add_timer(&executor, &timer));

        // Bucle de publicación
        while (1) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Limpieza si sale del bucle
        rcl_publisher_fini(&posicion_publisher, &node);
        rcl_publisher_fini(&voltaje_publisher, &node);
        rcl_publisher_fini(&angulo_publisher, &node);
        rcl_timer_fini(&timer);
        rclc_executor_fini(&executor);
        rcl_node_fini(&node);
        rclc_support_fini(&support);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
    
    // DESACTIVAR POWER SAVE DE WI-FI
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Wi-Fi Power Save desactivado (WIFI_PS_NONE)");

    if (pote_init() == ESP_OK) {
        ESP_LOGI(TAG, "Potenciometro inicializado correctamente");
    } else {
        ESP_LOGE(TAG, "Error inicializando potenciometro");
    }

    if (as5600_init() == ESP_OK) {
        ESP_LOGI(TAG, "AS5600 inicializado correctamente");
    } else {
        ESP_LOGE(TAG, "Error inicializando AS5600");
    }

    xTaskCreate(micro_ros_task, "micro_ros_task",
                MICRO_ROS_APP_STACK, NULL, MICRO_ROS_APP_TASK_PRIO, NULL);
}