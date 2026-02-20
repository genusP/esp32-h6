#include <iot_button.h>
#include "button_handler.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "sdkconfig.h"

static const char *TAG = "button_handler";

static button_callback_t g_user_callback = NULL;
static void *g_user_data = NULL;
static button_handle_t g_button_up = NULL;
static button_handle_t g_button_down = NULL;
static QueueHandle_t g_button_event_queue = NULL;

// Внутренняя структура для событий кнопок
typedef struct
{
    button_event_t event;
    button_id_t button_id;
} button_event_msg_t;

// Обработчики событий для кнопки вверх
static void button_up_single_press_cb(void *button_handle, void *usr_data)
{
    button_event_msg_t msg = {.event = BUTTON_SINGLE_CLICK, .button_id = BUTTON_ID_UP};
    xQueueSendFromISR(g_button_event_queue, &msg, NULL);
}

static void button_up_double_press_cb(void *button_handle, void *usr_data)
{
    button_event_msg_t msg = {.event = BUTTON_DOUBLE_CLICK, .button_id = BUTTON_ID_UP};
    xQueueSendFromISR(g_button_event_queue, &msg, NULL);
}

static void button_up_long_press_cb(void *button_handle, void *usr_data)
{
    button_event_msg_t msg = {.event = BUTTON_LONG_PRESS_START, .button_id = BUTTON_ID_UP};
    xQueueSendFromISR(g_button_event_queue, &msg, NULL);
}

// Обработчики событий для кнопки вниз
static void button_down_single_press_cb(void *button_handle, void *usr_data)
{
    button_event_msg_t msg = {.event = BUTTON_SINGLE_CLICK, .button_id = BUTTON_ID_DOWN};
    xQueueSendFromISR(g_button_event_queue, &msg, NULL);
}

static void button_down_double_press_cb(void *button_handle, void *usr_data)
{
    button_event_msg_t msg = {.event = BUTTON_DOUBLE_CLICK, .button_id = BUTTON_ID_DOWN};
    xQueueSendFromISR(g_button_event_queue, &msg, NULL);
}

static void button_down_long_press_cb(void *button_handle, void *usr_data)
{
    button_event_msg_t msg = {.event = BUTTON_LONG_PRESS_START, .button_id = BUTTON_ID_DOWN};
    xQueueSendFromISR(g_button_event_queue, &msg, NULL);
}

// Функция обработки одновременного нажатия
static void check_simultaneous_press_task(void *arg)
{
    bool both_pressed = false;
    TickType_t press_start_time = 0;
    const TickType_t simultaneous_threshold = pdMS_TO_TICKS(100); // 100ms для одновременного нажатия

    while (true)
    {
        bool up_pressed = (iot_button_get_event(g_button_up) == BUTTON_PRESS_DOWN);
        bool down_pressed = (iot_button_get_event(g_button_down) == BUTTON_PRESS_DOWN);

        if (up_pressed && down_pressed && !both_pressed)
        {
            // Обе кнопки нажаты
            both_pressed = true;
            press_start_time = xTaskGetTickCount();
        }
        else if (!up_pressed || !down_pressed)
        {
            // Хотя бы одна кнопка отпущена
            if (both_pressed)
            {
                TickType_t press_duration = xTaskGetTickCount() - press_start_time;
                if (press_duration < simultaneous_threshold)
                {
                    // Кнопки были нажаты одновременно в течение короткого времени
                    button_event_msg_t msg = {.event = (button_event_t)BUTTON_EVENT_SIMULTANEOUS_PRESS};
                    xQueueSend(g_button_event_queue, &msg, 0);
                }
            }
            both_pressed = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Проверяем каждые 10ms
    }
}

void button_handler_init(void)
{
    ESP_LOGI(TAG, "Initializing button handler");

    // Создаем очередь для событий кнопок
    g_button_event_queue = xQueueCreate(10, sizeof(button_event_msg_t));
    if (g_button_event_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create button event queue");
        return;
    }

    // Инициализация кнопки вверх
    button_config_t up_button_config = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_MS,
        .short_press_time = 50,
        .gpio_button_config = {
            .gpio_num = CONFIG_BUTTON_UP_PIN,
            .active_level = 0, // Активный низкий уровень (кнопка замыкает на GND)
        },
    };

    g_button_up = iot_button_create(&up_button_config);
    if (g_button_up == NULL)
    {
        ESP_LOGE(TAG, "Failed to create up button");
        return;
    }

    // Инициализация кнопки вниз
    button_config_t down_button_config = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_MS,
        .short_press_time = 50,
        .gpio_button_config = {
            .gpio_num = CONFIG_BUTTON_DOWN_PIN,
            .active_level = 0, // Активный низкий уровень
        },
    };

    g_button_down = iot_button_create(&down_button_config);
    if (g_button_down == NULL)
    {
        ESP_LOGE(TAG, "Failed to create down button");
        return;
    }

    // Регистрация обработчиков для кнопки вверх
    iot_button_register_cb(g_button_up, BUTTON_SINGLE_CLICK, button_up_single_press_cb, NULL);
    iot_button_register_cb(g_button_up, BUTTON_DOUBLE_CLICK, button_up_double_press_cb, NULL);
    iot_button_register_cb(g_button_up, BUTTON_LONG_PRESS_START, button_up_long_press_cb, NULL);

    // Регистрация обработчиков для кнопки вниз
    iot_button_register_cb(g_button_down, BUTTON_SINGLE_CLICK, button_down_single_press_cb, NULL);
    iot_button_register_cb(g_button_down, BUTTON_DOUBLE_CLICK, button_down_double_press_cb, NULL);
    iot_button_register_cb(g_button_down, BUTTON_LONG_PRESS_START, button_down_long_press_cb, NULL);

    // Создаем задачу для определения одновременного нажатия
    xTaskCreate(check_simultaneous_press_task, "simultaneous_press", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Button handler initialized successfully");
}

void button_handler_set_callback(button_callback_t callback, void *user_data)
{
    g_user_callback = callback;
    g_user_data = user_data;
}

void button_handler_task(void *arg)
{
    button_event_msg_t msg;

    ESP_LOGI(TAG, "Button handler task started");

    while (true)
    {
        // Ожидаем события от кнопок
        if (xQueueReceive(g_button_event_queue, &msg, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGD(TAG, "Button event received: %d", msg.event);

            // Вызываем пользовательский callback если он установлен
            if (g_user_callback != NULL)
            {
                g_user_callback(msg.event, msg.button_id, g_user_data);
            }
        }
    }
}
