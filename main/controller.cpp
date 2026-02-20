#include "controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "motor_control.h"

static const char *TAG = "controller";

// Кэш конфигурации
static config_t g_config = {0};

// Флаги состояния
static bool g_button_held = false;
static calibration_step_callback_t g_calibration_callback = NULL;

// Задача для периодической проверки границ
static TaskHandle_t g_boundary_check_task = NULL;

// Объявления функций
static void controller_button_callback(button_event_t event, button_id_t button_id, void *user_data);
static void controller_move_to_percentage(float percentage);
static void controller_handle_zebra_offset(void);
static bool controller_check_boundaries_and_stop(void);

void controller_init(void)
{
    ESP_LOGI(TAG, "Initializing controller");

    // Инициализация подсистем
    motor_control_init();
    position_sensor_init();
    button_handler_init();

    // Установка callback для кнопок
    button_handler_set_callback(controller_button_callback, NULL);

    // Установка начального состояния
    g_config.state = IDLE;
    g_config.auto_calibrate = !position_sensor_is_calibrated();

    ESP_LOGI(TAG, "Controller initialized. Calibrated: %s",
             position_sensor_is_calibrated() ? "Yes" : "No");
}

void controller_move_to_position(uint32_t position)
{
    if (g_config.state == CALIBRATING)
    {
        ESP_LOGW(TAG, "Cannot move to position during calibration");
        return;
    }

    uint32_t current_pos = position_sensor_read();

    if (current_pos == position)
    {
        ESP_LOGI(TAG, "Already at target position: %lu", position);
        return;
    }

    ESP_LOGI(TAG, "Moving from position %lu to %lu", current_pos, position);

    // Определяем направление на основе текущей и целевой позиций
    motor_direction_t direction = (position > current_pos
                                   ? MOTOR_DIR_DOWN;
                                   : MOTOR_DIR_UP);

    // Устанавливаем направление мотора
    motor_set_direction(direction);

    // Вычисляем количество шагов (упрощенный подход)
    uint32_t position_diff = (position > current_pos) ? (position - current_pos) : (current_pos - position);

    // Устанавливаем скорость по умолчанию
    uint32_t speed = CONFIG_MOTOR_DEFAULT_SPEED;
    motor_set_speed(speed);

    // Запускаем движение мотора
    motor_step(position_diff);

    // Обновляем состояние
    if (direction == MOTOR_DIR_UP)
    {
        g_config.state = MOVING_UP;
    }
    else
    {
        g_config.state = MOVING_DOWN;
    }

    g_config.position.current_position = position;

    // Проверяем границы сразу после запуска движения
    controller_check_boundaries_and_stop();
}

void controller_move_up(void)
{
    if (g_config.state == CALIBRATING)
    {
        ESP_LOGW(TAG, "Cannot move up during calibration");
        return;
    }

    ESP_LOGI(TAG, "Moving up");
    motor_set_direction(MOTOR_DIR_UP);

    // Устанавливаем скорость
    uint32_t speed = CONFIG_MOTOR_DEFAULT_SPEED;
    motor_set_speed(speed);

    // Движение вверх - можно использовать большое количество шагов для непрерывного движения
    motor_step(UINT32_MAX);

    g_config.state = MOVING_UP;
}

void controller_move_down(void)
{
    if (g_config.state == CALIBRATING)
    {
        ESP_LOGW(TAG, "Cannot move down during calibration");
        return;
    }

    ESP_LOGI(TAG, "Moving down");
    motor_set_direction(MOTOR_DIR_DOWN);

    // Устанавливаем скорость
    uint32_t speed = CONFIG_MOTOR_DEFAULT_SPEED;
    motor_set_speed(speed);

    // Движение вниз - можно использовать большое количество шагов для непрерывного движения
    motor_step(UINT32_MAX);

    g_config.state = MOVING_DOWN;
}

void controller_stop(void)
{
    ESP_LOGI(TAG, "Stopping motor");

    // Проверяем, движется ли мотор
    if (motor_is_moving())
    {
        ESP_LOGI(TAG, "Motor is moving, stopping");
        motor_stop();
    }
    else
    {
        ESP_LOGD(TAG, "Motor already stopped");
    }

    g_config.state = IDLE;
    g_button_held = false;
}

void controller_calibrate(void)
{
    ESP_LOGI(TAG, "Starting calibration mode");
    g_config.state = CALIBRATING;
    controller_stop();

    // Получаем callback для описания шагов калибровки
    g_calibration_callback = position_sensor_start_calibration();

    if (g_calibration_callback)
    {
        calibration_step_t current_step = position_sensor_next_calibration_step();
        const char *description = g_calibration_callback(current_step);
        ESP_LOGI(TAG, "Calibration step %d: %s", current_step, description);
    }
}

void controller_goto_top(void)
{
    if (position_sensor_is_calibrated())
    {
        // Получаем реальную минимальную позицию из position_sensor
        uint32_t min_pos = position_sensor_get_min_position();
        ESP_LOGI(TAG, "Moving to top position: %lu", min_pos);
        controller_move_to_position(min_pos);
    }
    else
    {
        ESP_LOGW(TAG, "Not calibrated, cannot goto top");
    }
}

void controller_goto_bottom(void)
{
    if (position_sensor_is_calibrated())
    {
        // Получаем реальную максимальную позицию из position_sensor
        uint32_t max_pos = position_sensor_get_max_position();
        ESP_LOGI(TAG, "Moving to bottom position: %lu", max_pos);
        controller_move_to_position(max_pos);
    }
    else
    {
        ESP_LOGW(TAG, "Not calibrated, cannot goto bottom");
    }
}

state_t controller_get_state(void)
{
    return g_config.state;
}

bool controller_is_moving(void)
{
    return motor_is_moving();
}

void controller_set_position_percentage(float percentage)
{
    if (percentage < 0.0f)
        percentage = 0.0f;
    if (percentage > 100.0f)
        percentage = 100.0f;

    if (position_sensor_is_calibrated())
    {
        uint32_t current_pos = position_sensor_read();
        float current_percentage = position_sensor_get_percentage();

        // Получаем реальные границы из position_sensor
        uint32_t min_pos = position_sensor_get_min_position();
        uint32_t max_pos = position_sensor_get_max_position();
        uint32_t range = max_pos - min_pos;
        uint32_t target_position = min_pos + (uint32_t)(range * percentage / 100.0f);

        ESP_LOGI(TAG, "Setting position %.1f%% (ADC: %lu, range: %lu-%lu)",
                 percentage, target_position, min_pos, max_pos);

        controller_move_to_position(target_position);
    }
    else
    {
        ESP_LOGW(TAG, "Not calibrated, cannot set position percentage");
    }
}

static void controller_button_callback(button_event_t event, button_id_t button_id, void *user_data)
{
    ESP_LOGI(TAG, "Button event: %d, button_id: %d", event, button_id);

    switch (event)
    {
    case BUTTON_EVENT_SIMULTANEOUS_PRESS:
        if (g_config.state == CALIBRATING)
        {
            // Выход из режима калибровки
            ESP_LOGI(TAG, "Exiting calibration mode");
            g_config.state = IDLE;
            g_calibration_callback = NULL;
            controller_stop();
        }
        else
        {
            // Вход в режим калибровки
            controller_calibrate();
        }
        break;

    case BUTTON_SINGLE_CLICK:
        if (g_config.state == CALIBRATING && g_calibration_callback)
        {
            // Сохраняем текущую позицию для шага калибровки
            uint32_t current_position = position_sensor_read();
            position_sensor_save_calibration_step(current_position);

            // Переходим к следующему шагу
            calibration_step_t next_step = position_sensor_next_calibration_step();

            if (next_step == CALIBRATION_STEP_COMPLETE)
            {
                // Калибровка завершена
                ESP_LOGI(TAG, "Calibration completed");
                g_config.state = IDLE;
                g_calibration_callback = NULL;
                controller_stop();
            }
            else
            {
                // Показываем описание следующего шага
                const char *description = g_calibration_callback(next_step);
                ESP_LOGI(TAG, "Calibration step %d: %s", next_step, description);
            }
        }
        else
        {
            // Одиночное нажатие - переход в крайнее положение
            if (button_id == BUTTON_ID_UP)
            {
                controller_goto_top();
            }
            else if (button_id == BUTTON_ID_DOWN)
            {
                controller_goto_bottom();
            }
        }
        break;

    case BUTTON_DOUBLE_CLICK:
        if (g_config.state != CALIBRATING)
        {
            // Двойное нажатие
#ifdef CONFIG_ZEBRA_BLINDS_SUPPORT
            if (position_sensor_is_calibrated())
            {
                uint32_t zebra_offset = position_sensor_get_zebra_offset();
                if (zebra_offset > 0)
                {
                    // Выполнение откалиброванного смещения для штор зебра
                    controller_handle_zebra_offset();
                    break;
                }
            }
#else
            // Переход на позицию 50%
            controller_set_position_percentage(50.0f);
#endif
        }
        break;

    case BUTTON_LONG_PRESS_START:
        if (g_config.state != CALIBRATING)
        {
            // Движение пока кнопка удерживается
            g_button_held = true;
            if (button_id == BUTTON_ID_UP)
            {
                controller_move_up();
            }
            else if (button_id == BUTTON_ID_DOWN)
            {
                controller_move_down();
            }

            // Запускаем задачу проверки границ
            if (g_boundary_check_task == NULL)
            {
                xTaskCreate(boundary_check_task, "boundary_check", 2048, NULL, 10, &g_boundary_check_task);
            }
        }
        break;

    case BUTTON_PRESS_UP:
        if (g_button_held)
        {
            // Остановка движения при отпускании кнопки
            controller_stop();
        }
        break;

    default:
        break;
    }
}

#ifdef CONFIG_ZEBRA_BLINDS_SUPPORT
static void controller_handle_zebra_offset(void)
{
    if (!position_sensor_is_calibrated())
    {
        ESP_LOGW(TAG, "Not calibrated, cannot handle zebra offset");
        return;
    }

    uint32_t zebra_offset = position_sensor_get_zebra_offset();
    if (zebra_offset == 0)
    {
        ESP_LOGW(TAG, "Zebra offset not configured");
        return;
    }

    // Вычисляем целевую позицию с учетом смещения
    uint32_t current_pos = position_sensor_read();
    uint32_t target_pos;

    // Получаем границы из position_sensor (нужно будет добавить функции)
    uint32_t min_pos = 0;    // position_sensor_get_min_position();
    uint32_t max_pos = 4095; // position_sensor_get_max_position();

    // Определяем direction на основе текущей позиции
    if (current_pos <= min_pos + zebra_offset)
    {
        // Двигаемся вниз на смещение
        target_pos = current_pos + zebra_offset;
    }
    else if (current_pos >= max_pos - zebra_offset)
    {
        // Двигаемся вверх на смещение
        target_pos = current_pos - zebra_offset;
    }
    else
    {
        // Чередуем направление движения
        static bool last_direction_up = false;
        if (last_direction_up)
        {
            target_pos = current_pos - zebra_offset;
        }
        else
        {
            target_pos = current_pos + zebra_offset;
        }
        last_direction_up = !last_direction_up;
    }

    // Ограничиваем целевую позицию в пределах диапазона
    if (target_pos < min_pos)
    {
        target_pos = min_pos;
    }
    else if (target_pos > max_pos)
    {
        target_pos = max_pos;
    }

    ESP_LOGI(TAG, "Moving to zebra offset position: %lu", target_pos);
    controller_move_to_position(target_pos);
}
#endif

// Функция проверки границ и автоматической остановки
static bool controller_check_boundaries_and_stop(void)
{
    if (!position_sensor_is_calibrated())
    {
        return false; // Нет калибровки - не проверяем границы
    }

    uint32_t current_pos = position_sensor_read();
    uint32_t min_pos = position_sensor_get_min_position();
    uint32_t max_pos = position_sensor_get_max_position();

    // Проверяем достижение границы
    if (current_pos <= min_pos || current_pos >= max_pos)
    {
        ESP_LOGI(TAG, "%s boundary reached: %lu", current_pos >= max_pos ? "Lower" : "current_pos >= max_pos", current_pos);
        controller_stop();
        return true;
    }

    return false;
}

// Периодическая задача для проверки границ при удержании кнопки
static void boundary_check_task(void *parameter)
{
    while (g_button_held && motor_is_moving())
    {
        // Проверяем границы каждые 100 мс
        if (controller_check_boundaries_and_stop())
        {
            break; // Вышли из цикла, так как достигли границы
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Задача завершается, когда кнопка отпущена или мотор остановлен
    g_boundary_check_task = NULL;
    vTaskDelete(NULL);
}
