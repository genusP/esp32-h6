#include "position_sensor.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "position_sensor";
static position_config_t position_config = {0};
static bool sensor_initialized = false;

// Состояние пошаговой калибровки
static calibration_step_t current_calibration_step = CALIBRATION_STEP_COMPLETE;
static uint32_t calibration_upper_position = 0;
static uint32_t calibration_lower_position = 0;
static uint32_t calibration_zebra_offset = 100;
static bool calibration_zebra_enabled = false;

// Получение описания шага калибровки
static const char *get_calibration_step_description(calibration_step_t step)
{
    switch (step)
    {
    case CALIBRATION_STEP_UPPER:
        return "Установите жалюзи в верхнее положение и нажмите кнопку";
    case CALIBRATION_STEP_LOWER:
        return "Установите жалюзи в нижнее положение и нажмите кнопку";
    case CALIBRATION_STEP_ZEBRA_OFFSET:
        return "Установите положение смещения для штор зебра и нажмите кнопку";
    case CALIBRATION_STEP_COMPLETE:
        return "Калибровка завершена";
    default:
        return "Неизвестный шаг калибровки";
    }
}

// Внутренние функции для управления питанием
static void position_sensor_power_on(void)
{
    gpio_set_level(POSITION_SENSOR_POWER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(POSITION_SENSOR_STABILIZATION_MS));
}

static void position_sensor_power_off(void)
{
    gpio_set_level(POSITION_SENSOR_POWER_PIN, 0);
}

void position_sensor_init(void)
{
    ESP_LOGI(TAG, "Инициализация датчика положения");

    // Инициализация GPIO для питания потенциометра
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << POSITION_SENSOR_POWER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    // Изначально выключаем питание
    gpio_set_level(POSITION_SENSOR_POWER_PIN, 0);

    // Инициализация ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(POSITION_SENSOR_ADC_CHANNEL, POSITION_SENSOR_ADC_ATTENUATION);

    // Инициализация конфигурации
    position_config.min_position = 100;  // Минимальное значение ADC
    position_config.max_position = 3900; // Максимальное значение ADC
    position_config.current_position = 0;
    position_config.calibrated = false;

    sensor_initialized = true;
    ESP_LOGI(TAG, "Датчик положения инициализирован");
    ESP_LOGI(TAG, "ADC пин: %d, пин питания: %d", POSITION_SENSOR_ADC_PIN, POSITION_SENSOR_POWER_PIN);
}

uint32_t position_sensor_read(void)
{
    if (!sensor_initialized)
    {
        ESP_LOGE(TAG, "Датчик не инициализирован");
        return 0;
    }

    // Включаем питание для измерения
    position_sensor_power_on();

    // Читаем ADC значение
    int raw_value = adc1_get_raw(POSITION_SENSOR_ADC_CHANNEL);

    // Выключаем питание
    position_sensor_power_off();

    if (raw_value < 0)
    {
        ESP_LOGE(TAG, "Ошибка чтения ADC");
        return position_config.current_position;
    }

    uint32_t adc_value = (uint32_t)raw_value;

    // Временная стабилизация значения (усреднение)
    static uint32_t filter_buffer[5] = {0};
    static uint8_t filter_index = 0;

    filter_buffer[filter_index] = adc_value;
    filter_index = (filter_index + 1) % 5;

    uint32_t sum = 0;
    for (int i = 0; i < 5; i++)
    {
        sum += filter_buffer[i];
    }
    adc_value = sum / 5;

    // Ограничиваем диапазон
    if (adc_value < position_config.min_position)
    {
        adc_value = position_config.min_position;
    }
    else if (adc_value > position_config.max_position)
    {
        adc_value = position_config.max_position;
    }

    position_config.current_position = adc_value;

    ESP_LOGD(TAG, "Прочитано значение: %lu", adc_value);

    return adc_value;
}

void position_sensor_set_calibration(uint32_t min_pos, uint32_t max_pos)
{
    if (min_pos >= max_pos)
    {
        ESP_LOGE(TAG, "Некорректные значения калибровки");
        return;
    }

    position_config.min_position = min_pos;
    position_config.max_position = max_pos;
    position_config.calibrated = true;

    ESP_LOGI(TAG, "Калибровка установлена: min=%lu, max=%lu", min_pos, max_pos);
}

void position_sensor_calibrate_start(void)
{
    ESP_LOGI(TAG, "Начало калибровки датчика положения");

    // Измеряем минимальное значение (полностью закрыто)
    ESP_LOGI(TAG, "Установите жалюзи в полностью закрытое положение");
    vTaskDelay(pdMS_TO_TICKS(5000)); // Ожидание 5 секунд

    uint32_t min_val = 0;
    for (int i = 0; i < 10; i++)
    {
        min_val += position_sensor_read();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    min_val /= 10;

    // Измеряем максимальное значение (полностью открыто)
    ESP_LOGI(TAG, "Установите жалюзи в полностью открытое положение");
    vTaskDelay(pdMS_TO_TICKS(5000)); // Ожидание 5 секунд

    uint32_t max_val = 0;
    for (int i = 0; i < 10; i++)
    {
        max_val += position_sensor_read();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    max_val /= 10;

    position_sensor_set_calibration(min_val, max_val);
    ESP_LOGI(TAG, "Калибровка завершена");
}

bool position_sensor_is_calibrated(void)
{
    return position_config.calibrated;
}

float position_sensor_get_percentage(void)
{
    if (!position_config.calibrated)
    {
        ESP_LOGW(TAG, "Датчик не откалиброван");
        return 0.0f;
    }

    uint32_t current = position_sensor_read();

    if (current <= position_config.min_position)
    {
        return 0.0f;
    }

    if (current >= position_config.max_position)
    {
        return 100.0f;
    }

    float percentage = ((float)(current - position_config.min_position) /
                        (float)(position_config.max_position - position_config.min_position)) *
                       100.0f;

    ESP_LOGD(TAG, "Позиция: %.1f%% (%lu)", percentage, current);

    return percentage;
}

// Новые функции для пошаговой калибровки
calibration_step_callback_t position_sensor_start_calibration(void)
{
    ESP_LOGI(TAG, "Starting step-by-step calibration");

    // Инициализация zebra_enabled из Kconfig
#ifdef CONFIG_ZEBRA_BLINDS_SUPPORT
    calibration_zebra_enabled = true;
#else
    calibration_zebra_enabled = false;
#endif

    // Загружаем предыдущие значения калибровки
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("position_sensor", NVS_READONLY, &nvs_handle);

    if (err == ESP_OK)
    {
        err = nvs_get_u32(nvs_handle, "upper_position", &calibration_upper_position);
        if (err != ESP_OK)
            calibration_upper_position = 0;

        err = nvs_get_u32(nvs_handle, "lower_position", &calibration_lower_position);
        if (err != ESP_OK)
            calibration_lower_position = 4095;

        err = nvs_get_u32(nvs_handle, "zebra_offset", &calibration_zebra_offset);
        if (err != ESP_OK)
            calibration_zebra_offset = 100;

        nvs_close(nvs_handle);
    }

    current_calibration_step = CALIBRATION_STEP_UPPER;

    ESP_LOGI(TAG, "Calibration started. Zebra support: %s",
             calibration_zebra_enabled ? "enabled" : "disabled");

    return get_calibration_step_description;
}

calibration_step_t position_sensor_next_calibration_step(void)
{
    if (current_calibration_step == CALIBRATION_STEP_COMPLETE)
    {
        return CALIBRATION_STEP_COMPLETE;
    }

    switch (current_calibration_step)
    {
    case CALIBRATION_STEP_UPPER:
        current_calibration_step = CALIBRATION_STEP_LOWER;
        break;
    case CALIBRATION_STEP_LOWER:
        if (calibration_zebra_enabled)
        {
            current_calibration_step = CALIBRATION_STEP_ZEBRA_OFFSET;
        }
        else
        {
            current_calibration_step = CALIBRATION_STEP_COMPLETE;
        }
        break;
    case CALIBRATION_STEP_ZEBRA_OFFSET:
        current_calibration_step = CALIBRATION_STEP_COMPLETE;
        break;
    default:
        current_calibration_step = CALIBRATION_STEP_COMPLETE;
        break;
    }

    ESP_LOGI(TAG, "Next calibration step: %d", current_calibration_step);
    return current_calibration_step;
}

void position_sensor_save_calibration_step(uint32_t position)
{
    switch (current_calibration_step)
    {
    case CALIBRATION_STEP_UPPER:
        calibration_upper_position = position;
        ESP_LOGI(TAG, "Upper position saved: %lu", position);
        break;
    case CALIBRATION_STEP_LOWER:
        calibration_lower_position = position;
        ESP_LOGI(TAG, "Lower position saved: %lu", position);

        // Устанавливаем калибровку в position_sensor
        if (calibration_upper_position < calibration_lower_position)
        {
            position_sensor_set_calibration(calibration_upper_position, calibration_lower_position);
        }
        break;
    case CALIBRATION_STEP_ZEBRA_OFFSET:
        calibration_zebra_offset = position;
        ESP_LOGI(TAG, "Zebra offset saved: %lu", position);
        break;
    case CALIBRATION_STEP_COMPLETE:
        // Сохраняем все данные в NVS
        position_sensor_save_calibration_data();
        break;
    }
}

uint32_t position_sensor_get_zebra_offset(void)
{
    return calibration_zebra_offset;
}

uint32_t position_sensor_get_min_position(void)
{
    return position_config.min_position;
}

uint32_t position_sensor_get_max_position(void)
{
    return position_config.max_position;
}

static void position_sensor_save_calibration_data(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("position_sensor", NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_u32(nvs_handle, "upper_position", calibration_upper_position);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error saving upper position: %s", esp_err_to_name(err));

    err = nvs_set_u32(nvs_handle, "lower_position", calibration_lower_position);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error saving lower position: %s", esp_err_to_name(err));

    err = nvs_set_u32(nvs_handle, "zebra_offset", calibration_zebra_offset);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error saving zebra offset: %s", esp_err_to_name(err));

    err = nvs_set_u8(nvs_handle, "zebra_enabled", calibration_zebra_enabled ? 1 : 0);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error saving zebra enabled: %s", esp_err_to_name(err));

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));

    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "Calibration data saved to NVS");
}
