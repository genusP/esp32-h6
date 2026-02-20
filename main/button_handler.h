// components/button_handler/button_handler.h
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <iot_button.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // Custom events that extend iot_button.h functionality
    typedef enum
    {
        BUTTON_EVENT_SHORT_PRESS_UP = BUTTON_SINGLE_CLICK,
        BUTTON_EVENT_SHORT_PRESS_DOWN = BUTTON_SINGLE_CLICK,
        BUTTON_EVENT_DOUBLE_PRESS_UP = BUTTON_DOUBLE_CLICK,
        BUTTON_EVENT_DOUBLE_PRESS_DOWN = BUTTON_DOUBLE_CLICK,
        BUTTON_EVENT_LONG_PRESS_UP = BUTTON_LONG_PRESS_START,
        BUTTON_EVENT_LONG_PRESS_DOWN = BUTTON_LONG_PRESS_START,
        BUTTON_EVENT_SIMULTANEOUS_PRESS = 100, // Custom event value
        BUTTON_EVENT_RELEASE = BUTTON_PRESS_UP
    } custom_button_event_t;

    // Button identifier to distinguish between different physical buttons
    typedef enum
    {
        BUTTON_ID_UP = 0,
        BUTTON_ID_DOWN = 1
    } button_id_t;

    // Enhanced callback that includes button identification
    typedef void (*button_callback_t)(button_event_t event, button_id_t button_id, void *user_data);

    void button_handler_init(void);
    void button_handler_set_callback(button_callback_t callback, void *user_data);
    void button_handler_task(void *arg);

#ifdef __cplusplus
}
#endif
