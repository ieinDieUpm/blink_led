/**
 * @file port_led.c
 * @author Román Cárdenas (r.cardenas@upm.es)
 * @brief Port layer for the LED emulation in the native platform.
 * @date 01-01-2024
 */
#include "port_system.h"
#include "port_led.h"
#include <stdio.h>
#include <stdint.h>

/* Enumerations ------------------------------------------------------------------*/
/**
 * @brief Enumerates the possible configurations for the LED GPIO.
 *
 */
typedef enum
{
    UNCONFIGURED = -1, /*!< The LED GPIO is not configured. */
    OUTPUT = 0,        /*!< The LED GPIO is configured as output. */
    INPUT = 1,         /*!< The LED GPIO is configured as input. */
} LED_GPIO_CONFIG;

/* Structures ------------------------------------------------------------------*/
/**
 * @brief Structure to store the LED GPIO configuration and status.
 *
 */
typedef struct
{
    LED_GPIO_CONFIG config; /*!< The LED GPIO configuration. */
    bool status;            /*!< The LED GPIO status. */
} led_gpio_t;

/* Variables ------------------------------------------------------------------*/
/**
 * @brief LED GPIO configuration and status.
 *
 */
static led_gpio_t led_gpio = {
    .config = UNCONFIGURED, /*!< LED configuration. */
    .status = false,        /*!< LED status. */
};

void port_led_gpio_setup()
{
    if (led_gpio.config == UNCONFIGURED)
    {
        printf("[%lld] LED clock enabled\n", (long long)port_system_get_millis());
    }
    if (led_gpio.config != OUTPUT)
    {
        led_gpio.config = OUTPUT;
        printf("[%lld] LED pin mode configured mode as output\n", (long long)port_system_get_millis());
        printf("[%lld] LED pin pull up/down configuration as no PUPD\n", (long long)port_system_get_millis());
    }
    fflush(stdout);
}

bool port_led_get_status()
{
    return led_gpio.status;
}

void port_led_on()
{
    if (led_gpio.config == OUTPUT)
    {
        led_gpio.status = true;
        printf("[%lld] LED ON\n", (long long)port_system_get_millis());
        fflush(stdout);
    }
    else
    {
        printf("[%lld] LED not configured as output\n", (long long)port_system_get_millis());
        fflush(stdout);
    }
}

void port_led_off()
{
    if (led_gpio.config == OUTPUT)
    {
        printf("[%lld] LED OFF\n", (long long)port_system_get_millis());
        fflush(stdout);
        led_gpio.status = false;
    }
    else
    {
        printf("[%lld] LED not configured as output\n", (long long)port_system_get_millis());
        fflush(stdout);
    }
}

void port_led_toggle()
{
    (port_led_get_status()) ? port_led_off() : port_led_on();
}
