/**
 * @file port_led.c
 * @author Josué Pagán (j.pagan@upm.es)
 * @brief Port layer for the LED of the STM32F4 Nucleo board.
 * @date 01-03-2024
 */
/* Standard C includes */
#include <stdio.h>

/* HW dependent includes */
#include "stm32f4xx.h"
#include "port_led.h"
#include "port_system.h"

// HW Nucleo-STM32F446RE:
#define LD2_PIN 5           /*!< GPIO pin of the LED2 in the Nucleo board */
#define LD2_GPIO_PORT GPIOA /*!< GPIO port of the LED2 in the Nucleo board */

#define MODER_LD2_MASK (GPIO_MODER_MODE0 << LD2_PIN * 2) /*!< Mask for the LED2 in the MODE Register */
#define PUPDR_LD2_MASK (GPIO_PUPDR_PUPD0 << LD2_PIN * 2) /*!< Mask for the LED2 in the PUPD Register */

#define MODER_LD2_AS_OUTPUT (GPIO_MODE_OUT << LD2_PIN * 2)     /*!< Output mode for the LED2 in the MODE Register */
#define PUPDR_LD2_AS_NOPUPD (GPIO_PUPDR_NOPULL << LD2_PIN * 2) /*!< No push/pull configuration for the LED2 in the MODE Register */

#define IDR_LD2_MASK (GPIO_IDR_ID0 << LD2_PIN) /*!< Mask for the LED2 in the Input Data Register */
#define ODR_LD2_MASK (GPIO_ODR_OD0 << LD2_PIN) /*!< Mask for the LED2 in the Output Data Register */

void port_led_gpio_setup(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;     // Enable peripheral clock
    LD2_GPIO_PORT->MODER &= ~MODER_LD2_MASK; // Clean the registers
    LD2_GPIO_PORT->PUPDR &= ~PUPDR_LD2_MASK;
    LD2_GPIO_PORT->MODER |= MODER_LD2_AS_OUTPUT; // Set the corresponding configuration
    LD2_GPIO_PORT->PUPDR |= PUPDR_LD2_AS_NOPUPD;
}

bool port_led_get_status(void)
{
    return (LD2_GPIO_PORT->IDR & IDR_LD2_MASK) != 0;
}

void port_led_on(void)
{
    LD2_GPIO_PORT->ODR |= ODR_LD2_MASK;
    printf("[%ld] LED ON\n", port_system_get_millis()); // For debug purposes only
}

void port_led_off(void)
{
    LD2_GPIO_PORT->ODR &= ~ODR_LD2_MASK;
    printf("[%ld] LED OFF\n", port_system_get_millis()); // For debug purposes only
}

void port_led_toggle(void)
{
    LD2_GPIO_PORT->ODR ^= ODR_LD2_MASK;

    if (port_led_get_status())
    {
        printf("[%ld] LED ON\n", port_system_get_millis()); // For debug purposes only
    }
    else
    {
        printf("[%ld] LED OFF\n", port_system_get_millis()); // For debug purposes only
    }
}
