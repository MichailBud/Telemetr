#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>

#include <libopencm3/cm3/nvic.h>

void clock_setup(void); // Конфигурация тактового сигнала

void gpio_setup(void); // Конфигурация GPIO

void usart_setup(void); // Конфигурация USART
 
void spi2_setup(void); // Конфигурация SPI 2
 
void nvic_setup(void); // Конфигурация NVIC

void usart2_isr(void); // Обработчик прерывания USART2


void spi2_isr(void); // Обработчик прерывания SPI2