#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "E32_dop.cpp"

void clock_setup(void)
{
    // Включаем тактирование для GPIOA, GPIOB и USART1, USART2
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_USART2);
}

void gpio_setup(void)
{
    // PA0 и PA1 как выходы (Output)
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);
    
    // PA8 как вход (Input)
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO8);
    
    // PA9 (USART1_TX) как альтернативная функция
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9); // AF7 для USART1 на этих пинах
    
    // PA10 (USART1_RX) как альтернативная функция
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO10); // AF7 для USART1 на этих пинах
    
    // PA2 (USART2_TX) как альтернативная функция
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2); // AF7 для USART2 на этих пинах
    
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO3); 
}

void usart_setup(void)
{
    // Настройка USART1
    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    
    // Включаем USART1
    usart_enable(USART1);
    
    // Настройка USART2
    usart_set_baudrate(USART2, 9600);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    
    // Включаем USART2
    usart_enable(USART2);
}

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();

    uint8_t str[6];
	uint8_t byte_data;

	gpio_set(GPIOB, GPIO0);
	gpio_set(GPIOB, GPIO1);

	uint8_t str_tx[]={0xC0,0x00,0x00,0x1A,0x06,0x44};
	uart2_write(str_tx,6);

	uint8_t str2_tx[]={0xC1, 0xC1, 0xC1};
	uart2_write(str2_tx,3);
    
    gpio_set(GPIOA, GPIO0);    // Установить PA0 в HIGH
    gpio_clear(GPIOA, GPIO1);  // Установить PA1 в LOW
    

    while (1) {
        
        usart_send_blocking(USART1, 'A');

        usart_send_blocking(USART2, 'B');
    }
    
    return 0;
}
