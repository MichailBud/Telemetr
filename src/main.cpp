#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>

#include "../inc/functions.hpp"
#include "../inc/initialization.hpp"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

enum State {
    idle,
    data_receive,
    finish
};

uint8_t data_buffer;	
Circular_buffer b;
Circular_buffer b_spi;

volatile uint32_t tiks = 0;

// Функция задержки в микросекундах
void delay_us(uint32_t us) {
    uint32_t start = systick_get_value();
    // Расчет тактов для указанного времени (72 тактов = 1 мкс при 72 МГц)
    uint32_t ticks = us * 72;
    while ((start - systick_get_value()) < ticks) {
        // Ждем, пока не пройдет нужное количество тактов
    }
}

// Функция задержки в миллисекундах
void delay_ms(uint32_t ms) {
    while (ms--) {
        delay_us(1000); // 1000 мкс = 1 мс
    }
}

void sys_tick_handler(void) { // функция обработчик-прерываний systick
    tiks++;
}

void systick_setup(void) {
    systick_set_frequency(1000, 72000000);
    systick_counter_enable();
    systick_interrupt_enable();
}

void config_radiomodule(void) { // Конфигурация радиомодуля (100% рабочая)
    gpio_set(GPIOA, GPIO0);
    gpio_set(GPIOA, GPIO1);
    
    delay_ms(200);

    uint8_t str_tx[] = {0xC0, 0x00, 0x00, 0x1A, 0x06, 0x44}; // Настройка для радиомодуля
    uart2_write(str_tx, 6); // Записываем конфигурацию в радиомодуль

    delay_ms(200);

    gpio_clear(GPIOA, GPIO0);
    gpio_clear(GPIOA, GPIO1);

    delay_ms(200);
}

void usart2_isr(void) {
    if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
        
        // Чтение принятых данных
        uint8_t data = usart_recv(USART2);
        
        // Отправляем данные в USART3 (эхо)
        usart_send_blocking(USART3, data);
        
        // Сохраняем данные в буфер
        b.put(data);
    }
}

// Обработчик прерывания SPI2
void spi2_isr(void) {
    // Проверяем флаг приема данных
    if (SPI_SR(SPI2) & SPI_SR_RXNE) {
        uint8_t received_data = spi_read(SPI2);
        // Отправляем эхо в USART2 для отладки
        usart_send_blocking(USART2, received_data);
        b_spi.put(received_data);
    }
}


int main(void) {
    clock_setup();
    gpio_setup();
    usart_setup();
    spi2_setup();
    systick_setup();
    nvic_setup();

    config_radiomodule();
    
    State state = idle;
    
    while (1) {
        // Индикация состояния PA8
        if (gpio_get(GPIOA, GPIO8)) {
            gpio_set(GPIOD, GPIO15);
        } else {
            gpio_clear(GPIOD, GPIO15);
        }
        
        // Отправка тестового сообщения
        uint8_t str_tx[] = {'H', 'E', 'L', 'L', 'O'};
        uart2_write(str_tx, 5);
        delay_ms(1000);

        // Обработка данных из SPI (раскомментируйте при необходимости)
        /*
        spi2_select_slave();
        if(!b_spi.empty()) {
            uint8_t data = b_spi.get();
            switch (state) {
                case idle:
                    usart_send_blocking(USART2, '2'); 
                    if (data == 36) { // '$'
                        state = data_receive;
                    }
                    break;
                case data_receive:
                    usart_send_blocking(USART2, '3'); 
                    if (data == 59) { // ';'
                        state = finish;
                    } else {
                        // usart_send_blocking(USART2, data); 
                    }
                    break;
                case finish:
                    usart_send_blocking(USART2, '4'); 
                    usart_send_blocking(USART2, '\n'); 
                    spi2_deselect_slave();
                    state = idle;
                    delay_ms(1000);
                    break;
                default:
                    state = idle;
                    break;
            }
        }
        */
        
        // Отправляем данные записанные в буфер USART2
        while (!b.empty()) {
            usart_send_blocking(USART3, b.get()); 
        }
    }
    
    return 0;
}
