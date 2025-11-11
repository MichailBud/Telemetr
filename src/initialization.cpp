
#include "../inc/initialization.hpp"



void clock_setup(void) {   
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    // Включаем тактирование для GPIOA, GPIOB и USART1, USART2
    rcc_periph_clock_enable(RCC_GPIOD); // Включаем группу портов ввода вывода D
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART3);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_SPI2);
}

void gpio_setup(void) {
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15); // Активируем 15 вывод в режиме выхода

    // PA0 и PA1 как выходы (Output)
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);
    
    // PA8 как вход (Input)
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO8);
    
    // PA9 (USART1_TX) как альтернативная функция
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9); // AF7 для USART1 на этих пинах
    
    // USART3: PB10 (TX), PB11 (RX) - альтернативная функция
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
    gpio_set_af(GPIOB, GPIO_AF7, GPIO10 | GPIO11);
    
    // PA2 (USART2_TX) как альтернативная функция
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2); // AF7 для USART2 на этих пинах
    
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO3); 

    // SPI2 на выводах PB13, PB14, PB15
    // PB13 - SCK, PB14 - MISO, PB15 - MOSI
    
    // Настройка выводов SPI в альтернативный режим
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, 
                   GPIO13 | GPIO14 | GPIO15);
    
    // Установка альтернативной функции SPI2 (AF5)
    gpio_set_af(GPIOB, GPIO_AF5, GPIO13 | GPIO14 | GPIO15);

    // Дополнительно: вывод для NSS (выбор ведомого) - PB12
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
    gpio_set(GPIOB, GPIO12); // Установить высокий уровень (ведомый не выбран)
}

void usart_setup(void) {
    // Настройка USART3
    usart_set_baudrate(USART3, 115200);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    
    // Включаем USART3
    usart_enable(USART3);
    
    // Настройка USART2
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    // Активируем прерывания по приёму данных в UART2
    usart_enable_rx_interrupt(USART2);
    
    // Включаем USART2
    usart_enable(USART2);
}
 
void spi2_setup(void) {  // Настройка SPI2
    // Сброс и инициализация SPI2
    spi_disable(SPI2);
    
    // Базовая настройка SPI
    spi_init_master(SPI2,
                   SPI_CR1_BAUDRATE_FPCLK_DIV_256,    // Предделитель (42MHz/32 = ~1.3MHz)
                   SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,  // Полярность: низкий уровень в idle
                   SPI_CR1_CPHA_CLK_TRANSITION_1,    // Фаза: данные захватываются по первому фронту
                   SPI_CR1_DFF_8BIT,                 // 8-битный формат данных
                   SPI_CR1_MSBFIRST);                // Старший бит первый
    
    // Дополнительные настройки
    spi_set_full_duplex_mode(SPI2);                  // Полнодуплексный режим
    spi_enable_software_slave_management(SPI2);      // Программное управление NSS
    spi_set_nss_high(SPI2);                          // NSS всегда высокий
    
    // Включаем SPI
    spi_enable(SPI2);

    // Включаем прерывания
    spi_enable_rx_buffer_not_empty_interrupt(SPI2);
}

void nvic_setup(void) {
    // Включаем прерывание USART2 в контроллере прерываний (NVIC)
    nvic_enable_irq(NVIC_USART2_IRQ);
    // Устанавливаем приоритет прерывания (меньше число - выше приоритет)
    nvic_set_priority(NVIC_USART2_IRQ, 0);

    // Включаем прерывание SPI2 (с более низким приоритетом)
    nvic_enable_irq(NVIC_SPI2_IRQ);
    nvic_set_priority(NVIC_SPI2_IRQ, 1);
}



