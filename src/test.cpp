#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>

enum State{
    idle,
    data_receive,
    finish
};

uint8_t data_buffer;	

volatile uint32_t tiks = 0;

//Функия задержки в микросекундах
void delay_us(uint32_t us) {
	uint32_t start = systick_get_value();
	// Расчет тактов для указанного времени (72 тактов = 1 мкс при 72 МГц)
	uint32_t ticks = us * 72;
	while ((start - systick_get_value()) < ticks) {
		// Ждем, пока не пройдет нужное количество тактов
		}
	}

//  Функция задержки в миллисекундах
void delay_ms(uint32_t ms) {
	while (ms--) {
		delay_us(1000); // 1000 мкс = 1 мс
	}
}

void sys_tick_handler(void){ //функция обработчик-прерываний systick
	tiks++;
}

void systick_setup(void){
systick_set_frequency(1000,72'000'000);
systick_counter_enable();
systick_interrupt_enable();

}



void clock_setup(void)
{   
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    // Включаем тактирование для GPIOA, GPIOB и USART1, USART2
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART3);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_SPI2);
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

void usart_setup(void)
{
    // Настройка USART1
    usart_set_baudrate(USART3, 9600);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    
    // Включаем USART1
    usart_enable(USART3);
    
    // Настройка USART2
    usart_set_baudrate(USART2, 9600);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    //активируем перрывания по приёму данных в UART2
  	usart_enable_rx_interrupt(USART2);
    
    // Включаем USART2
    usart_enable(USART2);
}

// Настройка SPI2
void spi2_setup(void)
{
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

    // Включаем прерывания (опционально)
    spi_enable_rx_buffer_not_empty_interrupt(SPI2);

    // Регистрируем обработчик прерывания в NVIC
    nvic_enable_irq(NVIC_SPI2_IRQ);
}

void spi2_isr(void)
{
    // Проверяем флаг приема данных
    if (SPI_SR(SPI2) & SPI_SR_RXNE) {
        uint8_t received_data = spi_read(SPI2);
        usart_send_blocking(USART2, received_data);
    }
}
// Функция для передачи данных через SPI2
uint8_t spi2_transfer()
{
    
    // Ждем приема данных
    while (!(SPI_SR(SPI2) & SPI_SR_RXNE));
    
    // Читаем принятые данные
    return static_cast<uint8_t>(spi_read(SPI2));
}

void spi2_select_slave(void)
{
    gpio_clear(GPIOB, GPIO12); // NSS низкий уровень
}

// Функция для освобождения ведомого устройства
void spi2_deselect_slave(void)
{
    gpio_set(GPIOB, GPIO12); // NSS высокий уровень
}

void nvic_setup(void)
{
    // Включаем прерывание USART2 в контроллере прерываний (NVIC)
    nvic_enable_irq(NVIC_USART2_IRQ);
    // Устанавливаем приоритет прерывания (меньше число - выше приоритет)
    nvic_set_priority(NVIC_USART2_IRQ, 0);

    // Включаем прерывание SPI2 (с более низким приоритетом)
    nvic_enable_irq(NVIC_SPI2_IRQ);
    nvic_set_priority(NVIC_SPI2_IRQ, 1);
}

void usart2_isr(void)
{
    if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
        
		// Чтение USART_DR автоматически очищает флаг USART_SR_RXNE
        usart_send_blocking(USART3,usart_recv(USART2));
		//активировать прерывание по готовности передатчика UART 
	}

}

void uart3_write(uint8_t* data, const uint32_t length ){
	for(uint32_t i = 0; i < length; i++ ){
		usart_send_blocking(USART3, data[i]);
	}
}

void config_radiomodule(void){
    gpio_set(GPIOA, GPIO0);
	gpio_set(GPIOA, GPIO1);
    
    delay_ms(200);

	uint8_t str_tx[]={0xC0,0x00,0x00,0x1A,0x06,0x44}; // Настройка для радиомодуля
    delay_ms(200);

    gpio_clear(GPIOA, GPIO0);
	gpio_clear(GPIOA, GPIO1);

    delay_ms(200);

}

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();
    spi2_setup();
    systick_setup();
    nvic_setup();

    //config_radiomodule();
	spi2_select_slave();
    State state = idle;
    
    while (1) {
        spi_send(SPI2, 'A');
        spi_send(SPI2, 'B');
        spi_send(SPI2, 'C');
        spi_send(SPI2, 'D');
        delay_ms(100);
    }
    
    return 0;
}


