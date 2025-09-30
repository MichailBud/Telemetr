#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "E32_dop.cpp"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>


uint8_t data_buffer;	
Circular_buffer b;

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
    
    // Включаем USART2
    usart_enable(USART2);
}

void uart3_write(uint8_t* data, const uint32_t length ){
	for(uint32_t i = 0; i < length; i++ ){
		usart_send_blocking(USART3, data[i]);
	}
}

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();
    systick_setup();

    

    uint8_t str[6];
	uint8_t byte_data;

	gpio_set(GPIOA, GPIO0);
	gpio_set(GPIOA, GPIO1);
    
    delay_ms(2000);

	uint8_t str_tx[]={0xC0,0x00,0x00,0x1A,0x06,0x44};
    //usart_send_blocking(USART1, 'B');
    uart2_write(str_tx,6);

    delay_ms(200);



	// uint8_t str2_tx[]={0xC1, 0xC1, 0xC1};
	// uart1_write(str2_tx,3);
    
    while (1) {
		//если индексы чтения и записи в кольцевом буфере совпадают
		if(!b.empty()){
			// usart_send_blocking(USART2,b.get());
			byte_data = b.get();//временно, по сути ничё не делает(надо изменить настройки класса)
			// usart_send_blocking(USART2,' ');
			// itoa(b.count,data_amount,10);
			// usart_send_blocking(USART2,data_amount[0]);
			
			//если в буфере накопилось 6 байт 
			if(b.count>=6){
				for(int i=0; i<6; ++i){
					str[i]=b.buf[6-b.wr_idx+i];//тут есть беда: если я начну записывать с конца буфера и перейду в начало, то...
				}
				for(int i=0; i<6; ++i){
					usart_send_blocking(USART3,str[i]);
				}
				b.wr_idx =0;
				b.rd_idx =0;
				b.full_ = false;
				gpio_clear(GPIOA, GPIO0);
				gpio_clear(GPIOA, GPIO1);
				// usart_send_blocking(USART1,'/n');
            }
        }
    }
    
    return 0;
}


