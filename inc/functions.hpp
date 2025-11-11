#include <cstdint>
#include <cmath> 
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>

#include <libopencm3/cm3/nvic.h>

constexpr uint8_t SIZE{64};
class Circular_buffer {
public:
	void put(uint8_t); // Положить данные в буфер

	uint8_t get(); // Получить данные из буфера
	uint8_t get_rd(); // Получить индекс для чтения
	uint8_t get_wr(); // Получить индекс для записи
	
	bool empty(); // Возвращает True, если буфер пустой 
	bool full(); // Возвращает True, если буфер полон
	
	Circular_buffer(); // Circular_buffer b1;
	uint8_t buf[SIZE]; // Противоречит конструктору с параметрами
		
private:
	uint8_t wr_idx;
	uint8_t rd_idx;
	bool full_;
};


void uart2_write(uint8_t* data, const uint32_t length );
void uart1_write(uint8_t* data, const uint32_t length );
void uart3_write(uint8_t* data, const uint32_t length );

char* reverse(char* buffer, int i, int j);


void spi2_select_slave(void); // Выбор slave устройства

// Функция для освобождения ведомого устройства
void spi2_deselect_slave(void);

void config_radiomodule(void);

// Функция для передачи данных через SPI2
uint8_t spi2_transfer(uint8_t data);