#include "../inc/functions.hpp"


void Circular_buffer::put(uint8_t d){
	buf[wr_idx] = d;
	wr_idx ++;
	wr_idx %= SIZE;
	if(wr_idx == rd_idx) full_ = true;
}
  

uint8_t Circular_buffer::get(){
	uint8_t d = buf[rd_idx];
	rd_idx++;
	rd_idx %= SIZE;
	return d;
}

uint8_t Circular_buffer::get_rd(void){
	return rd_idx;
}  

uint8_t Circular_buffer::get_wr(void){
	return wr_idx;
}  

bool Circular_buffer::empty(){return ((wr_idx == rd_idx) and (not full_));}
bool Circular_buffer::full(){return full_;}
Circular_buffer::Circular_buffer():wr_idx{},rd_idx{},full_{false}{}

void uart3_write(uint8_t* data, const uint32_t length) {
    for(uint32_t i = 0; i < length; i++) {
        usart_send_blocking(USART3, data[i]);
    }
}

void uart2_write(uint8_t* data, const uint32_t length ){
	for(uint32_t i = 0; i < length; i++ ){
		usart_send_blocking(USART2,data[i]);
	}

}

void uart1_write(uint8_t* data, const uint32_t length ){
	for(uint32_t i = 0; i < length; i++ ){
		usart_send_blocking(USART1,data[i]);
	}

}


char* reverse(char* buffer, int i, int j) {
  while (i < j) {
    char t = buffer[i];
    buffer[i++] = buffer[j];
    buffer[j--] = t;
  }

  return buffer;
}

void spi2_select_slave(void) {
    gpio_clear(GPIOB, GPIO12); // NSS низкий уровень
}

// Функция для освобождения ведомого устройства
void spi2_deselect_slave(void) {
    gpio_set(GPIOB, GPIO12); // NSS высокий уровень
}

// Функция для передачи данных через SPI2
uint8_t spi2_transfer(uint8_t data) {
    // Ждем, пока передатчик не будет готов
    while (!(SPI_SR(SPI2) & SPI_SR_TXE));
    
    // Отправляем данные
    spi_send(SPI2, data);
    
    // Ждем приема данных
    while (!(SPI_SR(SPI2) & SPI_SR_RXNE));
    
    // Читаем принятые данные
    return static_cast<uint8_t>(spi_read(SPI2));
}


