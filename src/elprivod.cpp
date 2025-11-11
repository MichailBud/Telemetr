#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>

uint32_t turn;
uint32_t adc_value = 0;

int main() {
    // Включение тактирования
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_TIM6);
    rcc_periph_clock_enable(RCC_GPIOA);  // Для АЦП (PA0)
    rcc_periph_clock_enable(RCC_ADC1);
    
    // Настройка светодиодов
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15 | GPIO14 | GPIO13 | GPIO12);
    
    // Настройка АЦП
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);  // PA0 - ADC1_IN0
    
    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28CYC);
    adc_power_on(ADC1);
    
    // Начальная настройка таймера
    timer_set_prescaler(TIM6, 160 - 1);
    timer_set_period(TIM6, 500 - 1);
    timer_enable_counter(TIM6);
    timer_enable_irq(TIM6, TIM_DIER_UIE);  // Разрешение прерывания по update
    nvic_enable_irq(NVIC_TIM6_DAC_IRQ);    // Разрешение прерываний в ЦПУ

    turn = (2048) * 3000;

    while(true) {
        // Считывание значения с АЦП
        adc_start_conversion_regular(ADC1);
        while(!adc_eoc(ADC1));  // Ждем завершения преобразования
        adc_value = (uint16_t)adc_read_regular(ADC1);
        
        // Обновление частоты таймера на основе значения АЦП
        // ADC_value от 0 до 4095, преобразуем в период от 100 до 2000
        uint32_t new_period = 100 + (adc_value * 1900) / 4095;
        
        // Обновляем период таймера (останавливаем, меняем, запускаем)
        timer_disable_counter(TIM6);
        timer_set_period(TIM6, new_period - 1);
        timer_enable_counter(TIM6);
        
        // Небольшая задержка для стабильности АЦП
        for(volatile int i = 0; i < 100000; i++);
    }
}

uint8_t led_num = 0b00000001;

// Функция-обработчик прерывания
void tim6_dac_isr() {
    if(turn > 0) {
        timer_clear_flag(TIM6, TIM_SR_UIF);  
        
        if(led_num >= 16) {
            led_num = 0b00000001;
        }

        switch(led_num) {
            case 0b00000001:  
                gpio_set(GPIOD, GPIO15);
                gpio_clear(GPIOD, GPIO14);
                gpio_clear(GPIOD, GPIO13);
                gpio_clear(GPIOD, GPIO12);
                break;
            case 0b00000010:
                gpio_clear(GPIOD, GPIO15);
                gpio_set(GPIOD, GPIO14);
                gpio_clear(GPIOD, GPIO13);
                gpio_clear(GPIOD, GPIO12);
                break;
            case 0b00000100:
                gpio_clear(GPIOD, GPIO15);
                gpio_clear(GPIOD, GPIO14);
                gpio_set(GPIOD, GPIO13);
                gpio_clear(GPIOD, GPIO12);
                break;
            case 0b00001000:
                gpio_clear(GPIOD, GPIO15);
                gpio_clear(GPIOD, GPIO14);
                gpio_clear(GPIOD, GPIO13);
                gpio_set(GPIOD, GPIO12);
                break;
        }
        turn--;
        led_num *= 2;
    } else {
        gpio_clear(GPIOD, GPIO15);
        gpio_clear(GPIOD, GPIO14);
        gpio_clear(GPIOD, GPIO13);
        gpio_clear(GPIOD, GPIO12);
    }
}