#include "stm32f446xx.h"
#include "stdio.h"
#include "clock.h"
#include "timer.h"
#include "uart.h"
#include "dma.h"
#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "globals.h"
#include "math.h"
#include "string.h"
#include "oled.h"
sys_info dp = { 0 };

volatile uint16_t cnt = 0;
volatile uint16_t pwm_target = 0;
volatile uint8_t low_temp_read = 0;
volatile uint8_t high_temp_read = 0;
volatile uint8_t flag = 0;
//dma variables
uint16_t buffer[DMA0_BUFFER_SIZE];
uint8_t proximity;
//uart varaible
char uart_buffer[50];
extern volatile uint8_t uart_busy;
volatile uint8_t dma_busy = 0;

//display message at start on oled
void welcome_message(void) {
	oled_clear();
	oled_print(0, 0, " PARAMETERS MONITOR  ");
	oled_print(0, 1, "---------------------");
	oled_print(0, 2, "       WELCOME       ");
	oled_print(0, 4, "Press Button To Start");
	oled_print(0, 6, "          ->         ");
	oled_flush();
}

//stopping cooling and heating process
void stop_cooling(void) {
	flag &= ~(COOLING_PROCESS);
	//blink off
	TIM2->CR1 &= ~(TIM_CR1_CEN);
	//vent close
	TIM4->DIER |= (TIM_DIER_UIE);
	pwm_target = 2000;
	//fan off
	TIM1->CR1 &= ~(TIM_CR1_CEN);
	dp.fan = 0;
	dp.vent = 0;
	high_temp_read = 0;
}
void stop_heat(void) {
	flag &= ~(HEATING_PROCESS);
	//blink off
	TIM2->CR1 &= ~(TIM_CR1_CEN);
	//vent close
	dp.vent = 0;
	TIM4->DIER |= (TIM_DIER_UIE);
	pwm_target = 2000;
}

//check temperature and decide to start cooling or heating process
void check_temp(void) {
	float vout = ((buffer[0] * 3.3f) / (4095.0f));
	float rntc = Rfix * (3.3f / vout - 1.0f);
	float lnR = logf(rntc);
	float inv_t = A + (B * lnR) + (C * lnR * lnR * lnR);
	float tempK = 1.0f / inv_t;
	dp.temp = (tempK - 273.15f);   // convert to Celsius
	if (dp.temp > optimum_temp_high) {
		++high_temp_read;
		if (!(flag & COOLING_PROCESS) && high_temp_read > 4) {
			flag |= (COOLING_PROCESS) | (START_COOLING);
			high_temp_read = 0;
		}
	} else if (dp.temp < optimum_temp_low) {
		++low_temp_read;
		if (!(flag & HEATING_PROCESS) && low_temp_read > 4) {
			flag |= HEATING_PROCESS | START_HEATING;
			low_temp_read = 0;
		}
	} else {
		high_temp_read = 0;
		low_temp_read = 0;
		if (flag & COOLING_PROCESS) {
			stop_cooling();
		}
		if (flag & HEATING_PROCESS)
			stop_heat();
	}
}

//check ldr value and proximity to decide door status
void check_ldr_ir_proximity() {
	float vout = ((buffer[1] * 3.3f) / (4095.0f));
	float rldr = Rfix * (3.3f / vout - 1.0f);
	float lux = pow(500000.0f / rldr, 1.428f);
	dp.ldr = lux;
	proximity = GPIOF->IDR & (GPIO_IDR_ID6);
	if (dp.ldr > LDR_Threshold && !proximity)
		dp.door = 1;
	if (dp.ldr < LDR_Threshold && dp.door == 1)
		dp.door = 0;
}

//update oled display with current parameters
void update_display(void) {
	if (oled_is_busy())
		return;
	char line[30];
	if (dp.temp > optimum_temp_high)
		snprintf(line, sizeof(line), "Temp: %.2f%cC [HIGH TEMP]", dp.temp,
		SYM_DEGREE);
	else if (dp.temp < optimum_temp_low)
		snprintf(line, sizeof(line), "Temp: %.2f%cC [LOW TEMP]", dp.temp,
		SYM_DEGREE);
	else
		snprintf(line, sizeof(line), "Temp: %.2f%cC [OK]", dp.temp, SYM_DEGREE);
	;
	oled_print(0, 2, line);
	snprintf(line, sizeof(line), "LDR: %u", buffer[1]);
	oled_print(0, 3, line);
	snprintf(line, sizeof(line), "Door: %s          ",dp.vent ? "OPEN" : "CLOSED");
	oled_print(0, 4, line);
	snprintf(line, sizeof(line), "Fan: %s           ", dp.fan ? "ON" : "OFF");
	oled_print(0, 5, line);
	snprintf(line, sizeof(line), "Vent: %s          ", dp.vent ? "OPEN" : "CLOSED");
	oled_print(0, 6, line);
	oled_flush();
}

//process data received from dma, check parameters, send via uart and update oled display
void process_dma_data(void) {
	check_temp();
	check_ldr_ir_proximity();
	while (uart_busy)
		;
	memset(uart_buffer, 0, sizeof(uart_buffer));
	snprintf(uart_buffer, sizeof(uart_buffer),
			"%u, %.2f, %u,%.2f %u, %u , %u, %u \r\n", ++cnt, dp.temp, buffer[0],
			dp.ldr, buffer[1], dp.door, dp.fan, dp.vent);
	send((char*) uart_buffer);
	update_display();
}
//stop all processes and reset system to initial state
void sys_stop(void) {
	//tim3 stop
	TIM3->CR1 &= ~(TIM_CR1_CEN);
	TIM3->CNT = 0;
	//servo back to initial and blink stop
	TIM4->DIER |= (TIM_DIER_UIE);
	TIM2->CR1 &= ~(TIM_CR1_CEN);
	pwm_target = 2000;
	TIM2->CCR1 = 0;
	//dma_stop
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream0->CR & DMA_SxCR_EN)
	;
	DMA2->LIFCR = (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0
		| DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0);
		
		welcome_message();
		GPIOC->BSRR = (GPIO_BSRR_BR7);
		sys_initialized = 0;
}
	//start cooling process: blink led, open vent and fan
void start_cooling(void) {
	//blink led in timer
	TIM2->CR1 |= (TIM_CR1_CEN);
	TIM2->CCR1 = 1999;
	//vent open
	TIM4->DIER |= (TIM_DIER_UIE);
	pwm_target = 1000;
	dp.vent = 1;
	//fan open
	TIM1->CCR2 = 400;
	TIM1->CR1 |= (TIM_CR1_CEN);
	dp.fan = 1;
}
	
void start_heating(void) {
	//blink led in timer
	TIM2->CR1 |= (TIM_CR1_CEN);
	TIM2->CCR1 = 1999;
	//vent open 45 degree
	TIM4->DIER |= (TIM_DIER_UIE);
	pwm_target = 1500;
	dp.vent = 1;
}

uint8_t sys_initialized = 0;
int main(void) {
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
	config_clock();
	gpio_init();
	timer_init();
	uart_init();
	i2c_gpio_init();
	dma_init();
	oled_init();
	adc_init();
	welcome_message();
	send((char*) "CNT TEMP RAW_TEMP LDR RAW_LDR DOOR FAN VENT\r\n");
	for (;;) {
		if (flag & START_SYS) {
			if (!sys_initialized) {
				GPIOC->BSRR = (GPIO_BSRR_BS7);
				sys_initialized = 1;
				oled_print(0, 4, "                     ");
				oled_print(0, 5, "  SENDING DATA ....  ");
				oled_print(0, 6, "                     ");
				oled_flush();
				TIM3->CR1 |= (TIM_CR1_CEN);
				DMA2_Stream0->CR |= DMA_SxCR_EN;
				TIM6->CR1 |= (TIM_CR1_CEN); 
			}
			if (flag & DMA_PROCESS) {
				process_dma_data();
				flag &= ~(DMA_PROCESS);
			}
			if (flag & START_COOLING) {
				start_cooling();
				flag &= ~(START_COOLING);
			}
			if (flag & HEATING_PROCESS) {
				start_heating();
				flag &= ~(START_HEATING);
			}
		} else {
			if (sys_initialized) {
				sys_stop();
			}
		}
	}
}
