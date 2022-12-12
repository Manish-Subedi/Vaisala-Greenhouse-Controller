/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>
#include "FreeRTOS.h"
#include "heap_lock_monitor.h"
#include "task.h"
#include "DigitalIoPin.h"
#include "ITM_write.h"
#include "Fmutex.h"
#include "LpcUart.h"
#include <cstring>
#include "modbusConfig.h"



#if 1

static void prvHardwareSetup(void) {
	SystemCoreClockUpdate();
	Board_Init();
	Board_LED_Set(0, false);
	Board_LED_Set(2, true);
	/* Initialize interrupt hardware */
	DigitalIoPin::GPIO_interrupt_init();
	ITM_init();
	ITM_write("ITM ok!\n");
}

struct BtnEvent {
	int pin;
	uint64_t timestamp;
};

/* filter duration */
static int filter_len = 50; // 50ms by default

Fmutex sysMutex;
QueueHandle_t hq;
//SemaphoreHandle_t xSem;
modbusConfig modbus;

/* Interrupt handlers must be wrapped with extern "C" */
#if 1
extern "C"{
/* ISR for SW 1 */
void PIN_INT0_IRQHandler(void)
{
	Board_LED_Toggle(2);
	/* this must be set to true so that the context switch
	 * gets back to the ongoing task
	 * */
	portBASE_TYPE xHigherPriorityTaskWoken = pdTRUE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	/* create an event and send to the queue upon an interrupt */

	/* switch back to the previous context */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void PIN_INT1_IRQHandler(void){
	portBASE_TYPE xHigherPriorityTaskWoken = pdTRUE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
	BtnEvent e { 2, xTaskGetTickCountFromISR() };
	xQueueSendFromISR(hq, &e, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/* ISR for button 3 */
void PIN_INT2_IRQHandler(void){
	Board_LED_Toggle(2);
	portBASE_TYPE xHigherPriorityTaskWoken = pdTRUE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
	BtnEvent e { 3, xTaskGetTickCountFromISR() };
	xQueueSendFromISR(hq, &e, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
}

#endif
/* @brief task reads from serial port and prints to serial and ITM */
static void vTaskSerialPort(void *pvParams){
	int len = 15;
	char str[25];
	char buff[len];
	int count = 0;
	LpcUart *dbgu = static_cast<LpcUart *>(pvParams);
	while(1){
		int read = dbgu->read(buff + count, len - count);
		if(read > 0){
			count += read;
			if(strchr(buff, '\r') != NULL || count >= len){
				sysMutex.lock();
				dbgu->write(buff, count);
				dbgu->write('\n');
				sysMutex.unlock();
				/* get the filter value */
#if 0			/* method 1 */
				char *token;
				token = strtok(buff, "   ");
				int c = 0;
				while(token != NULL){
					if (c == 1) filter_len = atoi(token);
					c++;
					token = strtok(NULL, "   ");
				}
#endif			/* method 2 */
				if(strncmp(buff, "filter ", 7) == 0){
					if(sscanf(buff + 7, "%d", &filter_len) == 1){
						snprintf(str, 25, "filter is set to %d\n", filter_len);
						sysMutex.lock();
						ITM_write(str);
						sysMutex.unlock();
					}
				}
				else ITM_write("Error: format is \"filter xxx\"\n\t\tif you wanted to set the filter value\n");
				/* reset the buffers and variables */
				memset(&buff, 0, strlen(buff));
				memset(&str, 0, strlen(str));
				count = 0;
			}
		}
	}
}

/* task to wait on the queue event from ISR and print it */
static void vTaskPrint(void *pvParams){
	char buffer[30];
	uint64_t elapsed, prev_timestamp = 0;
	BtnEvent e;
	while(1){
		if(xQueueReceive(hq, &e, portMAX_DELAY) == pdTRUE){
			elapsed = e.timestamp - prev_timestamp;

			if((static_cast<int>(elapsed)) >= filter_len){
				if (elapsed >= 1000){
					float elapsed_s = elapsed / 1000;
					snprintf(buffer, 30, "%.1f s SW%d pressed\n", elapsed_s, e.pin);
				}
				else snprintf(buffer, 30, "%d ms SW%d pressed\n", static_cast<int>(elapsed), e.pin);
				sysMutex.lock();
				ITM_write(buffer);
				sysMutex.unlock();
			}
		prev_timestamp = e.timestamp;
		}
	}
}

static void vTaskMeasure(void *pvParams){

	int temp = 0;
	int rh = 0;
	int co2  = 0;
	char buff[20];

	while(1)  {
		temp = modbus.get_temp();
		rh = modbus.get_rh();
		co2 = modbus.get_co2();

		sprintf(buff, "\n\rtemp: %d\n\rrh: %d\n\rco2: %d", temp, rh, co2);
		ITM_write(buff);
		vTaskDelay(500);
	}
}


int main(void) {
	prvHardwareSetup();
	heap_monitor_setup();

	/* UART port config */
	LpcPinMap none = {-1, -1}; // unused pin has negative values in it
	LpcPinMap txpin = { 0, 18 }; // transmit pin that goes to debugger's UART->USB converter
	LpcPinMap rxpin = { 0, 13 }; // receive pin that goes to debugger's UART->USB converter
	LpcUartConfig cfg = { LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false, txpin, rxpin, none, none };

	LpcUart *dbgu = new LpcUart(cfg);

	/* Configure pins and ports for Rotary Encoder as input, pullup, and inverted */
	DigitalIoPin encoder_A(0, 5, DigitalIoPin::pullup, true);
	DigitalIoPin encoder_B(0, 6, DigitalIoPin::pullup, true);
	DigitalIoPin encoder_Button(1, 8, DigitalIoPin::pullup, true);

	DigitalIoPin solenoid_valve(0, 27, DigitalIoPin::pullup, true);

	/* configure interrupts for those buttons */
	encoder_A.enable_interrupt(0, 0, 0, 5);
	encoder_B.enable_interrupt(1, 0, 0, 11);
	encoder_Button.enable_interrupt(2, 0, 1, 9);

	/* create a counting sempahore of max 5 events */
	//xSem = xSemaphoreCreateCounting(5, 0);

	/* create a queue of max 10 events */
	hq = xQueueCreate(10, sizeof(BtnEvent));


	xTaskCreate(vTaskMeasure, "Measuring",
			((configMINIMAL_STACK_SIZE)+128), NULL, tskIDLE_PRIORITY + 3UL,
						(TaskHandle_t *) NULL);

	vTaskStartScheduler();

	/* never arrive here */
    return 1 ;
}
#endif /* task 1 ends */
