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
#include <cstring>

#include "FreeRTOS.h"
#include "heap_lock_monitor.h"
#include "task.h"
#include "queue.h"
#include "DigitalIoPin.h"
#include "ITM_write.h"
#include "Fmutex.h"
#include "LpcUart.h"
#include <cstring>
#include "modbusConfig.h"
#include "LiquidCrystal.h"
#include "IntegerEdit.h"
#include "SimpleMenu.h"

SimpleMenu menu;
IntegerEdit *co2_t;
#if 1

static void prvHardwareSetup(void) {
	SystemCoreClockUpdate();
	Board_Init();
	Chip_RIT_Init(LPC_RITIMER);
	Board_LED_Set(0, false);
	Board_LED_Set(2, true);
	/* Initialize interrupt hardware */
	DigitalIoPin::GPIO_interrupt_init();
	ITM_init();
	ITM_write("ITM ok!\n");
}

Fmutex sysMutex;
QueueHandle_t hq;
SemaphoreHandle_t xSem;
modbusConfig modbus;
DigitalIoPin encoder_A(0, 5, DigitalIoPin::pullup, true);
DigitalIoPin encoder_B(0, 6, DigitalIoPin::pullup, true);

/* variables to read from MODBUS sensors */
struct SensorData {
	int temp;
	int rh;
	int co2;
	uint64_t time_stamp;
};
struct dataevent {
	int t;
	int r;
	int c;
	uint64_t t_stamp;
};

/* Interrupt handlers must be wrapped with extern "C" */

extern "C"{
/* ISR for encode rotator A */
void PIN_INT0_IRQHandler(void)
{
	Board_LED_Toggle(2);
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	/* create an event and send to the queue upon an interrupt */
    //int i = co2_t->getValue();
    if(encoder_B.read()){
    	//i--;
    	menu.event(MenuItem::down);
    }else{
    	//i++;
    	menu.event(MenuItem::up);
    }
	/* switch back to the previous context */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
#if 0
/* ISR for encode rotator B */
void PIN_INT1_IRQHandler(void){
	portBASE_TYPE xHigherPriorityTaskWoken = pdTRUE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
	BtnEvent e { 2, xTaskGetTickCountFromISR() };
	xQueueSendFromISR(hq, &e, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
#endif
/* ISR for button 3 */
void PIN_INT2_IRQHandler(void){
	Board_LED_Toggle(2);
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
	ITM_write("button ok");
	if(menu.getIndex()==1 ) menu.event(MenuItem::ok);
	//menu.event(MenuItem::ok);
	//menu.event(MenuItem::show);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/* enable runtime statistics */
void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}
/* end runtime statictics collection */

void vStartSimpleMQTTDemo(void);
}


/* @brief task controls MQTT interface */
static void vTaskMQTT(void *pvParameters){
	//implementation

}






/* task to wait on the queue event from ISR and print it */
static void vTaskLCD(void *pvParams){
	//LCD configuration
	DigitalIoPin *rs = new DigitalIoPin(0, 29, DigitalIoPin::output);
	DigitalIoPin *en = new DigitalIoPin(0, 9, DigitalIoPin::output);
	DigitalIoPin *d4 = new DigitalIoPin(0, 10, DigitalIoPin::output);
	DigitalIoPin *d5 = new DigitalIoPin(0, 16, DigitalIoPin::output);
	DigitalIoPin *d6 = new DigitalIoPin(1, 3, DigitalIoPin::output);
	DigitalIoPin *d7 = new DigitalIoPin(0, 0, DigitalIoPin::output);
	LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

	IntegerEdit *co2_= new IntegerEdit(&lcd, std::string("CO2"), 10000, 0, 1);
	co2_t= new IntegerEdit(&lcd, std::string("CO2 target"), 10000, 0, 1);
	IntegerEdit *rh_= new IntegerEdit(&lcd, std::string("RH"), 100, 0, 1);
	IntegerEdit *temp_= new IntegerEdit(&lcd, std::string("Temp"), 60, -40, 1);

	menu.addItem(new MenuItem(co2_));
	menu.addItem(new MenuItem(co2_t));
	menu.addItem(new MenuItem(rh_));
	menu.addItem(new MenuItem(temp_));

	co2_->setValue(0);
	co2_t->setValue(0);
	rh_->setValue(0);
	temp_->setValue(0);
	//lcd.begin(16, 2);
	menu.event(MenuItem::show);
	dataevent e;
	// 1. display values to LCD UI
	for( ;; ){

		// 2. take semaphore and update
		//if(xQueueReceive(hq, &e, portMAX_DELAY)){     // receive data sensors from queue
			xQueueReceive(hq, &e, portMAX_DELAY);
			temp_->setValue(e.t);
			rh_->setValue(e.r);
			co2_->setValue(e.c);
			menu.event(MenuItem::show);
		//}
		vTaskDelay(500);
	}

}

static void vTaskMODBUS(void *pvParams){

	char buff[40];
	SensorData sensor_event;

	while(1)  {
		sensor_event.temp = modbus.get_temp();
		sensor_event.rh = modbus.get_rh();
		sensor_event.co2 = modbus.get_co2();
        sensor_event.time_stamp= xTaskGetTickCount();

        xQueueSendToBack(hq, &sensor_event, portMAX_DELAY);


		vTaskDelay(500); //this is not required if semaphore is used
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
	//DigitalIoPin encoder_A(0, 5, DigitalIoPin::pullup, true);
	//DigitalIoPin encoder_B(0, 6, DigitalIoPin::pullup, true);
	DigitalIoPin encoder_Button(1, 8, DigitalIoPin::pullup, true);

	DigitalIoPin solenoid_valve(0, 27, DigitalIoPin::pullup, true);

	/* configure interrupts for those buttons */
	//enable_interrupt(IRQ number, NVIC priority, port, pin)
	encoder_A.enable_interrupt(0, 0, 0, 5);
	encoder_Button.enable_interrupt(2, 0, 1, 8);

	/* create a counting sempahore of max 5 events */
	xSem = xSemaphoreCreateCounting(10, 0);

	/* create a queue of max 10 events */
	hq = xQueueCreate(10, sizeof(int));

	/* task MQTT */
	vStartSimpleMQTTDemo();

	/* task LCD */
	xTaskCreate(vTaskLCD, "LCD_Task",
			((configMINIMAL_STACK_SIZE)+512), NULL, tskIDLE_PRIORITY + 1UL,
						(TaskHandle_t *) NULL);

	/* task co2 monitor */


	/* task measurement modbus */
	xTaskCreate(vTaskMODBUS, "Measuring",
			((configMINIMAL_STACK_SIZE)+128), NULL, tskIDLE_PRIORITY + 1UL,
						(TaskHandle_t *) NULL);

	vTaskStartScheduler();

	/* never arrive here */
    return 1 ;
}
#endif /* task 1 ends */
