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

#include "DigitalIoPin.h"
#include "ITM_write.h"
#include "Fmutex.h"
#include "LpcUart.h"
#include <cstring>
#include "modbusConfig.h"
#include "LiquidCrystal.h"
#include "IntegerEdit.h"
#include "SimpleMenu.h"
#include "./mqtt_demo/MQTT_custom.h"
#include "timers.h"

SimpleMenu menu;
IntegerEdit *co2_t;

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
QueueHandle_t hq; //sensor data
QueueHandle_t mq; // mqtt data
QueueHandle_t iq;

// Semaphore and timeout for sending data to MQTT in intervals
SemaphoreHandle_t xSemaphoreMQTT;
TimerHandle_t sendToMQTTTimer;
const TickType_t MQTTInterval = pdMS_TO_TICKS(2000);	// 30 000 = 5 minutes

// Semaphore and timeout for activating valve
SemaphoreHandle_t xSemaphoreValve;
TimerHandle_t controlValveTimer;
const TickType_t valveInterval = pdMS_TO_TICKS(2000);

// valve close timeout
TimerHandle_t controlValveTimerOpen;
const TickType_t valveIntervalOpen = pdMS_TO_TICKS(2000);

modbusConfig modbus;

DigitalIoPin *encoder_A;
DigitalIoPin *encoder_B;
DigitalIoPin *encoder_Button;
DigitalIoPin *solenoid_valve;

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

/* read co2 value set from the LCD UI */
int solenoid_state = 0;
int co2_new = 0;
/* Interrupt handlers must be wrapped with extern "C" */

extern "C"{
/* ISR for encode rotator SIGA */
void PIN_INT0_IRQHandler(void)
{
	Board_LED_Toggle(2);
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	int e = 1;
	xQueueSendFromISR(iq, &e, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/* ISR for button 3 */
void PIN_INT2_IRQHandler(void){
	Board_LED_Toggle(2);
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
	int e = 2;
	xQueueSendFromISR(iq, &e, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/* enable runtime statistics */
void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}
/* end runtime statictics collection */

/* function declarations for MQTT */
uint32_t prvGetTimeMs(void);
PlaintextTransportStatus_t prvConnectToServerWithBackoffRetries( NetworkContext_t * pxNetworkContext );
void prvCreateMQTTConnectionWithBroker( MQTTContext_t * pxMQTTContext, NetworkContext_t * pxNetworkContext );
void prvMQTTSubscribeWithBackoffRetries( MQTTContext_t * pxMQTTContext );
void prvMQTTPublishToTopic( char *, MQTTContext_t * pxMQTTContext );
MQTTStatus_t MQTT_ProcessLoop( MQTTContext_t * pContext, uint32_t timeoutMs );
}

uint8_t ucSharedBuffer[ mqttexampleSHARED_BUFFER_SIZE ];
uint32_t ulGlobalEntryTimeMs;
uint16_t usSubscribePacketIdentifier;
uint16_t usUnsubscribePacketIdentifier;

/* @brief task controls MQTT interface */
static void prvMQTTTask( void * pvParameters )
{
    //uint32_t ulPublishCount = 0U, ulTopicCount = 0U;
    //const uint32_t ulMaxPublishCount = 5UL;
    NetworkContext_t xNetworkContext = { 0 };
    PlaintextTransportParams_t xPlaintextTransportParams = { 0 };
    MQTTContext_t xMQTTContext;
    MQTTStatus_t xMQTTStatus;
    PlaintextTransportStatus_t xNetworkStatus;

    /* Remove compiler warnings about unused parameters. */
    ( void ) pvParameters;

    /* Set the pParams member of the network context with desired transport. */
    xNetworkContext.pParams = &xPlaintextTransportParams;

    ulGlobalEntryTimeMs = prvGetTimeMs();
    char buff[40];
    SensorData recv;
    for( ; ; )
    {
        /****************************** Connect. ******************************/

        /* Attempt to connect to the MQTT broker. If connection fails, retry after
         * a timeout. The timeout value will exponentially increase until the
         * maximum number of attempts are reached or the maximum timeout value is
         * reached. The function below returns a failure status if the TCP connection
         * cannot be established to the broker after the configured number of attempts. */
        xNetworkStatus = prvConnectToServerWithBackoffRetries( &xNetworkContext );
        configASSERT( xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS );

        /* Sends an MQTT Connect packet over the already connected TCP socket,
         * and waits for a connection acknowledgment (CONNACK) packet. */
        LogInfo( ( "Creating an MQTT connection to %s.", democonfigMQTT_BROKER_ENDPOINT ) );
        prvCreateMQTTConnectionWithBroker( &xMQTTContext, &xNetworkContext );

        /**************************** Subscribe. ******************************/

        /* If server rejected the subscription request, attempt to resubscribe to
         * the topic. Attempts are made according to the exponential backoff retry
         * strategy declared in backoff_algorithm.h. */
        //prvMQTTSubscribeWithBackoffRetries( &xMQTTContext );

        /******************* Publish and Keep Alive Loop. *********************/

        /* Publish messages with QoS0, then send and process Keep Alive messages. */
        for( ;; ){
        	xQueueReceive(mq, &recv, portMAX_DELAY);
        	sprintf(buff, "field1=%d&field2=%d&field3=%d&field4=%d&field5=%d", recv.co2, recv.rh, recv.temp, solenoid_state, co2_new);
        	prvMQTTPublishToTopic( buff, &xMQTTContext );
        }
    }
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
	co2_t= new IntegerEdit(&lcd, std::string("CO2 target"), 10000, 0, 20);
	IntegerEdit *rh_= new IntegerEdit(&lcd, std::string("RH"), 100, 0, 1);
	IntegerEdit *temp_= new IntegerEdit(&lcd, std::string("Temp"), 60, -40, 1);

	solenoid_valve = new DigitalIoPin(0, 27, DigitalIoPin::output);
	solenoid_valve->write(false);

	menu.addItem(new MenuItem(co2_));
	menu.addItem(new MenuItem(co2_t));
	menu.addItem(new MenuItem(rh_));
	menu.addItem(new MenuItem(temp_));
  
	co2_->setValue(0);
	co2_t->setValue(0);
	rh_->setValue(0);
	temp_->setValue(0);

	menu.event(MenuItem::show);
	dataevent e;

	// 1. display values to LCD UI
	for( ;; ){

		// 2. take semaphore and update
		if(xQueueReceive(hq, &e, portMAX_DELAY) == pdTRUE){     // receive data sensors from queue
			temp_->setValue(e.t);
			rh_->setValue(e.r);
			co2_->setValue(e.c);
			menu.event(MenuItem::show);
		}

		co2_new = co2_t->getValue();
		/* write to EEPROM */


		int offset = 10;
		char read[10];
		sprintf(read, "\nset_point co2 :%d\n", co2_new);
		ITM_write(read);
		/* control the valve */
		//if( xSemaphoreTake( xSemaphoreValve, ( TickType_t ) 0 ) == pdTRUE ) {
#if 0
			if((co2_new + offset) < co2_->getValue()) {
				solenoid_valve->write(false);
				solenoid_state = 0;
				ITM_write("\nvalve closed!\n");
			}
#endif
			if((co2_new-offset) > co2_->getValue()) {
				solenoid_valve->write(true);
				solenoid_state = 1;
				xTimerStart(controlValveTimerOpen, 0);
				ITM_write("\nvalve open!\n");
			}
		}
		vTaskDelay(500);
	}
}

static void vTaskMODBUS(void *pvParams){

	int old_temp = 0, old_rh = 0, old_co2 = 0;

	char buff[40];
	SensorData sensor_event;

	while(1)  {
		sensor_event.temp = modbus.get_temp();
		vTaskDelay(pdMS_TO_TICKS(50));
		sensor_event.rh = modbus.get_rh();
		vTaskDelay(pdMS_TO_TICKS(50));
		sensor_event.co2 = modbus.get_co2();
		vTaskDelay(pdMS_TO_TICKS(50));

        if( xSemaphoreTake( xSemaphoreMQTT, 0 ) == pdTRUE) {
        	sprintf(buff, "\n\rtemp: %d\n\rrh: %d\n\rco2: %d", sensor_event.temp, sensor_event.rh, sensor_event.co2);
			sysMutex.lock();
			ITM_write(buff);
			sysMutex.unlock();
			vTaskDelay(500);
			xQueueSend(hq, &sensor_event, 0); // LCD queue
        	xQueueSend(mq, &sensor_event, 0); //mqtt queue
        }
        else if (old_temp != sensor_event.temp || old_rh != sensor_event.rh || old_co2 != sensor_event.co2){
        	sprintf(buff, "\n\rtemp: %d\n\rrh: %d\n\rco2: %d", sensor_event.temp, sensor_event.rh, sensor_event.co2);
			sysMutex.lock();
			ITM_write(buff);
			sysMutex.unlock();
			xQueueSend(hq, &sensor_event, 0); // LCD queue

			old_temp = sensor_event.temp;
			old_rh = sensor_event.rh;
			old_co2 = sensor_event.co2;
        }

	}
}

/* callback for publish mqtt timer */
void vMQTTTimerCallback( TimerHandle_t xTimer ) {
    if( xSemaphoreGive( xSemaphoreMQTT ) != pdTRUE )
    {
    	sysMutex.lock();
    	ITM_write("xSemaphoreMQTT failed!");
    	sysMutex.unlock();
    } else {
    	sysMutex.lock();
    	ITM_write("xSemaphoreMQTT given!");
    	sysMutex.unlock();
    }
}
#if 0
/* callback for valve init */
void vValveTimerCallBack( TimerHandle_t xTimer ) {
    if( xSemaphoreGive( xSemaphoreValve ) != pdTRUE )
    {
    	sysMutex.lock();
    	ITM_write("xSemaphoreValve failed!");
    	sysMutex.unlock();
    } else {
    	sysMutex.lock();
    	ITM_write("xSemaphoreValve given!");
    	sysMutex.unlock();
    }
}
#endif

/* callback for valve open-close */
void vValveCloseTimerCallBack( TimerHandle_t xTimer ) {
    solenoid_valve->write(false);
    solenoid_state = 0;
}

static void vTaskISR(void * pvParameters){
	int e;
	while(1){
		if (xQueueReceive(iq, &e, 0) == pdTRUE){
			if (e == 1){
				if(encoder_B->read()){
					sysMutex.lock();
					menu.event(MenuItem::up);
					sysMutex.unlock();
				}
				if(encoder_A->read()){
					sysMutex.lock();
					menu.event(MenuItem::down);
					sysMutex.unlock();
				}
			}
			if (e == 2){
				if(menu.getIndex()==1 ) menu.event(MenuItem::ok);
			}
		}
	}
}
int main(void) {

 	prvHardwareSetup();
	heap_monitor_setup();

	/* Configure pins and ports for Rotary Encoder and valve as input, pullup, and inverted */
	encoder_A = new DigitalIoPin(0, 5, DigitalIoPin::pullup, true);
	encoder_B = new DigitalIoPin(0, 6, DigitalIoPin::pullup, true);
	encoder_Button = new DigitalIoPin(1, 8, DigitalIoPin::pullup, true);

	/* configure interrupts for those buttons */
	//enable_interrupt(IRQ number, NVIC priority, port, pin)

	encoder_A->enable_interrupt(0, 0, 0, 5);
	encoder_Button->enable_interrupt(2, 0, 1, 8);

	/* create a queue of max 10 events */

	hq = xQueueCreate(10, sizeof(dataevent));
	mq = xQueueCreate(10, sizeof(SensorData));
	iq = xQueueCreate(15, sizeof(int));

	/* setting things up for sending data to MQTT in invervals */
	xSemaphoreMQTT = xSemaphoreCreateBinary();
	sendToMQTTTimer = xTimerCreate("Send to MQTT", MQTTInterval, pdTRUE, (void *)0, vMQTTTimerCallback);
	xTimerStart(sendToMQTTTimer,0);

	/* setting things up for allowing valve control */
	xSemaphoreValve = xSemaphoreCreateBinary();
#if 0
	controlValveTimer = xTimerCreate("Valve control", valveInterval, pdTRUE, (void *)0, vValveTimerCallBack);
	xTimerStart(controlValveTimer,0);
#endif
	controlValveTimerOpen = xTimerCreate("Valve-close control", valveIntervalOpen, pdFALSE, (void *)0, vValveCloseTimerCallBack);
	/* task MQTT */
	xTaskCreate( prvMQTTTask,
	                 "MQTT task",
	                 ((configMINIMAL_STACK_SIZE)+512),
	                 NULL,
	                 tskIDLE_PRIORITY +1UL,
	                 NULL );

	/* task LCD */
	xTaskCreate(vTaskLCD, "LCD_Task",
			((configMINIMAL_STACK_SIZE)+512), NULL, tskIDLE_PRIORITY + 1UL,
			(TaskHandle_t *) NULL);

	/* task upon interrupt */
	xTaskCreate(vTaskISR, "handle ISR events",
				((configMINIMAL_STACK_SIZE)+256), NULL, tskIDLE_PRIORITY + 1UL,
				(TaskHandle_t *) NULL);

	/* task measurement modbus */
	xTaskCreate(vTaskMODBUS, "Measuring",
			((configMINIMAL_STACK_SIZE)+256), NULL, tskIDLE_PRIORITY + 1UL,
			(TaskHandle_t *) NULL);

	vTaskStartScheduler();

	/* never arrive here */
    return 1 ;
}
