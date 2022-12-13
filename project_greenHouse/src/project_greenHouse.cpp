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

#include "./mqtt_demo/MQTT_custom.h"

static SimpleMenu menu;
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

Fmutex sysMutex;
QueueHandle_t hq; //sensor data
QueueHandle_t mq; // mqtt data
SemaphoreHandle_t xSem;

modbusConfig modbus;
char msg[40];

/* variables to read from MODBUS sensors */
struct SensorData {
	int temp;
	int rh;
	int co2;
	uint64_t time_stamp;
};

/* Interrupt handlers must be wrapped with extern "C" */

extern "C"{
/* ISR for encode rotator A */
void PIN_INT0_IRQHandler(void)
{
	Board_LED_Toggle(2);
	/* this must be set to true so that the context switch
	 * gets back to the ongoing task
	 * */
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	/* create an event and send to the queue upon an interrupt */

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
	portBASE_TYPE xHigherPriorityTaskWoken = pdTRUE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
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
void prvCreateMQTTConnectionWithBroker( MQTTContext_t * pxMQTTContext,
                                               NetworkContext_t * pxNetworkContext );
void prvMQTTSubscribeWithBackoffRetries( MQTTContext_t * pxMQTTContext );
void prvMQTTPublishToTopic( char *, MQTTContext_t * pxMQTTContext );
MQTTStatus_t MQTT_ProcessLoop( MQTTContext_t * pContext,
                               uint32_t timeoutMs );

}

uint8_t ucSharedBuffer[ mqttexampleSHARED_BUFFER_SIZE ];
uint32_t ulGlobalEntryTimeMs;
uint16_t usSubscribePacketIdentifier;
uint16_t usUnsubscribePacketIdentifier;

/* @brief task controls MQTT interface */
static void prvMQTTTask( void * pvParameters )
{
    uint32_t ulPublishCount = 0U, ulTopicCount = 0U;
    const uint32_t ulMaxPublishCount = 5UL;
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
        	sprintf(buff, "field1=%d&field2=%d&field3=%d&field4=%ld", recv.co2, recv.rh, recv.temp, xTaskGetTickCount());
        	prvMQTTPublishToTopic( buff, &xMQTTContext );
        	vTaskDelay(500);
        }
#if 0
        /******************** Unsubscribe from the topic. *********************/
        LogInfo( ( "Unsubscribe from the MQTT topic %s.", mqttexampleTOPIC ) );
        prvMQTTUnsubscribeFromTopic( &xMQTTContext );

        /* Process the incoming packet from the broker. */
        xMQTTStatus = MQTT_ProcessLoop( &xMQTTContext,
                                        mqttexamplePROCESS_LOOP_TIMEOUT_MS );
        configASSERT( xMQTTStatus == MQTTSuccess );

        /**************************** Disconnect. *****************************/

        /* Send an MQTT Disconnect packet over the connected TCP socket.
         * There is no corresponding response for a disconnect packet. After
         * sending the disconnect, the client must close the network connection. */
        LogInfo( ( "Disconnecting the MQTT connection with %s.",
                   democonfigMQTT_BROKER_ENDPOINT ) );
        xMQTTStatus = MQTT_Disconnect( &xMQTTContext );
        configASSERT( xMQTTStatus == MQTTSuccess );

        /* Close the network connection. */
        xNetworkStatus = Plaintext_FreeRTOS_Disconnect( &xNetworkContext );
        configASSERT( xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS );

        /* Reset SUBACK status for each topic filter after completion of
         * subscription request cycle. */
        for( ulTopicCount = 0; ulTopicCount < mqttexampleTOPIC_COUNT; ulTopicCount++ )
        {
            xTopicFilterContext[ ulTopicCount ].xSubAckStatus = MQTTSubAckFailure;
        }

        /* Wait for some time between two iterations to ensure that we do not
         * bombard the MQTT broker. */
        LogInfo( ( "prvMQTTDemoTask() completed an iteration successfully. " ) );
                 //  "Total free heap is %u.", xPortGetFreeHeapSize() ) );
        LogInfo( ( "Demo completed successfully." ) );
        LogInfo( ( "Short delay before starting the next iteration.... \r\n" ) );
        vTaskDelay( mqttexampleDELAY_BETWEEN_DEMO_ITERATIONS );
#endif
    }
}


//LCD configuration
DigitalIoPin *rs = new DigitalIoPin(0, 29, DigitalIoPin::output);
DigitalIoPin *en = new DigitalIoPin(0, 9, DigitalIoPin::output);
DigitalIoPin *d4 = new DigitalIoPin(0, 10, DigitalIoPin::output);
DigitalIoPin *d5 = new DigitalIoPin(0, 16, DigitalIoPin::output);
DigitalIoPin *d6 = new DigitalIoPin(1, 3, DigitalIoPin::output);
DigitalIoPin *d7 = new DigitalIoPin(0, 0, DigitalIoPin::output);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

IntegerEdit *co2_= new IntegerEdit(&lcd, std::string("CO2"), 10000, 0, 1);
IntegerEdit *co2_t= new IntegerEdit(&lcd, std::string("CO2 target"), 10000, 0, 1);
IntegerEdit *rh_= new IntegerEdit(&lcd, std::string("RH"), 100, 0, 1);
IntegerEdit *temp_= new IntegerEdit(&lcd, std::string("Temp"), 60, -40, 1);


/* task to wait on the queue event from ISR and print it */
static void vTaskLCD(void *pvParams){
	menu.addItem(new MenuItem(co2_));
	menu.addItem(new MenuItem(co2_t));
	menu.addItem(new MenuItem(rh_));
	menu.addItem(new MenuItem(temp_));

	lcd.begin(16, 2);
	//lcd.setCursor(0,0);
	co2_->setValue(0);
	co2_t->setValue(0);
	rh_->setValue(0);
	temp_->setValue(0);

	MenuItem::show;

	SensorData e;
	// 1. display values to LCD UI
	for( ;; ){

		// 2. take semaphore and update
		if(xQueueReceive(hq, &e, portMAX_DELAY)){     // receive data sensors from queue
			co2_->setValue(e.co2);
			rh_->setValue(e.rh);
			temp_->setValue(e.temp);
			menu.event(MenuItem::show);

		}
	}

}

static void vTaskMODBUS(void *pvParams){

	char buff[40];
	SensorData sensor_event;

	while(1)  {
		sensor_event.temp = modbus.get_temp();
		sensor_event.rh = modbus.get_rh();
		sensor_event.co2 = modbus.get_co2();

		sprintf(buff, "\n\rtemp: %d\n\rrh: %d\n\rco2: %d", sensor_event.temp, sensor_event.rh, sensor_event.co2);
		sysMutex.lock();
		ITM_write(buff);
		sysMutex.unlock();
		//give semaphore to LCD task here
		snprintf(msg, 40, "\n\rtemp: %d\n\rrh: %d\n\rco2: %d", sensor_event.temp, sensor_event.rh, sensor_event.co2);

		xQueueSend(hq, &sensor_event, 0);
		vTaskDelay(500);
		xQueueSend(mq, &sensor_event, 0);
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
	encoder_Button.enable_interrupt(2, 0, 1, 9);

	/* create a counting sempahore of max 5 events */
	xSem = xSemaphoreCreateCounting(10, 0);

	/* create a queue of max 10 events */
	hq = xQueueCreate(10, sizeof(int));
	mq = xQueueCreate(10, sizeof(SensorData));
	/* task MQTT */
	xTaskCreate( prvMQTTTask,          /* Function that implements the task. */
	                 "DemoTask",               /* Text name for the task - only used for debugging. */
	                 democonfigDEMO_STACKSIZE, /* Size of stack (in words, not bytes) to allocate for the task. */
	                 NULL,                     /* Task parameter - not used in this case. */
	                 tskIDLE_PRIORITY,         /* Task priority, must be between 0 and configMAX_PRIORITIES - 1. */
	                 NULL );                   /* Used to pass out a handle to the created task - not used in this case. */

	/* task LCD */
	xTaskCreate(vTaskLCD, "LCD_Task",
			((configMINIMAL_STACK_SIZE)+128), NULL, tskIDLE_PRIORITY + 1UL,
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
