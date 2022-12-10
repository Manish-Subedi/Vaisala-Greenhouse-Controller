/*
 * DigitalIoPin.cpp
 *
 *  Created on: 31.1.2016
 *      Author: krl
 *      Modified: Manish 30.10.2022 [ Added GPIO interrupt config ]
 */

#include "DigitalIoPin.h"
#include "chip.h"
#include "FreeRTOS.h"

DigitalIoPin::DigitalIoPin(int port_, int pin_, pinMode mode, bool invert) : port(port_), pin(pin_), inv(invert) {
	if(mode == output) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, IOCON_MODE_INACT | IOCON_DIGMODE_EN);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, port, pin);
	}
	else {
		uint32_t pm = IOCON_DIGMODE_EN;

		if(invert) pm |= IOCON_INV_EN;

		if(mode == pullup) {
			pm |= IOCON_MODE_PULLUP;
		}
		else if(mode == pulldown) {
			pm |= IOCON_MODE_PULLDOWN;
		}

		Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, pm);
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
	}
}

DigitalIoPin::~DigitalIoPin() {
	// TODO Auto-generated destructor stub
}

bool DigitalIoPin::read() {
	return Chip_GPIO_GetPinState(LPC_GPIO, port, pin);
}

void DigitalIoPin::write(bool value) {
	return Chip_GPIO_SetPinState(LPC_GPIO, port, pin, inv ^ value);
}


void DigitalIoPin::GPIO_interrupt_init() {
	/* initialise PININT driver */
	Chip_PININT_Init(LPC_GPIO_PIN_INT);
	/* enable PININT clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PININT);
	/* Reset the PININT block */
	Chip_SYSCTL_PeriphReset(RESET_PININT);
}
/*
 * @brief configure pin interrupts
 * @param Pin-Interrupt channel number
 * @param Interrupt priority
 * @return nothing
 */
void DigitalIoPin::enable_interrupt(const int& pin_irq_index, int priority, int port, int pin) {
	/* Configure interrupt channel for the GPIO pin in INMUX block
	* GPIO Pin Interrupt Pin Select (sets PINTSEL register) */
	Chip_INMUX_PinIntSel(pin_irq_index, port, pin);
	/* set second highest interrupt priority for the channel
	 * set priority to 0 for highest priority
	 * */
	NVIC_SetPriority(PIN_INT0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + priority);
	/* Clear interrupt status in Pin interrupt block */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(pin_irq_index));
	/* Configure channel interrupt as edge sensitive */
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(pin_irq_index));
	/* and falling edge interrupt */
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(pin_irq_index));
	/* clears the pending bit of an external interrupt. */
	NVIC_ClearPendingIRQ((IRQn_Type)(PIN_INT0_IRQn + pin_irq_index));
	/* Enable interrupt in the NVIC */
	NVIC_EnableIRQ((IRQn_Type)(PIN_INT0_IRQn + pin_irq_index));
}
