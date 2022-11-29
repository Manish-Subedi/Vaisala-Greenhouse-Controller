/*
 * heap_lock_monitor.c
 *
 *  Created on: 13.1.2020
 *      Author: keijo
 */
#include "FreeRTOS.h"
#include "task.h"

#include "heap_lock_monitor.h"

// global pointer to highest used address of heap (maintained by newlib)
extern void *__end_of_heap;

extern char _pvHeapStart; // Pointer to start of heap (from linker script)

// This is defined when MCUXpresso style heap is used
// Optional pointer to end of heap (from linker script)
extern char _pvHeapLimit __attribute__((weak));

// This is needed to estimate heap size when LPCXpresso style heap is used
// Pointer to top of first RAM region
extern char __top_RAM;

unsigned int heapTotalSize_;
unsigned int heapAmountUsed_;
float heapPercentageUsed_;

extern const uint8_t FreeRTOSDebugConfig[];

void heap_monitor_setup(void)
{
	/* fool the linker to keep data that task aware debugger needs */
	heapTotalSize_ = FreeRTOSDebugConfig[0];

	/* calculate actual heap size */
	if(&_pvHeapLimit) {
		heapTotalSize_ = (int)(&_pvHeapLimit) - (int)(&_pvHeapStart);
	}
	else {
		heapTotalSize_ = (int)(&__top_RAM) - (int)(&_pvHeapStart) - DEFAULT_STACK_SIZE_;
	}
}

void heap_monitor_update(void)
{
	heapAmountUsed_ = (int)__end_of_heap - (int)(&_pvHeapStart);
	heapPercentageUsed_ = heapAmountUsed_ * 100.0 / heapTotalSize_;
}


void __malloc_lock (struct _reent *reent)
{
	(void) reent;
	vTaskSuspendAll();
}

void __malloc_unlock (struct _reent *reent)
{
	(void) reent;
	heap_monitor_update();
	xTaskResumeAll();
}


