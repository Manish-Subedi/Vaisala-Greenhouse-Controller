/*
 * heap_lock_monitor.h
 *
 *  Created on: 13.1.2020
 *      Author: keijo
 */

#ifndef HEAP_LOCK_MONITOR_H_
#define HEAP_LOCK_MONITOR_H_

#include <reent.h>

#define DEFAULT_STACK_SIZE_ 512

#ifdef __cplusplus
extern "C" {
#endif

extern unsigned int heapTotalSize_;
extern unsigned int heapAmountUsed_;
extern float heapPercentageUsed_;

void heap_monitor_setup(void);

void heap_monitor_update(void);

void __malloc_lock (struct _reent *reent);

void __malloc_unlock (struct _reent *reent);

#ifdef __cplusplus
}
#endif

#endif /* HEAP_LOCK_MONITOR_H_ */
