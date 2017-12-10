//
//  This file initializes and starts the Pulse Counter.

#include "driver/pcnt.h"

#define PCNT_TEST_UNIT      PCNT_UNIT_0
//  The high limit needs to be set to high value to "calibrate" the needle
//  against the left stop.
#define PCNT_H_LIM_VAL      2500
#define PCNT_INPUT_SIG_IO   4  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  5  // Control GPIO HIGH=count up, LOW=count down
//#define PCNT_THRESH0_VAL    100
//#define PCNT_THRESH1_VAL    5

SemaphoreHandle_t counterSemaphore;

// Decode what PCNT's unit originated an interrupt
// and pass this information together with the event type
// the main program using a queue.
//
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
	//  Can probably be simplified; really only acts as a blocker in the task.
//	BaseType_t xHigherPriorityTaskWoken;
//	uint32_t intr_status = PCNT.int_st.val;
	//  Clear the high limit interrupt.
//	PCNT.int_clr.val = BIT(0);  // High limit is interrupt bit 0.  Threshold 0 is bit 2.
	//  Will this clear everything?
	PCNT.int_clr.val = BIT(0);
//	PCNT.int_clr.val = BIT(1);
	PCNT.int_clr.val = BIT(2);
//	PCNT.int_clr.val = BIT(3);

xSemaphoreGiveFromISR(counterSemaphore, NULL);
//	    xQueueSendFromISR(pcnt_evt_queue, &intr_status, NULL);
	//xQueueOverwriteFromISR(pcnt_evt_queue, &intr_status, NULL);
}

static void pcnt_init(void)
{
	/* Prepare configuration for the PCNT unit */
	pcnt_config_t pcnt_config = {
			// Set PCNT input signal and control GPIOs
			.pulse_gpio_num = PCNT_INPUT_SIG_IO,  //  This is GPIO input to counter!!!  GPIO4
			.ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
			.channel = PCNT_CHANNEL_0,
			.unit = PCNT_TEST_UNIT,
			.pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
			.neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
			// What to do when control input is low or high?
			.lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
			.hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
			// Set the maximum limit value to watch.
			.counter_h_lim = PCNT_H_LIM_VAL,
	};
	// Initialize PCNT unit.
	pcnt_unit_config(&pcnt_config);

	/* Configure and enable the input filter */
	pcnt_set_filter_value(PCNT_TEST_UNIT, 700);
	pcnt_filter_enable(PCNT_TEST_UNIT);

	/* Enable events on maximum value. Disable zero!!! */
	pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
	pcnt_event_disable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);

	// Enable events on threshold 0 value.
	pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);

	/* Initialize PCNT's counter */
	pcnt_counter_pause(PCNT_TEST_UNIT);
	pcnt_counter_clear(PCNT_TEST_UNIT);

	/* Register ISR handler and enable interrupts for PCNT unit */
	pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, NULL);
	pcnt_intr_enable(PCNT_TEST_UNIT);

	// Everything is set up, now go to counting.
	pcnt_counter_resume(PCNT_TEST_UNIT);
}
