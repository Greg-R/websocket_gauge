#include "frozen/frozen.h"
#include "mgos.h"
#include "mgos_http_server.h"
#include "mgos_gpio.h"
#include "mgos_rpc.h"
#include "mcpwm_config.h"
#include "freertos/semphr.h"
#include "pcnt_init.h"

//  The incoming new meter position is written to this queue by the RPC handler.
xQueueHandle meter_set;

//  This is a helper function to initialize the GPIO12 for
//  use as a direction control signal.
void dir_control() {
	gpio_config_t gp;
	gp.intr_type = GPIO_INTR_DISABLE;
	gp.mode = GPIO_MODE_OUTPUT;
	gp.pin_bit_mask = GPIO_SEL_12;
	gp.pull_down_en = GPIO_PULLDOWN_ENABLE;
	gpio_config(&gp);
}

// This task performs the initial "calibration" by slamming
// the needle against the low stop.
static void calibrate(void *arg) {
	//  This task implements a delay long enough to
	//  guarantee the needle is slammed against the low stop.
//	uint32_t intr_status;
	dir_control();
	//  Set GPIO dir bit for cal counter-clockwise rotation.
	gpio_set_level(GPIO_NUM_12, 0);
	//  Configure the MCPWM.
	set_mcpwm_config();
	//  Set the count limit such that it is guaranteed the stepper
	//  slams the left rail.  See macro PCNT_H_LIM_VAL in pcnt_init.h.
	pcnt_init();
	//  Initialize the PWM.  This will start the PWM.
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	//  This blocks this task until the Counter's high limit interrupt runs the ISR and writes to the queue.
	xSemaphoreTake(counterSemaphore, portMAX_DELAY);
//	xQueueReceive(pcnt_evt_queue, &intr_status, portMAX_DELAY);
	//  Immediately stop the PWM and reset the counter.
	mcpwm_stop(MCPWM_UNIT_0, 0);
	pcnt_counter_pause(PCNT_TEST_UNIT);
	pcnt_counter_clear(PCNT_TEST_UNIT);
	printf("Complete calibration routine.\n");
	vTaskDelete(NULL);  //  This task is no longer required after completion of calibration.
}

//  This task determines the current needle position, and
//  then fires the necessary number of pulses with the correct
//  direction control to get the needle to the new position.
//  Assume the range of the meter is 0-1200.
static void set_meter(void * arg) {
	int16_t delta;
	int16_t current_position, new_position, delta_abs;
	uint32_t intr_status;
	esp_err_t stop_pwm;
	current_position = 0;  //  Initial value; this should be re-written.

	for(;;) {
//		printf("Top of the set_meter for loop.\n");
		//  Block the loop with a queue.
		//  The queue should contain the new value of the meter.
		//  This unblocks when the RPC is done via Websocket.
		xQueueReceive(meter_set, &new_position, portMAX_DELAY);
		pcnt_counter_pause(PCNT_TEST_UNIT);
		//  Determine how many pulses need to be emitted.
		//  delta is signed since direction is required.
		delta = new_position - current_position;
		current_position = new_position;
		if (delta < 0) {
			gpio_set_level(GPIO_NUM_12, 0);
		}
		else {
			gpio_set_level(GPIO_NUM_12, 1);
		}
		//  Set the pulse counter high level to delta.
		//  Clear counter back to zero.
		delta_abs = abs(delta);
		//  If the delta is zero, don't start the PWM!
		if (delta_abs != 0) {
//			printf("Setting event value to %d.\n", delta_abs);
			pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_H_LIM, delta_abs);
			//  The following commands are important!
			//  It's possible the counter has "shadow" registers.
			//  The counter uses the initilization value of 1200 if these
			//  commands are not used.
			pcnt_counter_clear(PCNT_TEST_UNIT);
			pcnt_counter_resume(PCNT_TEST_UNIT);
			mcpwm_start(MCPWM_UNIT_0, 0);
			//  This unblocks when the counter high limit interrupt fires.
			//  Could use a semaphore here.  Is it more efficient?
			xSemaphoreTake(counterSemaphore, portMAX_DELAY);
//			xQueueReceive(pcnt_evt_queue, &intr_status, portMAX_DELAY);  //  This will block until update received.
//			printf("Stopping PWM in set_meter.\n");
			stop_pwm = mcpwm_stop(MCPWM_UNIT_0, 0);
		}
	}
}

// Callback function pointer for Websocket to web page.
static void cb(struct mg_rpc_request_info *ri, void * cb_arg,
		struct mg_rpc_frame_info *fi, struct mg_str args) {
	int32_t mtr;
	if (json_scanf(args.p, args.len, ri->args_fmt, &mtr) == 1) {
		//		mgos_gpio_write(led, state);
		//  Can write to queue and send from queue here?
		xQueueSend(meter_set, &mtr, portMAX_DELAY);
	}
//	mg_rpc_send_responsef(ri, "true");
	(void)cb_arg;
	(void)fi;
}

enum mgos_app_init_result mgos_app_init(void) {
//	xQueueHandle meter_set;
//	xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
	//  This queue is used for un-blocking and also can send interrupt type information.
//	pcnt_evt_queue = xQueueCreate(2, sizeof(int32_t));
	counterSemaphore = xSemaphoreCreateBinary();
	//  This queue of size one stores the NEW meter setting;
	meter_set = xQueueCreate(400, sizeof(int16_t));

	struct mg_rpc *c = mgos_rpc_get_global();
	//	mgos_gpio_set_mode(17, MGOS_GPIO_MODE_OUTPUT);
	//	mgos_gpio_set_mode(18, MGOS_GPIO_MODE_OUTPUT);
	//	mgos_gpio_set_mode(19, MGOS_GPIO_MODE_OUTPUT);
	mg_rpc_add_handler(c, "meter", "{mtr: %d}", cb, NULL);

	//  This function runs only once at start-up.
	xTaskCreate(calibrate, "calibrate", 4096, NULL, 5, NULL); //  This gets priority and then deletes itself.

	//  This task waits for a queue the meter_set to unblock, and then gets the data
	//  from the queue written by RPC, and uses this to move the meter to the desired position.
	xTaskCreate(set_meter, "set_meter", 4096, NULL, 4, NULL);

	return MGOS_APP_INIT_SUCCESS;
}
