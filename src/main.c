#include "frozen/frozen.h"
#include "mgos.h"
#include "mgos_http_server.h"
#include "mgos_gpio.h"
#include "mgos_rpc.h"
#include "mcpwm_config.h"
#include "pcnt_init.h"

#define INITIAL_DIRECTION 0

int gpio_set_dir;

//  This variable is the gauge current set point.
uint32_t gauge_value;

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

// This function performs the initial "calibration" by slamming
// the needle against the low stop.
static void calibrate(void *arg) {
	//  The stepper pulses should have already commenced by the
	//  mcpwm_config task.  This task simply implements a delay
	//  long enough to guarantee the needle is slammed against
	//  the low stop.
	uint32_t intr_status;
	dir_control();
	//  Set GPIO dir bit for cal counter-clockwise rotation.
	gpio_set_level(GPIO_NUM_12, 0);
	//  Configure the MCPWM.
	set_mcpwm_config();
	//  Set the count limit such that it is guaranteed the stepper
	//  slams the left rail.  See macro PCNT_H_LIM_VAL in pcnt_init.h.
	pcnt_example_init();
//	pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_H_LIM, 1200);
	//  Resume the counter.
//	pcnt_counter_resume(PCNT_TEST_UNIT);
	//  Initialize the PWM.  This will start the PWM?
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
//	mcpwm_start(MCPWM_UNIT_0, 0);
	//  Allow enough time to hit the stop.  This needs to block everything else!
//	vTaskDelay(2000);
	xQueueReceive(pcnt_evt_queue, &intr_status, portMAX_DELAY);  //  This will block until update received.
	mcpwm_stop(MCPWM_UNIT_0, 0);
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);
	printf("Complete calibration routine.\n");
	vTaskDelete(NULL);
}

//  This task manages the stepper motor gauge.
static void gauge_controller(void *arg) {
	int16_t counter, high_limit;
	uint32_t intr_status;
	esp_err_t stop_pwm, set_pwm_low;
	gpio_set_level(GPIO_NUM_12, gpio_set_dir);

	gpio_set_dir = INITIAL_DIRECTION;
	//	int16_t count = 0;
	//  The only purpose here is to shutdown the PWM upon the Counter high limit interrupt.
	//  This task should become active after every high limit interrupt.
	while (1) {
		printf("Top of the gauge_controller while loop.\n");
		// pcnt_evt_t evt;

		//  This can be replaced by a semaphore.
		xQueueReceive(pcnt_evt_queue, &intr_status, portMAX_DELAY);  //  This will block until update received.
		//  Shutdown the PWM.
//		printf("GC intr_status %d\n", intr_status);
//		intr_status = PCNT.int_st.val;
		printf("GC intr_status after ISR = %d\n", intr_status);
		stop_pwm = mcpwm_stop(MCPWM_UNIT_0, 0);
		//  Must use set_pwm_high to reactivate!
//		set_pwm_low = mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
//		set_pwm_low = mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
		//		printf("Shutting down the PWM in the gauge controller.\n");
		//  The counter should have reset it self.  Print out the value.
		//		pcnt_get_counter_value(PCNT_TEST_UNIT, &counter);
		pcnt_get_event_value(PCNT_TEST_UNIT, PCNT_EVT_H_LIM, &high_limit);
		printf("From gauge_controller, the inbound high limit is %d.\n", high_limit);
		//  Flip the direction control
		//		gpio_set_dir = ~gpio_set_dir;
		//		gpio_set_level(GPIO_NUM_12, gpio_set_dir);

		//		pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
		//		printf("H_LIM EVT and interrupt number is %d.\n", evt.unit);

		//		vTaskDelay(500);
		//		mcpwm_start(MCPWM_UNIT_0, 0);
	}
}

//  This function determines the current needle position, and
//  then fires the necessary number of pulses with the correct
//  direction control to get the needle to the new position.
//  Assume the range of the meter is 0-1200.
static void set_meter(void * arg) {
	//    int16_t count = 0;
	int16_t delta, high_limit;

	uint32_t intr_status;
	esp_err_t stop_pwm;
	gpio_set_level(GPIO_NUM_12, gpio_set_dir);

	gpio_set_dir = INITIAL_DIRECTION;

	//  Since this is a static function, is this value static too?
	int16_t current_position, new_position, delta_abs, counter;
	current_position = 0;  //  Initial value; this should be re-written.
	//  Block the test with a queue.
	//  The queue should contain the new value of the meter.
	for(;;) {
		printf("Top of the set_meter for loop.\n");
		xQueueReceive(meter_set, &new_position, portMAX_DELAY);
		//		printf("new_position = %d, current_position = %d.\n", new_position, current_position);
		//  Shutdown the PWM.
//		mcpwm_stop(MCPWM_UNIT_0, 0);
//		mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
		pcnt_counter_pause(PCNT_TEST_UNIT);
		//  Get the pulse count value.
		//This is not going to work well.  The counter resets to zero at any event.
//		pcnt_get_counter_value(PCNT_TEST_UNIT, &counter);
//		printf("The counter value at meter set is %d.\n", counter);
//		pcnt_get_event_value(PCNT_TEST_UNIT, PCNT_EVT_H_LIM, &high_limit);
//		printf("From gauge_controller, the inbound high limit is %d.\n", high_limit);
		//  Determine how many pulses need to be emitted.
		//  delta is signed since direction is required.
		delta = new_position - current_position;
		current_position = new_position;  //  Saved in static function?
		if (delta < 0) {
			gpio_set_level(GPIO_NUM_12, 0);
		}
		else {
			gpio_set_level(GPIO_NUM_12, 1);
		}
		//  Set the pulse counter high level to delta.
		//  Clear counter back to zero.  Does this fire an interrupt???
		//  Set new event value.
		//	pcnt_set_event_value(pcnt_unit_t unit, pcnt_evt_type_t evt_type, int16_t value)
		delta_abs = abs(delta);
		//  Will absolute value work?
		//      printf("Delta is %d.\n", delta);
		//		pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_H_LIM, delta_abs);
		//  Let's see if it is getting set correctly!
		//  If the delta is zero, don't start the PWM!
		if (delta_abs != 0) {
			printf("Setting event value to %d.\n", delta_abs);
			pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_H_LIM, delta_abs);
			//  The following commands are important!
			//  It's possible the counter has "shadow" registers.
			//  The counter uses the initilization value of 1200 if these
			//  commands are not used.
			pcnt_counter_clear(PCNT_TEST_UNIT);
			pcnt_counter_resume(PCNT_TEST_UNIT);
			mcpwm_start(MCPWM_UNIT_0, 0);
			xQueueReceive(pcnt_evt_queue, &intr_status, portMAX_DELAY);  //  This will block until update received.
			printf("Stopping PWM in set_meter.\n");
			stop_pwm = mcpwm_stop(MCPWM_UNIT_0, 0);
		}
		//  Block here until Counter zero limit interrupt is fired.  Use a semaphore.
//		xSemaphoreTake(zero_wait_semaphore, portMAX_DELAY);
		//		pcnt_get_event_value(PCNT_TEST_UNIT, PCNT_EVT_H_LIM, &highlimitread);
		//		printf("The read-back of the high limit is %d.\n", highlimitread);
	}
}

// Callback function pointer for Websocket to web page.
static void cb(struct mg_rpc_request_info *ri, void * cb_arg,
		struct mg_rpc_frame_info *fi, struct mg_str args) {
	int32_t state, led;
	if (json_scanf(args.p, args.len, ri->args_fmt, &state, &led) == 2) {
		//  if (json_scanf(args.p, args.len, ri->args_fmt, &state) == 1) {
//		mgos_gpio_write(led, state);
		//  Can write to queue and send from queue here?
		xQueueSend(meter_set, &led, portMAX_DELAY);
	}
	mg_rpc_send_responsef(ri, "true");
	(void)cb_arg;
	(void)fi;
}

enum mgos_app_init_result mgos_app_init(void) {

	//	pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
	//  This queue is used for un-blocking and also sends interrupt type information.
	pcnt_evt_queue = xQueueCreate(1, sizeof(int32_t));
	//	pcnt_example_init();
zero_wait_semaphore = xSemaphoreCreateBinary();
	//  This queue of size one stores the NEW meter setting;
	//
	meter_set = xQueueCreate(5, sizeof(int16_t));

	struct mg_rpc *c = mgos_rpc_get_global();
	mgos_gpio_set_mode(17, MGOS_GPIO_MODE_OUTPUT);
	mgos_gpio_set_mode(18, MGOS_GPIO_MODE_OUTPUT);
	mgos_gpio_set_mode(19, MGOS_GPIO_MODE_OUTPUT);
	mg_rpc_add_handler(c, "ledtoggle", "{state: %d, led: %d}", cb, NULL);

	//  For now, this simply sets up the PWM so it starts generating pulses to be counted by PCNTer.
	//  Gauge should get slammed to one direction and stay.
	//	xTaskCreate(mcpwm_config, "mcpwm_example_config", 4096, NULL, 5, NULL);
	//  This function runs only once at start-up.
	xTaskCreate(calibrate, "calibrate", 4096, NULL, 5, NULL); //  This gets priority and then deletes itself.
//	xTaskCreate(gauge_controller, "gauge_controller", 4096, NULL, 4, NULL);  //

	//  This task waits for a queue to unblock, and then gets the data
	//  from the queue, and uses this to move the meter to the desired position.
	xTaskCreate(set_meter, "set_meter", 4096, NULL, 5, NULL);

	return MGOS_APP_INIT_SUCCESS;
}
