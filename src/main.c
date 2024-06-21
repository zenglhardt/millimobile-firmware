/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/pm/pm.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/time_units.h>

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
	#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),
/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

int adc_err = 0; // ADC errors
int32_t adc_val = 0; // Latest ADC value
uint16_t adc_val_buf; // ADC buffer to use for mV conversion 
struct adc_sequence adc_seq = {
	.buffer = &adc_val_buf,
	.buffer_size = sizeof(adc_val_buf)
};

// Constants
#define FS_HZ 500 // Sampling frequency [Hz]
#define N 32 // Number of samples in ADC buffer (larger ==> smoother ==> slower)
#define M 8 // Number of samples in frequency buffer (larger ==> smoother ==> slower)
#define ADC_COMMS_CHAN 7 // ADC channel to use for communication
#define MIN_MV_RANGE 0 // Minimum peak-to-peak mV needed to establish communication
#define DEMOD_THRESH 50 // Threshold in [0, 100] to use as demodulation threshold

#define COMM_INTERVAL_MS (1000 / FS_HZ) // Interval in milliseconds

void comm_timer_handler(struct k_timer *dummy);
void comm_sampling_work_handler(struct k_work *work);

K_TIMER_DEFINE(comm_timer, comm_timer_handler, NULL);
K_WORK_DEFINE(comm_work, comm_sampling_work_handler);

void comm_timer_handler(struct k_timer *dummy) {
	k_work_submit(&comm_work);
}

// Buffer stats
typedef struct buffer_stats {
	float avg;
	int max;
	int min;
	int range;
} buffer_stats;

// ADC ring buffer of size N
int adc_buf[N] = {0};
int n = 0;
struct buffer_stats adc_stats;

// FSK ring buffer
int f_buf[M] = {0};
int m = 0;
struct buffer_stats f_stats;

// Demodulation vars
int norm = 0;
int64_t t_rise_prev = 0;
bool state = 0;

// Update buffer stats
void update_buffer_stats(int *buffer, int len, buffer_stats *stats) {
	int max = -100000;
	int min = 100000;
	for (int j = 0; j < len; j++) {
		stats->avg += buffer[j];
		if (buffer[j] <= min) { min = buffer[j]; }
		if (buffer[j] >= max) { max = buffer[j]; }
	}
	stats->min = min;
	stats->max = max;
	stats->avg /= len;
	stats->range = NRFX_ABS(stats->max - stats->min);
}

int command = 0;
int consecutive_count_12hz = 0;
int consecutive_count_25hz = 0;
const int HYSTERESIS_THRESHOLD = 3; 

const float F_AVG_12HZ_LOWER = 11.5; // Upper bound: 1 / (12 Hz + 1 Hz)
const float F_AVG_12HZ_UPPER = 12.5; // Lower bound: 1 / (12 Hz - 1 Hz)
const float F_AVG_25HZ_LOWER = 24.5; // Upper bound: 1 / (25 Hz + 1 Hz)
const float F_AVG_25HZ_UPPER = 25.5;

int64_t prev_time = 0;

void update_command(int f_avg) {
	printk(">f_avg: %d\n", f_avg);
    // printk(">time: %d\n", DIV_ROUND_CLOSEST(100000, k_uptime_get() - prev_time));
    // printk(">time: %lld ms\n", k_uptime_get());
    // prev_time = k_uptime_get();

    // Check for 12 Hz range using f_avg
    if (f_avg >= F_AVG_12HZ_LOWER && f_avg <= F_AVG_12HZ_UPPER) {
        consecutive_count_25hz = 0; // Reset the other frequency's counter
        if (++consecutive_count_12hz >= HYSTERESIS_THRESHOLD) {
            command = -1;
            consecutive_count_12hz = HYSTERESIS_THRESHOLD; // Cap the count to avoid overflow
        }
    }
    // Check for 25 Hz range using f_avg
    else if (f_avg >= F_AVG_25HZ_LOWER && f_avg <= F_AVG_25HZ_UPPER) {
        consecutive_count_12hz = 0; // Reset the other frequency's counter
        if (++consecutive_count_25hz >= HYSTERESIS_THRESHOLD) {
            command = 1;
            consecutive_count_25hz = HYSTERESIS_THRESHOLD; // Cap the count to avoid overflow
        }
    }
    // Outside the target ranges
    else {
        consecutive_count_12hz = 0;
        consecutive_count_25hz = 0;
        command = 0;
    }
    // printk(">comm: %d\n", command);
}

void command_work_handler(struct k_work *work);
void command_timer_handler(struct k_timer *dummy);

#define COMMAND_TIMER_INTERVAL_MS 150

K_WORK_DEFINE(command_work, command_work_handler);
K_TIMER_DEFINE(command_timer, command_timer_handler, NULL);

void command_work_handler(struct k_work *work)
{
    // Read the current command value
    int current_command = command;

    // Perform actions based on the command value
    switch (current_command) {
        case -1:
            // Action for command = -1
            printk("Command is -1: Performing action for 12 Hz\n");
            break;
        case 0:
            // Action for command = 0
            printk("Command is 0: Performing default action\n");
            break;
        case 1:
            // Action for command = 1
            printk("Command is 1: Performing action for 25 Hz\n");
            break;
        default:
            // Handle unexpected command values
            printk("Unexpected command value: %d\n", current_command);
            break;
    }
}

void command_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&command_work);
}

int print_count = 0;

void comm_sampling_work_handler(struct k_work *work)
{
    // Perform ADC sampling + conversion on all available channels
    for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
        (void) adc_sequence_init_dt(&adc_channels[i], &adc_seq);
        adc_err = adc_read_dt(&adc_channels[i], &adc_seq);
        if (adc_err < 0) {
            printk("Could not read ADC channel (%d)\n", adc_err);
            continue;
        }
        if (adc_channels[i].channel_cfg.differential) {
            adc_val = (int32_t) ((int16_t) adc_val_buf);
        } else {
            adc_val = (int32_t) adc_val_buf;
        }
        adc_err = adc_raw_to_millivolts_dt(&adc_channels[i], &adc_val);
        if (adc_err < 0) {
            printk("ADC conversion to mV failed (%d)\n", adc_err);
        } else if (adc_channels[i].channel_id == ADC_COMMS_CHAN) {
            // Store the comm's ADC value in ring buffer
            adc_buf[n % N] = adc_val;
            n += 1;
        }
    }

    // Compute ADC buffer stats
    update_buffer_stats(adc_buf, N, &adc_stats);
    
    // Normalize adc_value to [0, 100]
    norm = 100.0 * (adc_val - adc_stats.min) / (adc_stats.max - adc_stats.min);
    
    // Start timer on 0->1 transition
    if ((norm > DEMOD_THRESH) & (state == 0)) {
        f_buf[m % M] = DIV_ROUND_CLOSEST(1000, k_uptime_get() - t_rise_prev);
        m += 1;
        t_rise_prev = k_uptime_get();
        state = 1;
    // Update state on 1->0 transition
    } else if ((norm < DEMOD_THRESH) & (state == 1)) {
        state = 0;
    }
    ++print_count;
    if (print_count % 2 == 0) {
        printk(">value:%d\n", norm);
    }
    // printk(">state:%d\n", state);

    // Compute stats on received freqs
    update_buffer_stats(f_buf, M, &f_stats);

    // Output
    if (NRFX_ABS(adc_stats.range) >= MIN_MV_RANGE) {
        // printk(">mv:%d\n", adc_val);
        // printk(">norm:%d\n", norm);
        // printk(">favg:%d\n", f_stats.avg);
        // printk(">mvrange:%d\n", adc_stats.range);
    }

	update_command(f_stats.avg);
}

void robot_step(struct k_work *work);

//////////////////////// BLE //////////////////////////
#define MSEC_TO_BLE_UNITS(TIME_MS) ((TIME_MS) * 1000 / 625)

#define MIN_BLUETOOTH_INTERVAL MSEC_TO_BLE_UNITS(500)
#define MAX_BLUETOOTH_INTERVAL MSEC_TO_BLE_UNITS(501)

static struct bt_le_adv_param *adv_param =
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_NONE,
                    MIN_BLUETOOTH_INTERVAL,
                    MAX_BLUETOOTH_INTERVAL,
                    NULL);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

// #define CUSTOM_ADV_TYPE 0xFF 
// #define COMPANY_ID_CODE 0x0059

// typedef struct the_adc_data {
//         uint16_t company_id;
//         int32_t adc_value;
// } the_adc_data_type;

// static the_adc_data_type the_adc_data = {COMPANY_ID_CODE, 0x00};

// static const struct bt_data ad[] = {
//         BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
//         BT_DATA(CUSTOM_ADV_TYPE, (const uint8_t *)&the_adc_data, sizeof(the_adc_data)),
// };

// static struct bt_data sd[1];

// void update_advertising_data(int32_t new_adc_value) {
//         the_adc_data.adc_value = new_adc_value;

//         int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
//         if (err) {
//                 printk("Advertising failed to start (Error %d)\n", err);
//         }
// }

// static const struct bt_data sd[] = {
// 	BT_DATA(BT_DATA_URI, url_data, sizeof(url_data)),
// };

//////////////////////// BLE //////////////////////////

//TIMING (all ms)
#define ADC_TIME 10 // Time delay to allow ADC reads of all channels
#define MOTOR_SHUTOFF_TIME 50 // Time delay to shutoff motors after being triggered
#define ROBOT_STEP_TIME 150 // Time delay between robot steps

/*Timing Constraints
ADC_TIME must be greater than the time it takes to read all ADC channels. readADC() is called ADC_TIME ms before robot_step() is called
MOTOR_SHUTOFF_TIME is the time the motors are left turned on after being triggered. This value must be smaller than ROBOT_STEP_TIME - ADC_TIME, 
        otherwise the ADC will be reading the motor capacitors while they are still being discharged
ROBOT_STEP_TIME is the time between robot steps. 

*/


//////////////////////// ADC //////////////////////////


// Constants for min val read on ADC for capacitor to discharge/recharge
#define SUPER_CAP_THRESHOLD_MV 2000 //190// Need to tune this value
#define MOTOR_CAP_RELEASE_THRESHOLD_MV 3000//310 //need to tune
#define SUPER_CAP_THRESHOLD (SUPER_CAP_THRESHOLD_MV / 2)
#define MOTOR_CAP_RELEASE_THRESHOLD (MOTOR_CAP_RELEASE_THRESHOLD_MV / 6)

// Define pins/state for capacitors
uint16_t cap_switch_state = 0; // 0: to supercap, 1: to motor cap
const struct device *gpio0;
const struct device *gpio1;
#define CAP_SWITCH_PIN 23  // gpio 0
#define MOTOR_1_CAP_PIN 13 // gpio 1
#define MOTOR_2_CAP_PIN 21 // gpio 0

// error variable
int err;

// ADC defs
#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),


uint32_t count = 0;
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
                .resolution = 10,
	};

// ADC MV Buffer
int32_t adc_outputs_mv[ARRAY_SIZE(adc_channels)];
#define SUPER_CAP_IDX 0
#define MOTOR_CAP_1_IDX 1
#define MOTOR_CAP_2_IDX 2

K_WORK_DEFINE(robot_step_work, robot_step);

void robot_step_timer_handler(struct k_timer *dummy)
{
        k_work_submit(&robot_step_work);
}
K_TIMER_DEFINE(robot_step_timer, robot_step_timer_handler, NULL);


int initCapGPIO()
{
        gpio0 = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(gpio0)));
        gpio1 = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(gpio1)));
        int cap_switch_pin_err;
        int motor_1_cap_pin_err;
        int motor_2_cap_pin_err;

        cap_switch_pin_err = gpio_pin_configure(gpio0, CAP_SWITCH_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
        motor_1_cap_pin_err = gpio_pin_configure(gpio1, MOTOR_1_CAP_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
        motor_2_cap_pin_err = gpio_pin_configure(gpio0, MOTOR_2_CAP_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
        if (cap_switch_pin_err < 0 || motor_1_cap_pin_err < 0 || motor_2_cap_pin_err < 0) return -1;

       // printk("GPIO initialized\n");

        return 0;
}

int initADC()
{
       /* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!device_is_ready(adc_channels[i].dev)) {
			//printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return 1;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);

		if (err < 0) {
			//printk("Could not setup channel #%d (%d)\n", i, err);
			return 1;
		}
	}
        return 0;
}

void readADC(struct k_work *work) //reads all ADC channels and stores mV outputs in adc_outputs_mv array
{
       //printk("ADC reading[%u]:\n", count++);
        for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {

                //printk("- %s, channel %d: ",
                //        adc_channels[i].dev->name,
                //        adc_channels[i].channel_id);
                

                (void)adc_sequence_init_dt(&adc_channels[i], &sequence);

                err = adc_read(adc_channels[i].dev, &sequence);
                if (err < 0) {
                        //printk("Could not read (%d)\n", err);
                        continue;
                }

                /*
                        * If using differential mode, the 16 bit value
                        * in the ADC sample buffer should be a signed 2's
                        * complement value.
                        */
                if (adc_channels[i].channel_cfg.differential) {
                        adc_outputs_mv[i] = (int32_t)((int16_t)buf);
                } else {
                        adc_outputs_mv[i] = (int32_t)buf;
                }
               // printk("Raw Value: %d, ", adc_outputs_mv[i]);
                //adc_outputs_mv[i] = adc_raw_to_millivolts_dt(&adc_channels[i], &val);
                adc_raw_to_millivolts(adc_ref_internal(adc_channels[i].dev), adc_channels[i].channel_cfg.gain, adc_channels[i].resolution, &adc_outputs_mv[i]);
                //adc_outputs_mv[i] = (int32_t)buf;
               // printk("MV Value: %d\n", adc_outputs_mv[i]);
        }
        // update_advertising_data(adc_outputs_mv[MOTOR_CAP_1_IDX]);
}
K_WORK_DEFINE(adc_work, readADC);

void adc_timer_handler(struct k_timer *dummy)
{
        k_work_submit(&adc_work);
}
K_TIMER_DEFINE(adc_timer, adc_timer_handler, NULL);

void motorShutoff(struct k_work *work)
{
        gpio_pin_set(gpio1, MOTOR_1_CAP_PIN, 0);
        gpio_pin_set(gpio0, MOTOR_2_CAP_PIN, 0);
}

K_WORK_DEFINE(motor_shutoff_work, motorShutoff);

void motor_shutoff_timer_handler(struct k_timer *dummy)
{
        k_work_submit(&motor_shutoff_work);
}
K_TIMER_DEFINE(motor_shutoff_timer, motor_shutoff_timer_handler, NULL);


void robot_step(struct k_work *work)
{       
        printk(">command:%d\n", command);
        if(adc_outputs_mv[SUPER_CAP_IDX] <= SUPER_CAP_THRESHOLD){
                gpio_pin_set(gpio0, CAP_SWITCH_PIN, 0);
                //printk("Switching to supercap\n");
        }
        else
        {
                gpio_pin_set(gpio0, CAP_SWITCH_PIN, 1);
                //printk("Switching to motor caps\n");
                if(adc_outputs_mv[MOTOR_CAP_1_IDX] >= MOTOR_CAP_RELEASE_THRESHOLD && adc_outputs_mv[MOTOR_CAP_2_IDX] >= MOTOR_CAP_RELEASE_THRESHOLD){
                        printk(">command:%d\n", command);
                        if(command == -1) {
                            gpio_pin_set(gpio1, MOTOR_1_CAP_PIN, 1);
                        } else if (command == 1) {
                            gpio_pin_set(gpio0, MOTOR_2_CAP_PIN, 1);
                        } else {
                            gpio_pin_set(gpio1, MOTOR_1_CAP_PIN, 1);
                            gpio_pin_set(gpio0, MOTOR_2_CAP_PIN, 1);
                        }
                        k_timer_start(&motor_shutoff_timer, K_MSEC(MOTOR_SHUTOFF_TIME), K_NO_WAIT);
                }
        }
        k_timer_start(&adc_timer, K_MSEC(ROBOT_STEP_TIME - ADC_TIME), K_NO_WAIT);
        k_timer_start(&robot_step_timer, K_MSEC(ROBOT_STEP_TIME), K_NO_WAIT);
        //printk("Robot Step\n");
        return;
}
void main(void)
{
	if (initCapGPIO() != 0) return;

	if (initADC() != 0) return;

        // Initialize buffer stats
	adc_stats.max = 0;
	adc_stats.min = 0;
	adc_stats.avg = 0;
	adc_stats.range = 0;
	f_stats.max = 0;
	f_stats.min = 0;
	f_stats.avg = 0;
	f_stats.range = 0;

    k_timer_start(&comm_timer, K_MSEC(COMM_INTERVAL_MS), K_MSEC(COMM_INTERVAL_MS)); // K_MSEC(COMM_INTERVAL_MS)
	// k_timer_start(&command_timer, K_MSEC(COMMAND_TIMER_INTERVAL_MS), K_MSEC(COMMAND_TIMER_INTERVAL_MS));
	k_timer_start(&adc_timer, K_MSEC(ADC_TIME), K_NO_WAIT);
    k_timer_start(&robot_step_timer, K_MSEC(ADC_TIME*2), K_NO_WAIT);

    while (true) {
        // printk("test\n");
        k_sleep(K_MSEC(1000)); // Sleep for a second (or any other period) to reduce CPU usage
    }

	err = bt_enable(NULL);
	if (err) return;

        err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
        // err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) return;

	for (;;) {
		k_cpu_idle();
	}
}
