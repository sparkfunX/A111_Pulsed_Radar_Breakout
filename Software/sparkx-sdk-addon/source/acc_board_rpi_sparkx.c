// Copyright (c) Acconeer AB, 2017-2018
// All rights reserved

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "acc_board.h"
#include "acc_device_gpio.h"
#include "acc_driver_gpio_linux_sysfs.h"
#include "acc_driver_spi_linux_spidev.h"
#include "acc_log.h"
#include "acc_os.h"
#include "acc_os_linux.h"


/**
 * @brief The module name
 *
 * Must exist if acc_log.h is used.
 */
#define MODULE		"acc_board_rpi_sparkx"

/**
 * @brief The number of sensors available on the board
 */
#define SENSOR_COUNT	1

/**
 * @brief Host GPIO pin number (BCM)
 *
 * This GPIO should be connected to sensor 1 GPIO 5
 */
#define GPIO0_PIN	25      // Interrupt pin

/**
 * @brief Host GPIO pin number (BCM)
 */
/**@{*/
#define RSTn_PIN	6        // Not connected
#define ENABLE_PIN	27

//#define CE_PIN      8        // Breakout CS pin
/**@}*/

/**
 * @brief The reference frequency used by this board
 *
 * This assumes 24 MHz on XR111-3 R1C
 */
#define ACC_BOARD_REF_FREQ	26000000

/**
 * @brief The SPI speed of this board
 */
//#define ACC_BOARD_SPI_SPEED	15000000
#define ACC_BOARD_SPI_SPEED	1000000

/**
 * @brief The SPI bus all sensors are using
 */
#define ACC_BOARD_BUS		0

/**
 * @brief The SPI CS all sensors are using
 */
#define ACC_BOARD_CS		0


/**
 * @brief Sensor states
 */
typedef enum {
	SENSOR_STATE_UNKNOWN,
	SENSOR_STATE_READY,
	SENSOR_STATE_BUSY
} acc_board_sensor_state_t;


/**
 * @brief Sensor state collection that keeps track of each sensor's current state
 */
static acc_board_sensor_state_t sensor_state[SENSOR_COUNT] = {SENSOR_STATE_UNKNOWN};


static const uint_fast8_t sensor_interrupt_pins[SENSOR_COUNT] = {
	GPIO0_PIN
};


/**
 * @brief Get the combined status of all sensors
 *
 * @return False if any sensor is busy
 */
static bool acc_board_all_sensors_inactive(void)
{
	for (uint_fast8_t sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {
		if (sensor_state[sensor_index] == SENSOR_STATE_BUSY) {
			return false;
		}
	}

	return true;
}


acc_status_t acc_board_gpio_init(void)
{
	acc_status_t		status;
	static bool		init_done = false;
	static acc_os_mutex_t	init_mutex = NULL;

	if (init_done) {
		return ACC_STATUS_SUCCESS;
	}

	acc_os_init();
	init_mutex = acc_os_mutex_create();

	acc_os_mutex_lock(init_mutex);
	if (init_done) {
		acc_os_mutex_unlock(init_mutex);
		return ACC_STATUS_SUCCESS;
	}

	if (
		(status = acc_device_gpio_set_initial_pull(GPIO0_PIN, 0)) ||
		(status = acc_device_gpio_set_initial_pull(RSTn_PIN, 1)) ||
		(status = acc_device_gpio_set_initial_pull(ENABLE_PIN, 0))// ||
		//(status = acc_device_gpio_set_initial_pull(CE_PIN, 1))
	) {
		ACC_LOG_WARNING("%s: failed to set initial pull with status: %s", __func__, acc_log_status_name(status));
	}

	if (
		(status = acc_device_gpio_input(GPIO0_PIN)) ||
		(status = acc_device_gpio_write(RSTn_PIN, 0)) ||
		(status = acc_device_gpio_write(ENABLE_PIN, 0))// ||
		//(status = acc_device_gpio_write(CE_PIN, 1))
	) {
		ACC_LOG_ERROR("%s failed with %s", __func__, acc_log_status_name(status));
		acc_os_mutex_unlock(init_mutex);
		return status;
	}

	init_done = true;
	acc_os_mutex_unlock(init_mutex);

	return ACC_STATUS_SUCCESS;
}


acc_status_t acc_board_init(void)
{
	static bool		init_done = false;
	static acc_os_mutex_t	init_mutex = NULL;

	if (init_done) {
		return ACC_STATUS_SUCCESS;
	}

	acc_driver_os_linux_register();
	acc_os_init();
	init_mutex = acc_os_mutex_create();

	acc_os_mutex_lock(init_mutex);
	if (init_done) {
		acc_os_mutex_unlock(init_mutex);
		return ACC_STATUS_SUCCESS;
	}

	acc_driver_gpio_linux_sysfs_register(28);
	acc_driver_spi_linux_spidev_register();

	for (uint_fast8_t sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {
		sensor_state[sensor_index] = SENSOR_STATE_UNKNOWN;
	}

	init_done = true;
	acc_os_mutex_unlock(init_mutex);

	return ACC_STATUS_SUCCESS;
}


/**
 * @brief Reset sensor
 *
 * Default setup when sensor is not active
 *
 * @return Status
 */
static acc_status_t acc_board_reset_sensor(void)
{
	acc_status_t status;

	status = acc_device_gpio_write(RSTn_PIN, 0);
	if (status != ACC_STATUS_SUCCESS) {
		ACC_LOG_ERROR("Unable to activate RSTn");
		return status;
	}

	status = acc_device_gpio_write(ENABLE_PIN, 0);
	if (status != ACC_STATUS_SUCCESS) {
		ACC_LOG_ERROR("Unable to deactivate ENABLE");
		return status;
	}

	return ACC_STATUS_SUCCESS;
}


acc_status_t acc_board_start_sensor(acc_sensor_t sensor)
{
	acc_status_t status;

	if (sensor_state[sensor - 1] == SENSOR_STATE_BUSY) {
		ACC_LOG_ERROR("Sensor %u already active.", sensor);
		return ACC_STATUS_FAILURE;
	}

	if (acc_board_all_sensors_inactive()) {
		status = acc_device_gpio_write(RSTn_PIN, 0);
		if (status != ACC_STATUS_SUCCESS) {
			ACC_LOG_ERROR("Unable to activate RSTn");
			acc_board_reset_sensor();
			return status;
		}

		status = acc_device_gpio_write(ENABLE_PIN, 1);
		if (status != ACC_STATUS_SUCCESS) {
			ACC_LOG_ERROR("Unable to activate ENABLE");
			acc_board_reset_sensor();
			return status;
		}

		// Wait for Power On Reset
		acc_os_sleep_us(5000);

		status = acc_device_gpio_write(RSTn_PIN, 1);
		if (status != ACC_STATUS_SUCCESS) {
			ACC_LOG_ERROR("Unable to deactivate RSTn");
			acc_board_reset_sensor();
			return status;
		}

		for (uint_fast8_t sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {
			sensor_state[sensor_index] = SENSOR_STATE_READY;
		}
	}

	if (sensor_state[sensor - 1] != SENSOR_STATE_READY) {
		ACC_LOG_ERROR("Sensor has not been reset");
		return ACC_STATUS_FAILURE;
	}

	sensor_state[sensor - 1] = SENSOR_STATE_BUSY;

	return ACC_STATUS_SUCCESS;
}


acc_status_t acc_board_stop_sensor(acc_sensor_t sensor)
{
	if (sensor_state[sensor - 1] != SENSOR_STATE_BUSY) {
		ACC_LOG_ERROR("Sensor %u already inactive.", sensor);
		return ACC_STATUS_FAILURE;
	}

	sensor_state[sensor - 1] = SENSOR_STATE_UNKNOWN;

	if (acc_board_all_sensors_inactive()) {
		return acc_board_reset_sensor();
	}

	return ACC_STATUS_SUCCESS;
}


void acc_board_get_spi_bus_cs(acc_sensor_t sensor, uint_fast8_t *bus, uint_fast8_t *cs)
{
	if ((sensor <= 0) || (sensor > SENSOR_COUNT)) {
		*bus = -1;
		*cs  = -1;
	} else {
		*bus = ACC_BOARD_BUS;
		*cs  = ACC_BOARD_CS;
	}
}


acc_status_t acc_board_chip_select(acc_sensor_t sensor, uint_fast8_t cs_assert)
{
	ACC_UNUSED(sensor);
	ACC_UNUSED(cs_assert);
	/*acc_status_t status;

	if (cs_assert) {
		uint_fast8_t cea_val = (sensor == 1 || sensor == 2) ? 0 : 1;
		uint_fast8_t ceb_val = (sensor == 1 || sensor == 3) ? 0 : 1;

		if (
			(status = acc_device_gpio_write(CE_A_PIN, cea_val)) ||
			(status = acc_device_gpio_write(CE_B_PIN, ceb_val))
		) {
			ACC_LOG_ERROR("%s failed with %s", __func__, acc_log_status_name(status));
			return status;
		}
		status = acc_device_gpio_write(CE_PIN, 0);
		if (status) {
			ACC_LOG_ERROR("%s failed with %s", __func__, acc_log_status_name(status));
			return status;
		}		
	}*/

	return ACC_STATUS_SUCCESS;
}


acc_sensor_t acc_board_get_sensor_count(void)
{
	return SENSOR_COUNT;
}


bool acc_board_is_sensor_interrupt_connected(acc_sensor_t sensor)
{
	ACC_UNUSED(sensor);

	return false;
}


bool acc_board_is_sensor_interrupt_active(acc_sensor_t sensor)
{
	acc_status_t status;
	uint_fast8_t value;

	status = acc_device_gpio_read(sensor_interrupt_pins[sensor - 1], &value);
	if (status != ACC_STATUS_SUCCESS) {
		ACC_LOG_ERROR("Could not obtain GPIO interrupt value for sensor %" PRIsensor " with status: %s.", sensor, acc_log_status_name(status));
		return false;
	}

	return value != 0;
}


float acc_board_get_ref_freq(void)
{
	return ACC_BOARD_REF_FREQ;
}


uint32_t acc_board_get_spi_speed(uint_fast8_t bus)
{
	ACC_UNUSED(bus);

	return ACC_BOARD_SPI_SPEED;
}


acc_status_t acc_board_set_ref_freq(float ref_freq)
{
	ACC_UNUSED(ref_freq);

	return ACC_STATUS_UNSUPPORTED;
}
