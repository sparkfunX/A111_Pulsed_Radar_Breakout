// Copyright (c) Acconeer AB, 2017-2018
// All rights reserved

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "acc_board.h"
#include "acc_device_gpio.h"
#include "acc_device_os.h"
#include "acc_device_spi.h"
#include "acc_driver_gpio_linux_sysfs.h"
#include "acc_driver_os_linux.h"
#include "acc_driver_spi_linux_spidev.h"
#include "acc_log.h"


/**
 * @brief The module name
 *
 * Must exist if acc_log.h is used.
 */
#define MODULE		"acc_board_rpi_sparkx-3_r1c"

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
#define ACC_BOARD_SPI_SPEED	15000000

/**
 * @brief The SPI bus all sensors are using
 */
#define ACC_BOARD_BUS		0

/**
 * @brief The SPI CS all sensors are using
 */
#define ACC_BOARD_CS		0

/**
 * @brief Number of GPIO pins
 */
#define GPIO_PIN_COUNT 28


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


static acc_board_isr_t master_isr;
static acc_device_handle_t spi_handle;
static gpio_t              gpios[GPIO_PIN_COUNT];

static void isr_sensor1(void)
{
	if (master_isr)
	{
		master_isr(1);
	}
}


/**
 * @brief Get the combined status of all sensors
 *
 * @return False if any sensor is busy
 */
static bool acc_board_all_sensors_inactive(void)
{
	for (uint_fast8_t sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++)
	{
		if (sensor_state[sensor_index] == SENSOR_STATE_BUSY)
		{
			return false;
		}
	}

	return true;
}


acc_status_t acc_board_gpio_init(void)
{
	static bool		init_done = false;
	static acc_os_mutex_t	init_mutex = NULL;

	if (init_done)
	{
		return ACC_STATUS_SUCCESS;
	}

	acc_os_init();
	init_mutex = acc_os_mutex_create();

	acc_os_mutex_lock(init_mutex);

	if (init_done)
	{
		acc_os_mutex_unlock(init_mutex);
		return ACC_STATUS_SUCCESS;
	}

	if (
		!acc_device_gpio_set_initial_pull(GPIO0_PIN, 0) ||
		!acc_device_gpio_set_initial_pull(RSTn_PIN, 1) ||
		!acc_device_gpio_set_initial_pull(ENABLE_PIN, 0))
	{
		ACC_LOG_WARNING("%s: failed to set initial pull", __func__);
	}

	if (
		!acc_device_gpio_input(GPIO0_PIN) ||
		!acc_device_gpio_write(RSTn_PIN, 0) ||
		!acc_device_gpio_write(ENABLE_PIN, 0))
	{
		acc_os_mutex_unlock(init_mutex);
		return ACC_STATUS_FAILURE;
	}

	init_done = true;
	acc_os_mutex_unlock(init_mutex);

	return ACC_STATUS_SUCCESS;
}


acc_status_t acc_board_init(void)
{
	static bool		init_done = false;
	static acc_os_mutex_t	init_mutex = NULL;

	if (init_done)
	{
		return ACC_STATUS_SUCCESS;
	}

	acc_driver_os_linux_register();
	acc_os_init();
	init_mutex = acc_os_mutex_create();

	acc_os_mutex_lock(init_mutex);

	if (init_done)
	{
		acc_os_mutex_unlock(init_mutex);
		return ACC_STATUS_SUCCESS;
	}

	acc_driver_gpio_linux_sysfs_register(GPIO_PIN_COUNT, gpios);
	acc_driver_spi_linux_spidev_register();

	acc_device_gpio_init();

	acc_device_spi_configuration_t configuration;

	configuration.bus           = ACC_BOARD_BUS;
	configuration.configuration = NULL;
	configuration.device        = ACC_BOARD_CS;
	configuration.master        = true;
	configuration.speed         = ACC_BOARD_SPI_SPEED;

	spi_handle = acc_device_spi_create(&configuration);

	for (uint_fast8_t sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++)
	{
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
	if (!acc_device_gpio_write(RSTn_PIN, 0))
	{
		ACC_LOG_ERROR("Unable to activate RSTn");
		return ACC_STATUS_FAILURE;
	}

	if (!acc_device_gpio_write(ENABLE_PIN, 0))
	{
		ACC_LOG_ERROR("Unable to deactivate ENABLE");
		return ACC_STATUS_FAILURE;
	}

	return ACC_STATUS_SUCCESS;
}


acc_status_t acc_board_start_sensor(acc_sensor_t sensor)
{
	if (sensor_state[sensor - 1] == SENSOR_STATE_BUSY)
	{
		ACC_LOG_ERROR("Sensor %u already active.", sensor);
		return ACC_STATUS_FAILURE;
	}

	if (acc_board_all_sensors_inactive())
	{
		if (!acc_device_gpio_write(RSTn_PIN, 0))
		{
			ACC_LOG_ERROR("Unable to activate RSTn");
			acc_board_reset_sensor();
			return ACC_STATUS_FAILURE;
		}

		// Wait for PMU to stabilize
		acc_os_sleep_us(5000);

		if (!acc_device_gpio_write(ENABLE_PIN, 1))
		{
			ACC_LOG_ERROR("Unable to activate ENABLE");
			acc_board_reset_sensor();
			return ACC_STATUS_FAILURE;
		}

		// Wait for Power On Reset
		acc_os_sleep_us(5000);

		if (!acc_device_gpio_write(RSTn_PIN, 1))
		{
			ACC_LOG_ERROR("Unable to deactivate RSTn");
			acc_board_reset_sensor();
			return ACC_STATUS_FAILURE;
		}

		for (uint_fast8_t sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++)
		{
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
	if (sensor_state[sensor - 1] != SENSOR_STATE_BUSY)
	{
		ACC_LOG_ERROR("Sensor %u already inactive.", sensor);
		return ACC_STATUS_FAILURE;
	}

	sensor_state[sensor - 1] = SENSOR_STATE_UNKNOWN;

	if (acc_board_all_sensors_inactive())
	{
		return acc_board_reset_sensor();
	}

	return ACC_STATUS_SUCCESS;
}


acc_status_t acc_board_chip_select(acc_sensor_t sensor, uint_fast8_t cs_assert)
{
	ACC_UNUSED(sensor);
	ACC_UNUSED(cs_assert);

	return ACC_STATUS_SUCCESS;
}


acc_sensor_t acc_board_get_sensor_count(void)
{
	return SENSOR_COUNT;
}


bool acc_board_is_sensor_interrupt_connected(acc_sensor_t sensor)
{
	ACC_UNUSED(sensor);

	return true;
}


bool acc_board_is_sensor_interrupt_active(acc_sensor_t sensor)
{
	uint_fast8_t value;

	if (!acc_device_gpio_read(sensor_interrupt_pins[sensor - 1], &value))
	{
		ACC_LOG_ERROR("Could not obtain GPIO interrupt value for sensor %" PRIsensor "", sensor);
		return false;
	}

	return value != 0;
}


acc_status_t acc_board_register_isr(acc_board_isr_t isr)
{
	if (isr != NULL)
	{
		if (
			!acc_device_gpio_register_isr(GPIO0_PIN, ACC_DEVICE_GPIO_EDGE_RISING, &isr_sensor1))
		{
			return ACC_STATUS_FAILURE;
		}
	}
	else
	{
		if (
			!acc_device_gpio_register_isr(GPIO0_PIN, ACC_DEVICE_GPIO_EDGE_NONE, NULL))
		{
			return ACC_STATUS_FAILURE;
		}
	}

	master_isr = isr;

	return ACC_STATUS_SUCCESS;
}


float acc_board_get_ref_freq(void)
{
	return ACC_BOARD_REF_FREQ;
}


acc_status_t acc_board_set_ref_freq(float ref_freq)
{
	ACC_UNUSED(ref_freq);

	return ACC_STATUS_UNSUPPORTED;
}


acc_status_t acc_board_sensor_transfer(acc_sensor_t sensor_id, uint8_t *buffer, size_t buffer_length)
{
	acc_status_t status;
	uint_fast8_t bus = acc_device_spi_get_bus(spi_handle);

	acc_device_spi_lock(bus);

	if ((status = acc_board_chip_select(sensor_id, 1)))
	{
		ACC_LOG_ERROR("%s failed with %s", __func__, acc_log_status_name(status));
		acc_device_spi_unlock(bus);
		return status;
	}

	if ((status = acc_device_spi_transfer(spi_handle, buffer, buffer_length)))
	{
		acc_device_spi_unlock(bus);
		return status;
	}

	if ((status = acc_board_chip_select(sensor_id, 0)))
	{
		acc_device_spi_unlock(bus);
		return status;
	}

	acc_device_spi_unlock(bus);

	return status;
}
