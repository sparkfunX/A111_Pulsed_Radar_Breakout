// Copyright (c) Acconeer AB, 2018
// All rights reserved

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include "acc_detector_distance_peak.h"
//#include "acc_os.h"
#include "acc_rss.h"
#include "acc_sweep_configuration.h"
#include "acc_types.h"
#include "acc_version.h"


/**
 * @brief Example that shows how to use the distance peak detector
 *
 * This is an example on how the distance peak detector can be used.
 * The example executes as follows:
 *   - Activate Radar System Services (RSS)
 *   - Create a distance peak detector configuration (with blocking mode as default)
 *   - Create a distance peak detector using the previously created configuration
 *   - Activate the distance peak detector
 *   - Get the result and print it 100 times
 *   - Deactivate and destroy the distance peak detector
 *   - Destroy the distance peak detector configuration
 *   - Deactivate Radar System Services
 */


#define FIXED_THRESHOLD_VALUE   (1500)
#define SENSOR_ID               (1)
#define RANGE_START_M           (0.2)
#define RANGE_LENGTH_M          (0.5)

#define NUM_DISTANCE_DETECT           100
#define NUM_DISTANCE_THRESHOLD_DETECT 100


static acc_detector_distance_peak_status_t distance_peak_detect_with_blocking_calls(acc_detector_distance_peak_configuration_t distance_configuration);
static acc_detector_distance_peak_status_t distance_peak_detect_with_blocking_calls_with_estimated_threshold(acc_detector_distance_peak_configuration_t distance_configuration);
static char *format_distances(uint16_t reflection_count,
			      const acc_detector_distance_peak_reflection_t *reflections,
			      float sensor_offset);
static void configure_detector(acc_detector_distance_peak_configuration_t distance_configuration);
void waitForEnter(void);

static uint8_t not_interrupted = 1;

void exit_handler(int sig_num)
{
	ACC_UNUSED(sig_num);
	printf("\nCtrl+C pressed. Beginning shutdown\n");
	not_interrupted = 0;
}

int main(int argc, char *argv[])
{
	ACC_UNUSED(argc);
	ACC_UNUSED(argv);

	acc_detector_distance_peak_status_t detector_status;

	printf("Acconeer software version %s\n", ACC_VERSION);
	printf("Acconeer RSS version %s\n", acc_rss_version());

	//Initialize Radar Service System
	if (!acc_rss_activate()) {
		return EXIT_FAILURE;
	}

	//Create the detector configuration
	acc_detector_distance_peak_configuration_t distance_configuration = acc_detector_distance_peak_configuration_create();

	if (distance_configuration == NULL) {
		printf("\nacc_service_distance_configuration_create() failed");
		return EXIT_FAILURE;
	}
	printf("\nA111 Initialized\n");

	printf("\nPress ENTER to run distance detection...");
	waitForEnter();

	//Run distance peak detection in blocking mode
	detector_status = distance_peak_detect_with_blocking_calls(distance_configuration);
	if (detector_status != ACC_DETECTOR_DISTANCE_PEAK_STATUS_SUCCESS) {
		printf("Running distance peak detector in blocking mode failed");
	}

	printf("\nPress ENTER to calculate THRESHOLD and run peak detection...");
	waitForEnter();

	detector_status = distance_peak_detect_with_blocking_calls_with_estimated_threshold(distance_configuration);
	if (detector_status != ACC_DETECTOR_DISTANCE_PEAK_STATUS_SUCCESS) {
		printf("Running distance peak detector in blocking mode with estimated threshold failed");
	}

	acc_detector_distance_peak_configuration_destroy(&distance_configuration);

	acc_rss_deactivate();

	return EXIT_SUCCESS;
}

acc_detector_distance_peak_status_t distance_peak_detect_with_blocking_calls(acc_detector_distance_peak_configuration_t distance_configuration)
{
	acc_detector_distance_peak_status_t detector_status = ACC_DETECTOR_DISTANCE_PEAK_STATUS_SUCCESS;
	acc_detector_distance_peak_metadata_t metadata;
	uint_fast16_t reflection_count = 10;
	acc_detector_distance_peak_reflection_t reflections[reflection_count];

	printf("Running distance peak detector in blocking mode\n");

	configure_detector(distance_configuration);

	detector_status = acc_detector_distance_peak_set_threshold_mode_fixed(distance_configuration, FIXED_THRESHOLD_VALUE);

	acc_detector_distance_peak_handle_t handle = acc_detector_distance_peak_create(distance_configuration);

	if (handle == NULL) {
		printf("acc_service_create failed\n");
		return ACC_DETECTOR_DISTANCE_PEAK_STATUS_FAILURE;
	}

	acc_detector_distance_peak_get_metadata(handle, &metadata);
	printf("Free space absolute offset: %u mm\n", (unsigned int)(metadata.free_space_absolute_offset * 1000.0 + 0.5));
	printf("Actual start: %u mm\n", (unsigned int)(metadata.actual_start_m * 1000.0 + 0.5));
	printf("Actual length: %u mm\n", (unsigned int)(metadata.actual_length_m * 1000.0 + 0.5));
	printf("Actual end: %u mm\n", (unsigned int)((metadata.actual_start_m + metadata.actual_length_m) * 1000.0 + 0.5));
	printf("\n");

	acc_detector_distance_peak_result_info_t result_info;

	float start_m = metadata.actual_start_m;
	float end_m = metadata.actual_start_m + metadata.actual_length_m;

	detector_status = acc_detector_distance_peak_activate(handle);

	if (detector_status == ACC_DETECTOR_DISTANCE_PEAK_STATUS_SUCCESS) {
		uint_fast8_t detection_runs = NUM_DISTANCE_DETECT;

		while (detection_runs > 0) {

			reflection_count = 20;

			detector_status = acc_detector_distance_peak_get_next(handle,
										reflections,
										&reflection_count,
										&result_info);

			if (detector_status == ACC_DETECTOR_DISTANCE_PEAK_STATUS_SUCCESS) {
				printf("Distance detector: Reflections: %u. Seq. nr: %u. (%u-%u mm): %s\n",
				(unsigned int)reflection_count,
				(unsigned int)result_info.sequence_number,
				(unsigned int)(start_m * 1000.0 + 0.5),
				(unsigned int)(end_m * 1000.0 + 0.5),
				format_distances(reflection_count, reflections, metadata.free_space_absolute_offset));
			}
			else {
				printf("reflection data not properly retrieved\n");
			}

			detection_runs--;
		}

		detector_status = acc_detector_distance_peak_deactivate(handle);
	}
	else {
		printf("acc_service_activate() failed\n");
	}

	acc_detector_distance_peak_destroy(&handle);

	return detector_status;

}

acc_detector_distance_peak_status_t distance_peak_detect_with_blocking_calls_with_estimated_threshold(acc_detector_distance_peak_configuration_t distance_configuration)
{
	acc_detector_distance_peak_status_t detector_status = ACC_DETECTOR_DISTANCE_PEAK_STATUS_SUCCESS;
	acc_detector_distance_peak_metadata_t metadata;
	uint_fast16_t reflection_count = 10;
	acc_detector_distance_peak_reflection_t reflections[reflection_count];

	printf("Running distance peak detector in blocking mode with estimated threshold\n");

	configure_detector(distance_configuration);

	acc_detector_distance_peak_handle_t handle = acc_detector_distance_peak_create(distance_configuration);

	if (handle == NULL) {
		printf("acc_service_create failed\n");
		return ACC_DETECTOR_DISTANCE_PEAK_STATUS_FAILURE;
	}

	acc_detector_distance_peak_get_metadata(handle, &metadata);
	printf("Free space absolute offset: %u mm\n", (unsigned int)(metadata.free_space_absolute_offset * 1000.0 + 0.5));
	printf("Actual start: %u mm\n", (unsigned int)(metadata.actual_start_m * 1000.0 + 0.5));
	printf("Actual length: %u mm\n", (unsigned int)(metadata.actual_length_m * 1000.0 + 0.5));
	printf("Actual end: %u mm\n", (unsigned int)((metadata.actual_start_m + metadata.actual_length_m) * 1000.0 + 0.5));
	printf("\n");

	acc_detector_distance_peak_result_info_t result_info;

	float start_m = metadata.actual_start_m;
	float end_m = metadata.actual_start_m + metadata.actual_length_m;

	detector_status = acc_detector_distance_peak_set_threshold_mode_estimation(distance_configuration);

	printf("Estimating threshold...\n");

	detector_status = acc_detector_distance_peak_threshold_estimation_update(distance_configuration,
										100,
										metadata.actual_start_m,
										metadata.actual_start_m + metadata.actual_length_m);
	printf("Estimating threshold done...\n");
	printf("\nPress ENTER to measure threshold-ed distance\nThen press CTRL+C to STOP\n");
	waitForEnter();

	not_interrupted = 1;
	signal(SIGINT, exit_handler); // Register interupt handler to exit gracefully

	while (not_interrupted != 0)
	{
		detector_status = acc_detector_distance_peak_activate(handle);

		if (detector_status == ACC_DETECTOR_DISTANCE_PEAK_STATUS_SUCCESS) {
			uint_fast8_t detection_runs = NUM_DISTANCE_THRESHOLD_DETECT;

			while (detection_runs > 0) {

				reflection_count = 10;

				detector_status = acc_detector_distance_peak_get_next(handle,
											reflections,
											&reflection_count,
											&result_info);

				if (detector_status == ACC_DETECTOR_DISTANCE_PEAK_STATUS_SUCCESS) {
					printf("Distance detector: Reflections: %u. Seq. nr: %u. (%u-%u mm): %s\n",
					(unsigned int)reflection_count,
					(unsigned int)result_info.sequence_number,
					(unsigned int)(start_m * 1000.0 + 0.5),
					(unsigned int)(end_m * 1000.0 + 0.5),
					format_distances(reflection_count, reflections, metadata.free_space_absolute_offset));
				}
				else {
					printf("reflection data not properly retrieved\n");
				}

				detection_runs--;
			}
		}
		else {
			printf("acc_service_activate() failed\n");
			break;
		}
	}

	detector_status = acc_detector_distance_peak_deactivate(handle);

	acc_detector_distance_peak_destroy(&handle);

	return detector_status;

}

char *format_distances(uint16_t reflection_count,
		       const acc_detector_distance_peak_reflection_t *reflections,
		       float sensor_offset)
{
	static char	buffer[1024];
	size_t		total_count = 0;
	int		count;

	*buffer = 0;
	for (uint_fast8_t reflection_index = 0; reflection_index < reflection_count; reflection_index++) {
		if (total_count > 0) {
			count = snprintf(&buffer[total_count], sizeof(buffer) - total_count, ", ");
			if (count < 0) {
				break;
			}
			total_count += count;
		}

		count = snprintf(&buffer[total_count], sizeof(buffer) - total_count, "%u mm (%u)",
				(unsigned int)((reflections[reflection_index].distance - sensor_offset) * 1000.0 + 0.5),
				(unsigned int)(reflections[reflection_index].amplitude + 0.5));
		if (count < 0) {
			break;
		}
		total_count += count;

		if (total_count >= sizeof(buffer) - 1) {
			break;
		}
	}

	return buffer;
}


static void configure_detector(acc_detector_distance_peak_configuration_t distance_configuration)
{
	acc_sweep_configuration_t sweep_configuration = acc_detector_distance_peak_get_sweep_configuration(distance_configuration);

	if (sweep_configuration == NULL) {
		printf("\nSweep configuration not available");
	}
	else {
		acc_detector_distance_peak_profile_set(distance_configuration, ACC_DETECTOR_DISTANCE_PEAK_PROFILE_MAXIMIZE_DEPTH_RESOLUTION);

		acc_sweep_configuration_sensor_set(sweep_configuration, SENSOR_ID);
		acc_sweep_configuration_requested_start_set(sweep_configuration, RANGE_START_M);
		acc_sweep_configuration_requested_length_set(sweep_configuration, RANGE_LENGTH_M);
	}
}

void waitForEnter(void)
{
	fflush(stdout);
	getchar();
}