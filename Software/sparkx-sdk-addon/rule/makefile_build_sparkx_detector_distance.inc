BUILD_ALL += out/sparkx_detector_distance

out/sparkx_detector_distance : \
					out/sparkx_detector_distance.o \
					libacconeer.a \
					libacconeer_a111_r2c.a \
					libacc_local_server.a \
					libacc_message_driver_dummy.a \
					out/libcustomer.a \
					libacc_envelope.a \
					libacc_power_bins.a \
					libacc_service.a \
					libacc_detector_distance_peak.a \
					out/acc_board_rpi_sparkx.o
	@echo "    Linking $(notdir $@)"
	@mkdir -p out
	@$(LINK.o) -Wl,--start-group $^ -Wl,--end-group $(LOADLIBES) $(LDLIBS) -o $@
