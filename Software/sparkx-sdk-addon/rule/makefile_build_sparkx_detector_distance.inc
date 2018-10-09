BUILD_ALL += out/sparkx_detector_distance

out/sparkx_detector_distance : \
					out/sparkx_detector_distance.o \
					libacconeer.a \
					libacconeer_a111_r2c.a \
					out/libcustomer.a \
					libacc_service.a \
					libacc_detector_distance_peak.a \
					out/acc_board_rpi_sparkx.o
	@echo "    Linking $(notdir $@)"
	@mkdir -p out
	@$(LINK.o) -Wl,--start-group $^ -Wl,--end-group $(LOADLIBES) $(LDLIBS) -o $@
