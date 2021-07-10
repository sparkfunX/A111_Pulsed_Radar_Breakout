BUILD_ALL += $(OUT_DIR)/sparkx_detector_distance

$(OUT_DIR)/sparkx_detector_distance : \
					$(OUT_OBJ_DIR)/sparkx_detector_distance.o \
					libacconeer.a \
					libacconeer_a111_r2c.a \
					libcustomer.a \
					libacc_service.a \
					libacc_detector_distance_peak.a \
					$(OUT_OBJ_DIR)/acc_board_rpi_sparkx.o
	@echo "    Linking $(notdir $@)"
	$(SUPPRESS)mkdir -p $(OUT_DIR)
	$(SUPPRESS)$(LINK.o) -Wl,--start-group $^ -Wl,--end-group $(LOADLIBES) $(LDLIBS) -o $@
