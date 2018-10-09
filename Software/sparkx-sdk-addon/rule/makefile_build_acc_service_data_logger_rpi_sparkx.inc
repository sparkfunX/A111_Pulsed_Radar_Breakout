BUILD_ALL += utils/acc_service_data_logger_rpi_sparkx

utils/acc_service_data_logger_rpi_sparkx : \
					out/acc_service_data_logger.o \
					libacconeer.a \
					libacconeer_a111_r2c.a \
					out/libcustomer.a \
					libacc_service.a \
					out/acc_board_rpi_sparkx.o
	@echo "    Linking $(notdir $@)"
	@mkdir -p utils
	@$(LINK.o) -Wl,--start-group $^ -Wl,--end-group $(LOADLIBES) $(LDLIBS) -o $@
