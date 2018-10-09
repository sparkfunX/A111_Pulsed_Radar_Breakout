BUILD_ALL += utils/acc_streaming_server_rpi_sparkx

utils/acc_streaming_server_rpi_sparkx : \
					acc_streaming_server.o \
					cJSON.o \
					libacconeer.a \
					libacconeer_a111_r2c.a \
					out/libcustomer.a \
					libacc_service.a \
					out/acc_board_rpi_sparkx.o
	@echo "    Linking $(notdir $@)"
	@mkdir -p utils
	@$(LINK.o) -Wl,--start-group $^ -Wl,--end-group $(LOADLIBES) $(LDLIBS) -o $@
