BUILD_ALL += out/sparkx_power_bins_example

out/sparkx_power_bins_example : \
					out/example_service_power_bins.o \
					libacconeer.a \
					libacconeer_a111_r2c.a \
					out/libcustomer.a \
					libacc_service.a \
					out/acc_board_rpi_sparkx.o
	@echo "    Linking $(notdir $@)"
	@mkdir -p out
	@$(LINK.o) -Wl,--start-group $^ -Wl,--end-group $(LOADLIBES) $(LDLIBS) -o $@
