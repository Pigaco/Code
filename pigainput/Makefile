pigainput: pigainput.c 
	$(CC) $(CFLAGS) \
		-g -Werror -I. \
		-DWLR_USE_UNSTABLE \
		$(shell pkg-config --cflags --libs libevdev) \
		-o $@ $<

clean:
	rm -f pigainput

.DEFAULT_GOAL=pigainput
.PHONY: clean
