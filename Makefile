# The program lives in Prueba/; these targets forward to its Makefile.
all test clean:
	$(MAKE) -C Prueba $@

.PHONY: all test clean
