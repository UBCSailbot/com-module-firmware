.PHONY: test clean compile_commands help

help:
	@printf "%s\n" \
		"Targets:" \
		"  test              Build and run host-side tests" \
		"  clean             Clean test build artifacts" \
		"  compile_commands  Regenerate top-level compile_commands.json" \
		"  help              Show this help"

test:
	$(MAKE) -C tests test

clean:
	$(MAKE) -C tests clean

compile_commands:
	$(MAKE) -C controller-projects/wingsail-controller compile_commands
