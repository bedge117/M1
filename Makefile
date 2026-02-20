.PHONY: all setup build clean check_env

all: check_env build

# Check if the ARM compiler is in the PATH
check_env:
	@if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then \
		echo "Error: arm-none-eabi-gcc not found."; \
		echo "Run 'make setup' first to install dependencies."; \
		exit 1; \
	fi

# Distro-specific setup
setup:
	@if [ -f /etc/arch-release ]; then \
		./setup_arch.sh; \
	else \
		echo "Manual setup required: Ensure cmake, ninja, and arm-none-eabi-gcc are installed."; \
	fi

build:
	./build.sh

clean:
	rm -rf build artifacts