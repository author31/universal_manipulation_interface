VENV_NAME ?= .venv
PYTHON = $(VENV_NAME)/bin/python

.PHONY: install-uv
install-uv:  ## Install uv if it's not present.
	@command -v uv >/dev/null 2>&1 || curl -LsSf https://astral.sh/uv/install.sh | sh

.PHONY: install-exiftool
install-exiftool:
	TARGET_VERSION="13.30"; \
	MIN_VERSION="12.5"; \
	CURRENT_VERSION="$$(command -v exiftool >/dev/null 2>&1 && exiftool -ver || echo '0')"; \
	ver_ge() { \
		[ "$$(printf '%s\n' "$$1" "$$2" | sort -V | head -n1)" = "$$2" ]; \
	}; \
	if ver_ge "$$CURRENT_VERSION" "$$MIN_VERSION"; then \
		echo "ExifTool version $$CURRENT_VERSION found (>= $$MIN_VERSION). Skipping installation."; \
	else \
		TEMP_DIR="$$(mktemp -d)"; \
		INSTALL_PREFIX="/usr/local"; \
		echo "Installing ExifTool version $$TARGET_VERSION..."; \
		cd "$$TEMP_DIR"; \
		TARBALL="Image-ExifTool-$${TARGET_VERSION}.tar.gz"; \
		URL="https://sourceforge.net/projects/exiftool/files/$${TARBALL}/download"; \
		echo "Downloading $$TARBALL..."; \
		wget -q "$$URL" -O "$$TARBALL"; \
		echo "Extracting..."; \
		tar xzf "$$TARBALL"; \
		cd "Image-ExifTool-$${TARGET_VERSION}"; \
		echo "Building..."; \
		perl Makefile.PL; \
		make; \
		echo "Installing..."; \
		sudo make install; \
		echo "Cleaning up..."; \
		cd /; \
		rm -rf "$$TEMP_DIR"; \
		echo "Installation complete. Verifying version..."; \
		INSTALLED="$$(exiftool -ver)"; \
		echo "Installed ExifTool version: $$INSTALLED"; \
		if [ "$$INSTALLED" = "$$TARGET_VERSION" ]; then \
			echo "Success: Version properly installed."; \
		else \
			echo "Warning: Expected version $$TARGET_VERSION, but found $$INSTALLED."; \
		fi; \
	fi

.PHONY: venv
venv: install-uv  ## Create virtual environment if it doesn't exist.
	@[ -d $(VENV_NAME) ] || uv venv $(VENV_NAME) --python 3.9

.PHONY: deps
deps: venv
	uv pip install -r requirements.txt --python $(PYTHON)

.PHONY: run-preprocess
run-preprocess: venv
	@if [ -z "$(dirname)" ]; then echo "Error: dirname is required"; exit 1; fi
	$(PYTHON) run_slam_pipeline.py $(dirname)

.PHONY: run-generate-zarr-dataset
run-generate-zarr-dataset: venv
	@if [ -z "$(dirname)" ]; then echo "Error: dirname is required"; exit 1; fi
	$(PYTHON) scripts_slam_pipeline/07_generate_replay_buffer.py -o $(patsubst %/,%,$(dirname))/dataset.zarr.zip $(patsubst %/,%,$(dirname))
