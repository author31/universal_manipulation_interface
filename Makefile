VENV_NAME ?= .venv
PYTHON = $(VENV_NAME)/bin/python

.PHONY: install-uv
install-uv:  ## Install uv if it's not present.
	@command -v uv >/dev/null 2>&1 || curl -LsSf https://astral.sh/uv/install.sh | sh

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
