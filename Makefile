.PHONY: install docker format
SOURCES := $(shell find . -regextype posix-extended -regex  ".*\.(cpp|cxx|cc|hpp|hxx|h)" | grep -vE "^./(build|3rdparty)/")

install:
	pip3 -v install . && cp build/*/compile_commands.json build/

uninstall:
	pip3 -v uninstall vdbfusion

dev:
	pip3 -v install -e . && cp build/*/compile_commands.json build/

test:
	pytest .

clean:
	git clean -xf . && rm -rf build/

format:
	clang-format -i $(SOURCES)

# Docker targets
docker:
	@echo Building builder docker container
	docker-compose build builder

docker_run:
	@echo Building Running docker
	docker-compose run --rm builder

docker_push:
	@echo Pushing builder image to docker registry
	docker-compose push builder

docker_pip:
	@echo Building pip docker container
	docker-compose build pip_builder

docker_pip_push:
	@echo Pushing pip image to docker registry
	docker-compose push pip_builder
