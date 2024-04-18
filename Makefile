# MIT License
#
# # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
.PHONY: install docker
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
