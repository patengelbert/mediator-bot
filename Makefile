#!/bin/bash

.PHONY: docs release clean build test run

clean:
	rm -rf env htmlcov build .tox .eggs

build:
	virtualenv -p /usr/bin/python env
	. env/bin/activate && \
	pip install -U -r requirements.txt

test: clean build
	. env/bin/activate && \
	py.test mediator_bot/test && \
	py.test --cov=mediator_bot mediator_bot/test && \
	coverage html && \
	coverage report

docs:
	sphinx-build -aE docs build/docs > /dev/null

release: test docs
	xdg-open mediator_bot/version.py
	xdg-open build/docs/index.html > /dev/null 2>&1 &
	xdg-open htmlcov/index.html > /dev/null 2>&1 &

run:
	. env/bin/activate && \
	python mediator_bot/bot.py -c mediator_bot/config/local.conf --debug

tox:
	. env/bin/activate && \
	tox
