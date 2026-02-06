#!/bin/bash

export CONAN_ARGS="--pathToExternalModules=./af-basilisk-ext/af_basilisk_ext"
uvx --python 3.11 --with wheel,conan==2.15.1 pip wheel --no-deps -w dist/ .
