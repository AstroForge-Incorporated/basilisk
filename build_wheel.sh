#!/bin/bash

uv cache clean
uvx --python 3.11 --with wheel pip wheel --no-deps -w dist/ ./af-basilisk-ext
export CONAN_ARGS="--pathToExternalModules=./af-basilisk-ext/af_basilisk_ext --clean"
uvx --python 3.11 --with wheel,conan==2.15.1 pip wheel --no-deps -w dist/ .
