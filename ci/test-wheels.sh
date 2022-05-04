#!/bin/bash
# Based on github.com/pypa/python-manylinux-demo, all rights reserved to the original authors
set -e -u -x

# Install packages and test installation
for PYBIN in /opt/python/*/bin/; do
    "${PYBIN}/pip" install -r /io/requirements.txt
    "${PYBIN}/pip" install -r /io/dev-requirements.txt
    "${PYBIN}/pip" install --no-index -f /io/wheelhouse vdbfusion
    (cd "$HOME"; "${PYBIN}/pytest" --capture=sys /io/)
done
