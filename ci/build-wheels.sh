#!/bin/bash
# Based on github.com/pypa/python-manylinux-demo, all rights reserved to the original authors
set -e -u -x

function repair_wheel {
    wheel="$1"
    if ! auditwheel show "$wheel"; then
        echo "Skipping non-platform wheel $wheel"
    else
        auditwheel repair "$wheel" --plat "$PLAT" -w /io/wheelhouse/
    fi
}

# Install vdbfusion minimal set of dependencies
yum install -y ccache cmake

# Setup CCache
mkdir -p ccache
export CCACHE_BASEDIR=/io/
export CCACHE_DIR=/io/ccache/

# Compile wheels
for PYBIN in /opt/python/*/bin; do
    CMAKE_ARGS=-DSILENCE_WARNINGS=ON "${PYBIN}/pip" --verbose wheel /io/ --no-deps -w wheelhouse/
done

# Bundle external shared libraries into the wheels
for whl in wheelhouse/*.whl; do
    repair_wheel "$whl"
done
