#!/bin/sh
set -ex
cd "${1:-.}"
rm -rf .openblas
mkdir .openblas

# Select OpenBLAS download based on architecture
if [ "${CIBW_ARCHS}" = "ARM64" ]; then
    curl -sSL -o .openblas/openblas.zip https://github.com/OpenMathLib/OpenBLAS/releases/download/v0.3.34/OpenBLAS-0.3.34-woa64-dll.zip
    python -m zipfile -e .openblas/openblas.zip .openblas
    rm .openblas/openblas.zip
    # ARM64 zip extracts to OpenBLAS/ subdirectory, so we need to move files up
    mkdir .openblas/lib
    mkdir .openblas/bin
    cp .openblas/OpenBLAS/lib/openblas.lib .openblas/lib/openblas.lib
    cp .openblas/OpenBLAS/bin/*.dll .openblas/bin/
else
    # Default to AMD64/x64
    curl -sSL -o .openblas/openblas.zip https://github.com/OpenMathLib/OpenBLAS/releases/download/v0.3.34/OpenBLAS-0.3.34-x64.zip
    python -m zipfile -e .openblas/openblas.zip .openblas
    rm .openblas/openblas.zip
    cp .openblas/lib/libopenblas.lib .openblas/lib/openblas.lib
fi
