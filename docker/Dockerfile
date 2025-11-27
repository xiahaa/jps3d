# Multi-stage Dockerfile for building BLJPS_Python and jps3d shared libraries
FROM idea-laser.tencentcloudcr.com/public/python:3.10.6 AS builder

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libeigen3-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip install --no-cache-dir \
    pybind11[global] \
    setuptools \
    wheel \
    numpy

# Set working directory
WORKDIR /workspace

# Debug: Check Python configuration and fix headers
RUN echo "=== Python Configuration Debug ===" && \
    echo "Python executable: $(which python3)" && \
    echo "Python version: $(python3 --version)" && \
    echo "Python include dir: $(python3 -c 'import sysconfig; print(sysconfig.get_path("include"))')" && \
    echo "Python library: $(python3 -c 'import sysconfig; print(sysconfig.get_config_var("LIBDIR"), sysconfig.get_config_var("LDLIBRARY"))')" && \
    echo "Checking for python.h..." && \
    find /usr -name "python.h" 2>/dev/null | head -5 && \
    echo "Creating symlink for python headers if needed..." && \
    PYTHON_INCLUDE=$(python3 -c 'import sysconfig; print(sysconfig.get_path("include"))') && \
    if [ ! -f "$PYTHON_INCLUDE/python.h" ] && [ -f "/usr/include/python3.10/Python.h" ]; then \
        ln -sf /usr/include/python3.10/Python.h $PYTHON_INCLUDE/python.h; \
    fi && \
    echo "=== End Debug ==="

# Clone the source code from GitHub repository
RUN git clone https://github.com/xiahaa/jps3d.git . && \
    git submodule update --init --recursive

# Build BLJPS_Python
WORKDIR /workspace/3rdparty/BLJPS_Python

# Clone pybind11 since it's not included in the repository
RUN git clone https://github.com/pybind/pybind11.git pybind11

# Clean any existing build artifacts and create fresh build directory
RUN rm -rf build && mkdir -p build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_STANDARD=17 \
        -DPYBIND11_PYTHON_VERSION=3.10 && \
    cmake --build . --config Release --parallel $(nproc)

# Build jps3d Python bindings
WORKDIR /workspace/3rdparty/jps3d
# Clean any existing build artifacts
RUN rm -rf build && rm -f *.so
# Set environment variables for jps3d build
ENV BOOST_INCLUDE_DIR=/usr/include
ENV JPS_INCLUDE_DIR=include
RUN python3 setup.py build_ext --inplace

# Verify builds completed successfully
RUN echo "=== Build Verification ===" && \
    echo "BLJPS_Python shared libraries:" && \
    find /workspace/3rdparty/BLJPS_Python -name "*.so" -ls && \
    echo "jps3d shared libraries:" && \
    find /workspace/3rdparty/jps3d -name "*.so" -ls && \
    echo "=== End Verification ==="

# Production stage
FROM idea-laser.tencentcloudcr.com/public/python:3.10.6

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python runtime dependencies
RUN pip install --no-cache-dir \
    pybind11 \
    numpy

# Create directories for shared libraries
RUN mkdir -p /usr/local/lib/python3.10/site-packages

# Copy built shared libraries from builder stage
COPY --from=builder /workspace/3rdparty/BLJPS_Python/build/*.so /usr/local/lib/python3.10/site-packages/
COPY --from=builder /workspace/3rdparty/jps3d/*.so /usr/local/lib/python3.10/site-packages/

# Set Python path to include the shared libraries
ENV PYTHONPATH="/usr/local/lib/python3.10/site-packages:${PYTHONPATH}"

# Set working directory
WORKDIR /app

# Default command
CMD ["python3"]
