# Multi-stage Dockerfile for building BLJPS_Python and jps3d shared libraries
FROM idea-laser.tencentcloudcr.com/public/python:3.10.6 as builder

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

# Clone the source code from GitHub repository
RUN git clone https://github.com/xiahaa/jps3d.git . && \
    git submodule update --init --recursive

# Build BLJPS_Python
WORKDIR /workspace/3rdparty/BLJPS_Python
# Clean any existing build artifacts and create fresh build directory
RUN rm -rf build && mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
           -DPYTHON_EXECUTABLE=$(which python3) \
           -DPYTHON_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
           -DPYBIND11_PYTHON_VERSION=3.10 && \
    cmake --build . --config Release

# Build jps3d Python bindings
WORKDIR /workspace/3rdparty/jps3d
# Clean any existing build artifacts
RUN rm -rf build && rm -f *.so
# Set environment variables for jps3d build
ENV BOOST_INCLUDE_DIR=/usr/include
ENV JPS_INCLUDE_DIR=include
RUN python3 setup.py build_ext --inplace

# Verify builds completed successfully
RUN echo "Verifying BLJPS_Python build..." && \
    find /workspace/3rdparty/BLJPS_Python/build -name "*.so" -ls && \
    echo "Verifying jps3d build..." && \
    find /workspace/3rdparty/jps3d -name "*.so" -ls

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
