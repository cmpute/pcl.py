FROM quay.io/pypa/manylinux2010_x86_64
ARG PYTHON_VERSION

WORKDIR /opt

RUN yum install -y pcl-devel wget
RUN mkdir /opt/cmake && wget -qO- "https://cmake.org/files/v3.15/cmake-3.15.0-Linux-x86_64.tar.gz" | tar --strip-components=1 -xz -C /opt/cmake
ENV PATH="/opt/cmake/bin:${PATH}"
RUN /opt/python/cp${PYTHON_VERSION}-cp${PYTHON_VERSION}m/bin/python -m pip install numpy cython scikit-build

COPY . /opt/pcl.py
WORKDIR /opt/pcl.py
RUN /opt/python/cp${PYTHON_VERSION}-cp${PYTHON_VERSION}m/bin/python setup.py build
