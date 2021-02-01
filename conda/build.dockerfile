
FROM continuumio/miniconda

RUN conda install -y conda-build
RUN apt update
RUN apt install -y build-essential

COPY . /pcl.py
WORKDIR /pcl.py/conda
RUN conda build -c conda-forge .
