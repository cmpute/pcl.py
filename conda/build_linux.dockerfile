FROM continuumio/miniconda

RUN conda install -y conda-build

COPY . /pcl.py
WORKDIR /pcl.py/conda
RUN conda build -c conda-forge --output-folder . .
