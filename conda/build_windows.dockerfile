FROM mcr.microsoft.com/windows:20H2

COPY . C:/pcl.py
WORKDIR C:/pcl.py/conda

RUN powershell -noprofile -executionpolicy bypass -file install_miniconda.ps1
RUN setx path '%path%;C:/Miniconda/condabin'

RUN conda install -y conda-build
RUN conda build -c conda-forge .
