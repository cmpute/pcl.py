FROM mcr.microsoft.com/windows:20H2

COPY . C:/pcl.py
WORKDIR C:/pcl.py/conda

RUN powershell -PassThru install_miniconda.ps1

RUN C:/miniconda/bin/conda.exe install -y conda-build
RUN C:/miniconda/bin/conda.exe build -c conda-forge .
