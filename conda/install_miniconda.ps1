Invoke-WebRequest -Uri https://repo.anaconda.com/miniconda/Miniconda2-latest-Windows-x86_64.exe -OutFile miniconda.exe
Start-Process -FilePath ./miniconda.exe -ArgumentList "/InstallationType=JustMe /RegisterPython=1 /D=C:\Minicond
a" -Wait
