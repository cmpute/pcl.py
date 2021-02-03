Invoke-WebRequest -Uri https://repo.anaconda.com/miniconda/Miniconda2-latest-Windows-x86_64.exe -OutFile miniconda.exe
$miniconda = Start-Process ./miniconda.exe /InstallationType=JustMe /RegisterPython=1 /S /D=C:\Miniconda
$miniconda.WaitForExit()
