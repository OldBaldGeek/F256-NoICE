:: This command file is intended to be used as an interface between NoICE
:: and fnxmgr to load the NoICE monitor and user programs.
::
:: To configure NoICE:
:: 1) Run NoICE (possibly getting timeout errors if no monitor is running)
:: 2) Select "Options," "Extensions..."
:: 3) Under "When NoICE starts..." enter the path to this file.
::    Select "Wait for program to complete before continuing."
:: 4) Under "At LOAD command..." enter the path to this file, and add "%1"
::    to pass the name of the file to be loaded as a parameter.
:: 5) Select "OK"
::
:: When NoICE runs on the PC, it will invoke this file to load the monitor.
::
:: When you tell NoICE to LOAD a file, the name of the file will be passed
:: to this file. We stop the target (which is in the NoICE monitor), load
:: the file, and resume target execution

:: The loader we are using (your path will probably differ)
:: set fnx="C:\repos\FoenixMgr\FoenixMgr\fnxmgr.py"
set fnx="\utilities\fnxmgr.zip"

:: The monitor to load (your path will probably differ)
set mon="C:\repos\F256-NoICE\Mon6502_F256_kernel.pgz"

:: If we have a filename parameter, load it (program to debug)
IF "%~1" == "" goto nofile

python %fnx% --stop
if not %ERRORLEVEL% == 0 pause

python %fnx% --upload-srec %1
if not %ERRORLEVEL% == 0 pause

python %fnx% --start
if not %ERRORLEVEL% == 0 pause
exit


:nofile
:: Load the NoICE monitor
python %fnx% --run-pgz %mon%
if not %ERRORLEVEL% == 0 pause
