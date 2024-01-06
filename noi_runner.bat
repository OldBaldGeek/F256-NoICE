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
:: When you tell NoICE to LOAD a file, the name of the file will be passed
:: to this file. The file will be loaded, and the monitor reloaded since fnxmgr
:: may overwrite the reset vector, and it resets the target.

:: The loader we are using (your path will probably differ)
:: TODO: normal set fnx="\utilities\fnxmgr.zip"
set fnx="C:\repos\FoenixMgr\FoenixMgr\fnxmgr.py"

:: The monitor to load (your path will probably differ)
set mon="C:\repos\F256-NoICE\Mon6502_F256_Merlin.pgz"

:: Put the target CPU into reset until we do ALL our commands
:: This also suspends reset/release by further invocations until we release_reset
python %fnx% --assert_reset
if not %ERRORLEVEL% == 0 pause

:: This version of the monitor owns the interrupt vectors and lives at F900
python %fnx% --boot RAM
if not %ERRORLEVEL% == 0 pause

:: If we have a filename parameter, load it (program to debug)
IF "%~1" == "" goto nofile
python %fnx% --upload-srec %1
if not %ERRORLEVEL% == 0 pause
:nofile

:: Load the NoICE monitor, which includes reset vector
python %fnx% --run-pgz %mon%
if not %ERRORLEVEL% == 0 pause

:: Release the target CPU from RESET, causing the monitor to run.
python %fnx% --release_reset
if not %ERRORLEVEL% == 0 pause
