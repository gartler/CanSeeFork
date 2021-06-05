ESP32 / Arduino firmware as a fast CANbus access gateway for the CanZE app. See Wiki for documentation

Build the firmware using PlatformIO with VSCode and the PlaformIO plugin

Documentation is provided as in-code Doxygen style comment
to generate the documentation, change to the root folder
of this project (where this platformio.ini file is) and
issue the doxygen command. The output will be directed to
the html folder. If doxygen was installed using all default
options and without a path to it set, the proper command
from PowerShell is (mark the back-tick)
```
C:\Program` Files\doxygen\bin\doxygen
```
And display it using
```
C:\Program` Files\Mozilla` Firefox\firefox.exe file:///C:/Users/notic/Documents/PlatformIO/Projects/cansee/html/index.html
```

Note: please be aware that PlatformIO issues a compiler warning for
```
...\.platformio\packages\framework-arduinoespressif32\cores\esp32\esp32-hal-spi.c
```
This can be ignored.
