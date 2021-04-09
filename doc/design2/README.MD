This design is provided by EliotFR. See also his post [here](https://canze.fisch.lu/this-is-a-speculative-post/#comment-2678)

# Overview
This system works with 2 PCBs, the lower and the upper PCB, joined by a 7PIN pin header. A few pictures are included in this directory

The 7 PIN 2.54mm junction:  
PIN 1 SQUARE : +3.3V  
PIN 2 CIRCULAR : To PIN OBD 12 – CAN 1 LOW  
PIN 3 ROUND : To PIN OBD 13 – CAN 1 HIGH  
PIN 4 ROUND : To PIN OBD 16 – +12V  
PIN 5 ROUND : To pin OBD 14 – CAN 0 LOW  
PIN 6 ROUND : To pin OBD 6 – CAN 0 HIGH  
PIN 7 ROUND : To pin OBD 4 + 5 – GND  

CAN 0 = CAN ISO15765-4  
CAN 1 = Renault proprietary CAN network  

# The lower PCB:  
Designed to fit on the OBD connector and provide the 3.3V from the 12v pin.  
You can find 4 variations:  
– Without power supply  
– With the “tiny dc-dc converter” prefabricated module on aliexpress  
– Based on the mp2451 by monolitic power  
– Based on the mp2315 by monolitic power  
Designs based on mp weren’t tested in real life.

# The upper PCB:  
– ESP32 + 5 LEDS + CAN 0 + Reset Button  
– ESP32 + 5 LEDS + CAN 1 without Reset Button  
JP CAN0 and JP CAN1 are two jumpers to enable/disable the 120 OHM terminator resistors.

# How to program it:
Edit the cs_config.mode_leds value in the config.cpp to enable the multi led function:  
0 = No led  
1 = LED_SINGLE  
2 = LED_MULTI  


    curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
    python3 get-platformio.py
    export PATH=$PATH:\~/.platformio/penv/bin
    git clone https://gitlab.com/jeroenmeijer/cansee.git
    cd cansee
    nano src/config.cpp
    platformio run –target upload

