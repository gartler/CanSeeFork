```
Boot device with USB connected. Hookup to a terminal emulator, Bluetooth
terminal emulator or Telnet over WiFi.

Commands:
a: Show all buffered free CAN_frames

gxxx: Get free frame with id xxx

ixxx,yyyy: Get ISO-TP frame with id xxx, PID yyyy.

nxxx,config: set EEPROM configuration
xxx config
=== =======================================================================
100 Set the flags configuration as hex as ssbbwwlldd (01010200ff)
      ss: Serial over USB
        00 for off, any other for on
      bb: Bluetooth
        00 for off, any other for on
      ww: WiFi
        00 for off, 01 for Station mode, 02 for Soft Access Point mode
        For station mode a DHCP server is assumed
      ll: Use leds
        00 for off, ff for on
      dd: Debug
        00 for off, ff for on. A bit pattern can be used and added
          01: show CANbus receive free CAN_frame_std
          02: show CANbus receive ISO-TP CAN_frame_std
          04: show Commands issued
          08: show free frame Commands
          10: show ISO-TP Commands
200 Set the Bluetooth name (CANSee)
201 Set the Bluetooth pin (1234, not implemented by the API!!)
300 Set the WiFi ssid for Soft Access Point mode (CANSee)
301 Set the WiFi password for Soft Access Point mode (CANSeeMe)
400 Set the WiFi ssid for Station mode (Home)
401 Set the WiFi password for Station mode (Password)
500 Set CANbus parameter for can0 in hex as sprxtx
      sp: speed
        00 for 100 kbps
        01 for 125 kbps
        02 for 250 kbps
        04 for 500 kbps (for can1)
        03 for 800 kbps (for can0)
        05 for 1 mbps
      rx: Rx pin of transceiver (4 for can0, 18 for can1)
      tx: Tx pin of transceiver (5 for can0, 19 for can1)
      Note that this is a very advanced setting and selecting pins that are
      internally used on your board (notably 6-11) will crash the board. Also
      pins 34 and up are input only.
501 Set CANbus parameter for can1 in hex as sprxtx (see above).
      Usage of the semi-secondary CANbus by switching the single controller to
      another pin pair is not supported and basically fails spectacularly.

z: Reboot (needed for all configuration changes except debug mode)

Defaults are shown in parenthesis. Defaults are used, when no invalid
configuration is in the EEPROM.

The device can be forced to defaults by pressing BUT (GPIO0) within half a
second of resetting the device.

Note: For ISO-TP: a multiframe message can be received but commands are now
only supported in single frame, so a maximum of 6 bytes

For maximum speed use n100,0001000000 (Android) or n100,0000020000 (iOS)
```
