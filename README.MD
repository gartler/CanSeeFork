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
300 Set the WiFi ssid for Station mode (Home)
301 Set the WiFi password for Station mode (Password)

z: Reboot (needed for all configuration changes except debug mode)

Defaults are shown in brackets. Defaults are used, when no invalid configuration
is in the EEPROM.

The device can be forced to defaults by pressing BUT (GPIO0) with half a second
of resetting the device.

Note: For ISO-TP: multiframe a multiframe can be received but commands are now
only supported in single frame, so a maximum of 6 bytes

For maximum speed use n100,0001000000 (Android)
```