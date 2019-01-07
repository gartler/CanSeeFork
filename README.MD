Boot device with USB connected. Hookup to a terminal emulator

Commands:
a: show all buffered free CAN_frames

gxxx: get free frame with id gxxx

ixxx,yyyy: get iso-tp frame with id xxx, PID yyyy.

nxxx,config: set EPROM configuration
xxx
100 set the flags config as hex as ssbbwwlldd (01010200ff)
      ss=serial over USB
        00 for off, any other for on
      bb=bluetooth
        00 for off, any other for on
      ww=wifi
        00 for off, 01 for Station mode, 02 for Soft Access Point mode
        For station mode a DHCP server is assumed
      ll=use leds
        00 for off, ff for on
      dd=debug
        00 for off, ff for on. A bit pattern can be used and added
          01=show CANbus receive free CAN_frame_std
          02=show CANbus receive ISO-TP CAN_frame_std
          04=show Commands issued
          08=show free frame Commands
          10=show ISO-TP Commands
200 set the bluetooth name (CANSee)
201 set the Bluetooth pin (1234, not implemented by the API!!)
300 set the Wifi ssid for Soft Access Point mode (CANSee)
301 set the Wifi password for Soft Access Point mode (CANSeeMe)
300 set the Wifi ssid for Station mode (Home)
301 set the Wifi password for Station mode (Password)

z: reboot (needed for all configuration changes except debug mode)

Defaults are shown in brackets. Defaults are used, when no invalid configuration
is in the EEPROM.

The device can be forced to defaults by pressing BUT (GPIO0) with half a second
of resetting the device.

Note: For ISO-TP: multiframe a multiframe can be received but commands are now
only supported in single frame, so a maximum of 6 bytes

For maximum speed use n100,0001000000 (Android)
