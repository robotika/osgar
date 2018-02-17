CAN Bus Bridge
==============

"CAN Bus Bridge" is used for communication between PC serial port and CAN
bus network. The module was developed and manufactured by RobSys.cz

Communication Protocol
----------------------

The bridge is in configuration mode after power up (or application of "RESET"
via Serial.setDTR(0)). This is reported once via set of 0xFF bytes followed
by 0xFE 0x10 packets. After that is bridge waiting for "sync" via ten 0xFF
bytes.

You can configure CAN bridge parameters after "sync" with bridge. For example
to set CAN communication speed to 1Mb use 0xFE 0x57.

Once you are finished with configuration you can switch mode to operational
via 0xFE 0x31. The bridge starts to listen on CAN bus and forward all available
(or selected, if configured) messages to PC via serial port.

