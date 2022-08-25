# CyGarage Schematics

All files were produced using [EasyEDA](https://easyeda.com/). The following is a list of files and what they are:

- Model1-Single-Door/COM_CyGarage.csv - The bill of materials in CSV format.

- Model1-Single-Door/PCB_CyGarage.pdf - A PDF version of the PCB view.

- Model1-Single-Door/SCH_CyGarage.pdf - A PDF version of the schematic view.

- Model1-Single-Door/PCB_CyGarage.json - The original EasyEDA CAD file of the schematic.

- Model1-Single-Door/SCH_CyGarage.json - The original EasyEDA CAD file of the PCB.

- Model1-Single-Door/gerber - Exported Gerber files useful for importing into other CAD programs or for sending to fab houses that do not support Easy EDA files.

- Model2-Dual-Door/* - Contains all of the same things, but for the 2 door model.

## NOTES

DC1 and DC2 (and DC3 and DC4 for Model2) are actually intended to connect to the limit switches on the garage door track. These are used for sensing door position.

The power supply used is a 5VDC 1A supply which connects to JP1.

When DC1 (connected to JP2) closes, the controller will consider the door to be in the 'open' state. This is the limit switch on the near-end of the trolley track (closest to the opener). When this switch is closed, the garage door opener stops because the door should be fully opened.

When DC2 (connected to JP3) closes the controller will consider the door to be in the 'closed' state (unless DC1 is also closed, which should never happen). This is the limit switch on the far-end of the trolley track (closest to the garage door). When this switch is closed, the garage door opener stops because the door should be fully closed. When both switches are opened, the door is considered to be "ajar".

The OPENER terminals should connect to the button terminals on the garage door opener. When the relay activates, these will be connected for approx. 2.5 seconds and then open again, thus simulating a momentary button press which should in turn activate the garage door opener.

Additionally, I opted to use female stacking headers where the pins of the ESP8266 should go (creating a "socket" of sorts) to allow the module to be removable. This is optional, but handy in the event that the module ever becomes damaged or if the EEPROM wears out, or something of that nature, it can be easily replaced.

DC1 and DC2 (and DC3 and DC4 for Model2) can also be connected to door contacts if using on a garage door that doesn't use limit switches (such as automatic industrial overhead door openers).
