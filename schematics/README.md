# CyGarage Schematics

All files were produced using [Fritzing](https://fritzing.org/home/). The following is a list of files and what they are:

- CyGarage_bb.pdf - A PDF version of the breadboard view.

- CyGarage_bom.html - The bill of materials in HTML format.

- CyGarage_pcb.pdf - A PDF version of the PCB view.

- CyGarage_schema.pdf - A PDF version of the schematic view.

- CyGarage.fzz - The original Fritzing CAD file.

## NOTES

DC1 and DC2 are actually magnetic door contacts. I chose the ones from Adafruit which can be found [here](https://www.adafruit.com/product/375?gclid=EAIaIQobChMI59zX7pOo4wIVE4zICh2WnAJ3EAQYASABEgKrYfD_BwE). These contacts should connect to J2 and J3 and are not polarity sensitive.

The Adafruit Power Relay FeatherWing was used because it had a 3V trigger and built-in logic-level and protection circuitry (which means no problems connecting to the ESP8266 and no additional transistors or diodes needed). This board needs to have a pin configured as the 'signal' (trigger) line. I chose pin A which is to the immediate right of the GND pin. This is done by simply soldering the jumper pads next to that pin on the underside of the board.

The power supply used is a 5VDC 1A supply which connects to J1.

When DC1 (connected to J2) closes, the controller will consider the door to be in the 'open' state. This is because the contact and associated magnet should be mounted so that the magnet aligns with the contact when the garage door is all the way open.

When DC2 (connected to J3) closes the controller will consider the door to be in the 'closed' state (unless DC1 is also closed, which should never happen). This is because the contact and associated magnet should be mounted so that the magnet aligns with the contact when the garage door is all the way closed. Should the contact separate from the magnet, the door will be considered to be in the 'ajar' state.

NO and COM on the relay should connect to the button terminals on the garage door opener. When the relay activates, these will be connected for approx. 1 second and then open again, thus simulating a momentary button press which should in turn activate the garage door opener.
