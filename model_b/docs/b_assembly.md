# Assembly Guide: DEC–HEX Converter, Model B

This guide provides the simple steps required to assemble your Model B kit. Thanks to the custom-designed PCB (Printed Circuit Board), the component layout is clearly marked, making this a straightforward build and an excellent soldering practice project.

**Estimated time:** approx. 1 hour (depending on soldering experience).

**Tools required:** Soldering iron, solder, side cutters (for trimming leads), small hex screwdriver, and hobby glue (e.g., hot glue or super glue).

Refer to the [**Parts List**](b_parts_list.md) (name and number of pieces) to correctly identify each component. All through-hole components are inserted from the Component Side (the side with the silkscreen markings) and soldered on the reverse side (the Solder Side). All component locations are clearly marked on the Component Side.

### 1. Solder all Components

Start by soldering components flush to the PCB, typically from shortest to tallest.

* **Resistors:** Solder the resistors, following the PCB markings for their values (check the Parts List for color codes if unsure). Note: skip the two pull up resistors for the rotary encoder, according to markings in the image and schematics.
* **Capacitors:** Solder the ceramic capacitors according to their values. Solder the electrolytic capacitor and ensure it is placed with the correct orientation/polarity according to the PCB markings. This capacitor can be angled down toward the PCB to save vertical space.
* **Transistor and LDO Regulator:** Solder the Transistor and LDO regulator into their respective positions. Note that they both use similar packages; compare the markings on the components to differentiate them and ensure correct orientation.
* **IC Socket (DIP-8):** Solder the socket for the ATtiny85/45 microcontroller. Pay attention to the notch (orientation marker) on the socket; it must match the marker printed on the PCB.
* **Rotary Encoder:** Place and solder the five pins of the rotary encoder as well as the mounting tabs to secure it firmly.
* **Pin Header and Battery Box Wires:** Solder the pin header (4 pins) for the LCD connection. Then solder the battery box wires, either directly to the PCB or to a separate pin header. Note the connection polarity.

### 2. Insert Microcontroller

* Carefully insert the **pre-programmed ATtiny85/45 microcontroller** into its socket. The notch on the IC body must match the notch on the socket and the PCB silkscreen.

### 3. Prepare the Battery Box

* Install the four AAA batteries.
* Ensure the battery box's switch is set to the **ON** position. The unit requires this switch to be permanently ON; when inactive, the unit enters a minimal power consumption sleep mode.
* Apply hobby glue (or double-sided tape) to the fixed bottom of the **battery box** (the side opposite the opening lid) and securely glue it onto the designated area on the enclosure's **bottom plate**.

### 4. Mount the PCB and LCD

* Place the assembled PCB into the main part of the **3D-printed enclosure**.
* Use two of the provided **M3 screws** and **spacers (2mm)** to fasten the PCB securely into the enclosure holes. The spacers ensure proper screw depth and provide a secure mounting for the PCB. Screw the rotary encoder on from the front and put the knob on.
* Connect the included **4-pin cable** to the LCD and the PCB. **IMPORTANT:** Ensure the connection is correct by comparing the printed text near the pins. The pins are in the same order on the PCB, even though the labels may differ (VSS = GND, VDD = VCC, SCK = SCL). Incorrect polarity may damage the components.
* Mount the LCD using four of the **M3 screws** and **spacers (5mm)**. Note that the cables must be connected before the LCD is inserted and screwed in place. For correct orientation, the text on the backside should be oriented in the correct way.

### 5. Close the Enclosure

* Place the bottom plate (with the glued-on battery box) onto the main enclosure.
* Secure the enclosure using the **M4 screws** through the bottom plate and into the main housing.

**Congratulations! Your DEC–HEX Converter, Model B, is now fully assembled and ready for use!**

---

/Oskar Fornander
