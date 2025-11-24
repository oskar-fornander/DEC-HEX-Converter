# Assembly Guide: DEC–HEX Converter, Model A

This guide provides the simple steps required to assemble your Model A kit. Thanks to the custom-designed PCB, the component layout is clearly marked, making this a straightforward build and an excellent **soldering practice** project.

**Estimated time:** 1–2 hours (depending on soldering experience).

**Tools required:** Soldering iron, solder, side cutters (for trimming leads), small hex screwdriver, and hobby glue (e.g., hot glue or super glue).

It is crucial to follow the recommended soldering order to ensure easy access to all pads. Refer to the [**Parts List**](a_parts_list.md) (name and number of pieces) to correctly identify each component. All through-hole components are inserted from the Component Side (the side with the silkscreen markings) and soldered on the reverse side (the Solder Side). All component locations are clearly marked on the Component Side silkscreen.

### 1. Solder all Components, Starting with Low-Profile Parts

Start by soldering components flush to the PCB, typically from shortest to tallest.

* **Decoupling Capacitors (100nF, 6pcs):** These small ceramic capacitors should be soldered first. This is critical for clearance, as they must fit beneath the subsequent IC sockets.
* **IC Sockets (five DIP-16 and one DIP-8):** Solder the sockets for the ATtiny85/45 and the shift registers 74HC595. Pay attention to the notch (orientation marker) on the socket; it must match the marker printed on the PCB. Ensure the decoupling capacitors are soldered before this step.
* **Resistors:** Solder the resistors, following the PCB markings for their values (check the Parts List for color codes if unsure).
* **Capacitors, diode and transistor:** Solder the rest of the smaller components in place according to their values. Ensure the transistor, diode, and electrolytic capacitor are placed with the correct orientation/polarity. Refer to the PCB markings (e.g., diode stripe, capacitor negative leg).
* **Power Switch and Battery Box:** Solder the wires of the dedicated ON/OFF switch and battery box, either directly to the pcb or on pin headers.
* **7-Segment Displays:** Insert and solder the five 7-segment displays. Ensure they are placed flat against the PCB and in the correct orientation.
* **Rotary Encoder:** Place and solder the five pins of the rotary encoder as well as the mounting tabs to secure it firmly.

### 2. Insert Microcontroller and Integrated Circuits

* Carefully insert the **pre-programmed ATtiny85/45 microcontroller** and the **74HC595 Shift Registers** into their sockets. The notch on the IC body must match the notch on the socket and the PCB silkscreen.

### 3. Prepare the Battery Box

* Install the three AAA batteries.
* Ensure the battery box's switch is set to the **ON** position. The unit requires this to be permanently ON since the main power switch on the PCB controls power.
* Apply hobby glue (or double-sided tape) to the fixed bottom of the **battery box** (the side opposite the opening lid) and glue it securely onto the designated area on the enclosure's **bottom plate**.

### 4. Mount the PCB

* Place the assembled PCB into the main part of the **3D-printed enclosure**.
* Use the provided **M3 screws** and **spacers** to fasten the PCB securely to the enclosure standoffs. The spacers ensure proper screw depth and provide a secure mounting for the PCB.
* Mount the **Power Switch** in its opening on the enclosure and connect its wires to the switch.

### 5. Close the Enclosure

* Place the bottom plate (with the glued-on battery box) onto the main enclosure.
* Secure the enclosure using the **M4 screws** through the bottom plate and into the main housing.

**Congratulations! Your DEC–HEX Converter, model A, is now fully assembled and ready for use!**

---

/Oskar Fornander
