# DEC–HEX Converter: DIY Electronics Kit (Models A & B)

This repository contains the firmware, hardware design, and comprehensive documentation for the **DEC–HEX Converter**, a fun and functional DIY calculator kit. It is designed to help hobbyists practice their soldering skills while creating a useful, physical tool for converting between number systems.

**Quick Navigation:**
* [**Documentation in Swedish**](sv/README.md)
* **Ready to Buy?** See the [**Ordering Guide**](ordering.md)

## Key Features (Common to Both Models)

The DEC–HEX Converter is built around a simple, tactile interface:

* **Fast Conversion:** Quickly convert between Decimal and Hexadecimal number bases (and Binary depending on the model).
* **Rotary Encoder:** Use the responsive and precise rotary encoder to increment/decrement values. It provides satisfying tactile feedback with distinct steps for each change.
* **Intuitive Control:** **Rotate** to count up/down, and **Press** to reset the counter to zero.
* **The Brain:** Both units use the reliable **ATtiny85/45 microcontroller** for processing.
* **Enclosure:** The electronics are housed in a robust **3D-printed enclosure**.
* **The Kit:** Delivered as a complete kit with a **custom-designed PCB**, all necessary components, and a **pre-programmed microcontroller**—ideal for excellent **soldering practice**.
* **Customization:** The microcontroller is pre-programmed, but it can be **re-programmed**. The source code is available within this repository.

## Two Models

The DEC–HEX Converter comes in two variants, catering to different preferences for display type and soldering complexity:

### Model A: The Classic Display
* Displays values using bright, classic **seven-segment displays**.
* Converts between **Decimal** and **Hexadecimal**.
* Features a **dedicated power switch** and a power-saving sleep mode.
* *Involves more soldering.*

### Model B: The LCD Multi-Base Display
* Displays values on an **LCD (Liquid Crystal Display)**.
* Converts between **Decimal**, **Hexadecimal**, and **Binary**.
* **No power switch**; utilizes an ultra-low-power sleep mode and wakes up instantly when the knob is turned or pressed.
* *Involves less soldering.*

## Repository Contents

This repository is organized into specific directories for each model. All project files, documentation, and source code are provided below.

* **General Files:** Find pricing, licenses, and the Swedish documentation entry point.
* **Model-Specific:** Documentation, hardware files, and firmware are located within the dedicated `/model_a/` and `/model_b/` directories.

---

### Model A Files (`/model_a/`)

The following files are unique to the **seven-segment display** model:

| Document/File | Content Description | Path (Click to View) |
| :--- | :--- | :--- |
| **Firmware (.ino)** | The Arduino source code for the microcontroller. | [model\_a.ino](model_a/model_a.ino) |
| **Assembly Guide** | Step-by-step instructions for soldering and assembly. | [a\_assembly.md](model_a/docs/a_assembly.md) |
| **Specification** | Detailed feature list and technical specifications for Model A. | [a\_spec.md](model_a/docs/a_spec.md) |
| **Parts List** | Full Bill of Materials (BOM) listing all included components. | [a\_parts\_list.md](model_a/docs/a_parts_list.md) |
| **Schematics (PDF)** | Detailed hardware circuit diagram. | [a\_schematic.pdf](model_a/docs/a_schematic.pdf) |
| **Images** | Photos of the parts and the assemlbe unit. | [images/](model_a/images/) |

### Model B Files (`/model_b/`)

The following files are unique to the **LCD multi-base display** model:

| Document/File | Content Description | Path (Click to View) |
| :--- | :--- | :--- |
| **Firmware (.ino)** | The Arduino source code for the microcontroller. | [model\_b.ino](model_b/model_b.ino) |
| **Assembly Guide** | Step-by-step instructions for soldering and assembly. | [b\_assembly.md](model_b/docs/b_assembly.md) |
| **Specification** | Detailed feature list and technical specifications for Model B. | [b\_spec.md](model_b/docs/b_spec.md) |
| **Parts List** | Full Bill of Materials (BOM) listing all included components. | [b\_parts\_list.md](model_b/docs/b_parts_list.md) |
| **Schematics (PDF)** | Detailed hardware circuit diagram. | [b\_schematic.pdf](model_b/docs/b\_schematic.pdf) |
| **Images** | Photos of the parts and the assemlbe unit. | [images/](model_b/images/) |

### General Project Files

| File | Content Description | Path (Click to View) |
| :--- | :--- | :--- |
| **Order Guide** | Information on pricing and how to order your kit. | [ordering.md](ordering.md) |
| **Swedish Docs** | Entry point for all project documentation in Swedish. | [sv/](sv/README.md) |
| **License** | Software and hardware license information. | [LICENSE.md](LICENSE.md) |
---

**Interested in purchasing a kit?** Read more in the [**Ordering Guide**](ordering.md)!

---

> **A Note on Guarantees:** This is a hobby project. While I strive for high quality, I offer no formal guarantee. Please contact me directly if you encounter any issues with the unit or assembly process.

/Oskar Fornander
