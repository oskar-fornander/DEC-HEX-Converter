# DEC–HEX Converter: DIY Electronics Kit (Models A & B)

This repository contains the firmware, hardware design, and comprehensive documentation for the **DEC–HEX Converter**, a fun and functional DIY calculator kit. It is designed to help hobbyists practice their soldering skills while creating a useful, physical tool for converting between number systems.

**Quick Navigation:**
* [**Documentation in Swedish**](sv/README.md)
* **Ready to Buy?** See our [**Ordering Guide**](BUY.md)

## 💡 Key Features (Common to Both Models)

The DEC–HEX Converter is built around a simple, tactile interface:

* **Fast Conversion:** Quickly convert between Decimal and Hexadecimal number bases (and Binary depending on the model).
* **Rotary Encoder:** Use the responsive and precise rotary encoder to increment/decrement values. It provides satisfying tactile feedback with distinct steps for each change.
* **Intuitive Control:** **Rotate** to count up/down, and **Press** to reset the counter to zero.
* **The Brain:** Both units use the reliable **ATtiny85/45 microcontroller** for processing.
* **Enclosure:** The electronics are housed in a robust **3D-printed enclosure**.
* **The Kit:** Delivered as a complete kit with a **custom-designed PCB**, all necessary components, and a **pre-programmed microcontroller**—ideal for excellent **soldering practice**.
* **Customization:** The microcontroller is pre-programmed, but it can be **re-programmed**. The source code is available within this repository.

## 🧰 The Two Models

The DEC–HEX Converter comes in two variants, catering to different preferences for display type and soldering complexity:

### Model A: The Classic Display
* Displays values using bright, classic **seven-segment displays**.
* Converts between **Decimal** and **Hexadecimal**.
* Features a **dedicated power switch** and a power-saving sleep mode.
* *Involves more soldering.*

### Model B: The Multi-Base Display
* Displays values on an **LCD (Liquid Crystal Display)**.
* Converts between **Decimal**, **Hexadecimal**, and **Binary**.
* **No power switch**; utilizes an ultra-low-power sleep mode and wakes up instantly when the knob is turned or pressed.
* *Involves less soldering.*

## 📂 Repository Contents

This GitHub repository provides everything you need to build, modify, and understand the DEC–HEX Converter:

| Directory/File | Content Description |
| :--- | :--- |
| `/firmware` | Source code and compiled files for the ATtiny85/45 microcontroller. |
| `/hardware` | PCB design files (Schematics, Layouts) and component BOM (Bill of Materials). |
| `/enclosure` | 3D models (STL files) for the printed enclosures for both Model A and B. |
| `/docs` | Detailed assembly instructions and user guides. |
| **`BUY.md`** | Information on pricing and how to order your kit. |
| **`sv/`** | Full project documentation in Swedish. |

## 📂 Repository Contents

This GitHub repository is structured to provide all files needed for both building and understanding the DEC–HEX Converter kits:

| Path/Directory | Content Description | Applies To |
| :--- | :--- | :---: |
| **`README.md`** | Project introduction, key features, and model comparison. | Both |
| **`buy.md`** | Information on pricing and how to order your kit. | Both |
| **`/model_A/`** | Files specific to **Model A** (Seven-Segment Display). | A |
| **`/model_B/`** | Files specific to **Model B** (LCD Multi-Base Display). | B |
| **`/model_[A/B]/docs/`** | Contains all textual and schematic documentation for assembly. | A & B |
| `model_[A/B]/docs/assembly_instructions.md` | Step-by-step guide for soldering and final assembly. | A & B |
| `model_[A/B]/docs/description.md` | Detailed marketing and feature description of the model. | A & B |
| `model_[A/B]/docs/parts_list.md` | Full Bill of Materials (BOM) listing required components. | A & B |
| `model_[A/B]/docs/schematics.pdf` | Detailed hardware circuit diagram. | A & B |
| **`/model_[A/B]/images/`** | Photos of the PCB and other hardware components. | A & B |
| `model_[A/B]/model_[A/B].ino` | The Arduino source code (firmware) for the microcontroller. | A & B |
| **`/sv/`** | Full project documentation and README in Swedish. | Both |
| `LICENSE.txt` | Software and hardware license information. | Both |

*Note: Paths are generalized using `[A/B]` for brevity. Replace with `A` or `B` for the specific model's files.*

## 📂 Repository Contents

This repository is organized into specific directories for each model. All project files, documentation, and source code are provided below.

* **General Files:** Find pricing, licenses, and the Swedish documentation entry point.
* **Model-Specific:** Documentation, hardware files, and firmware are located within the dedicated `/model_A/` and `/model_B/` directories.

---

### Model A Files (`/model_A/`)

The following files are unique to the **seven-segment display** model:

| Document/File | Content Description | Path (Click to View) |
| :--- | :--- | :--- |
| **Firmware (.ino)** | The Arduino source code for the microcontroller. | [model\_A.ino](model_A/model_A.ino) |
| **Assembly Guide** | Step-by-step instructions for soldering and assembly. | [assembly\_instructions.md](model_A/docs/assembly_instructions.md) |
| **Description** | Detailed feature list and specifications for Model A. | [description.md](model_A/docs/description.md) |
| **Parts List (BOM)** | Full Bill of Materials listing all required components. | [parts\_list.md](model_A/docs/parts_list.md) |
| **Schematics (PDF)** | Detailed hardware circuit diagram. | [schematics.pdf](model_A/docs/schematics.pdf) |
| **Images** | Photos and renderings of the PCB and components. | [images/](model_A/images/) |

### Model B Files (`/model_B/`)

The following files are unique to the **LCD multi-base display** model:

| Document/File | Content Description | Path (Click to View) |
| :--- | :--- | :--- |
| **Firmware (.ino)** | The Arduino source code for the microcontroller. | [model\_B.ino](model_B/model_B.ino) |
| **Assembly Guide** | Step-by-step instructions for soldering and assembly. | [assembly\_instructions.md](model_B/docs/assembly_instructions.md) |
| **Description** | Detailed feature list and specifications for Model B. | [description.md](model_B/docs/description.md) |
| **Parts List (BOM)** | Full Bill of Materials listing all required components. | [parts\_list.md](model_B/docs/parts_list.md) |
| **Schematics (PDF)** | Detailed hardware circuit diagram. | [schematics.pdf](model_B/docs/schematics.pdf) |
| **Images** | Photos and renderings of the PCB and components. | [images/](model_B/images/) |

### General Project Files

| File | Content Description | Path (Click to View) |
| :--- | :--- | :--- |
| **Buy Guide** | Information on pricing and how to order your kit. | [buy.md](buy.md) |
| **Swedish Docs** | Entry point for all project documentation in Swedish. | [sv/](sv/README.md) |
| **License** | Software and hardware license information. | [LICENSE.txt](LICENSE.txt) |


---

**Interested in purchasing a kit?** Read more in our [**Ordering Guide**](BUY.md)!

---

> **A Note on Guarantees:** This is a hobby project. While I strive for high quality, I offer no formal guarantee. Please contact me directly if you encounter any issues with the unit or assembly process.

/Oskar Fornander
