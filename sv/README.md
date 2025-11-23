# DEC–HEX Converter: Elektronikbyggsats (Modell A & B)

Denna *repository* innehåller *firmware*, *hardware-design* och komplett dokumentation för **DEC–HEX Converter**, ett roligt och funktionellt byggsats-projekt. Den är utformad för att ge hobbyister chansen att öva sina lödskunskaper samtidigt som de skapar ett användbart, fysiskt verktyg för omvandling mellan talsystem.

**Snabblänkar:**
* [**Documentation in English**](README.md)
* **Redo att köpa?** Se vår [**Beställningsguide**](ordering.md)

## 💡 Huvudfunktioner (Gemensamma för båda modellerna)

DEC–HEX Converter är byggd kring ett enkelt och taktilt gränssnitt:

* **Snabb Konvertering:** Omvandla snabbt mellan Decimala och Hexadecimala talbaser (och Binära beroende på modell).
* **Rotationsenkoder:** Använd den responsiva och exakta rotationsenkodern för att öka/minska värden. Den ger tydlig taktil *feedback* med distinkta steg för varje ändring.
* **Intuitiv Styrning:** **Vrid** för att räkna uppåt/nedåt, och **Tryck** för att nollställa räkneverket.
* **Hjärnan:** Båda enheterna använder den pålitliga **ATtiny85/45 mikrokontrollern** för bearbetning.
* **Kapsling:** Elektroniken är monterad i en robust **3D-printad inbyggnadslåda**.
* **Byggsatsen:** Levereras som ett komplett kit med ett **specialdesignat kretskort (PCB)**, alla nödvändiga komponenter och en **förprogrammerad mikrokontroller** – idealiskt för utmärkt **lödövning**.
* **Anpassning:** Mikrokontrollern är förprogrammerad, men den kan **programmeras om**. Källkoden finns tillgänglig i denna *repository*.

## 🧰 De Två Modellerna

DEC–HEX Converter finns i två varianter, anpassade efter olika önskemål kring displaytyp och lödkomplexitet:

### Modell A: Den Klassiska Displayen
* Visar värden med ljusa, klassiska **sjusegmentdisplayer**.
* Konverterar mellan **Decimal** och **Hexadecimal**.
* Har en **dedikerad strömbrytare** och ett strömsparläge.
* *Innebär mer lödning*.

### Modell B: Multi-Bas Displayen
* Visar värden på en **LCD (Liquid Crystal Display)**.
* Konverterar mellan **Decimal**, **Hexadecimal** och **Binär**.
* **Ingen strömbrytare**; använder ett *ultra-low-power* viloläge och vaknar omedelbart när vredet vrids eller trycks in.
* *Innebär mindre lödning*.

## 📂 Repository Innehåll

Denna *repository* är organiserad i specifika kataloger för varje modell. Alla projektfiler, dokumentation och källkod tillhandahålls nedan.

* **Allmänna Filer:** Här hittar du priser, licenser och ingången till den svenska dokumentationen.
* **Modellspecifikt:** Dokumentation, *hardware*-filer och *firmware* finns i de dedikerade katalogerna `/model_a/` och `/model_b/`.

---

### Modell A Filer (`/model_a/`)

Följande filer är unika för modellen med **sjusegmentdisplay**:

| Dokument/Fil | Beskrivning av Innehåll | Sökväg (Klicka för att se) |
| :--- | :--- | :--- |
| **Firmware (.ino)** | Arduino källkod för mikrokontrollern. | [model\_a.ino](model_a/model_a.ino) |
| **Monteringsanvisning** | Steg-för-steg guide för lödning och slutmontering. | [a\_assembly.md](model_a/docs/a_assembly.md) |
| **Specifikation** | Detaljerad funktionslista och tekniska specifikationer för Modell A. | [a\_spec.md](model_a/docs/a_spec.md) |
| **Komponentlista (BOM)** | Fullständig lista över alla nödvändiga komponenter. | [a\_bom.md](model_a/docs/a_bom.md) |
| **Kretsschema (PDF)** | Detaljerat *hardware* kretsschema. | [a\_schematic.pdf](model_a/docs/a_schematic.pdf) |
| **Bilder** | Foton på kretskort (fram/bak) och komponenter. | [images/](model_a/images/) |

### Modell B Filer (`/model_b/`)

Följande filer är unika för modellen med **LCD Multi-Bas Display**:

| Dokument/Fil | Beskrivning av Innehåll | Sökväg (Klicka för att se) |
| :--- | :--- | :--- |
| **Firmware (.ino)** | Arduino källkod för mikrokontrollern. | [model\_b.ino](model_b/model_b.ino) |
| **Monteringsanvisning** | Steg-för-steg guide för lödning och slutmontering. | [b\_assembly.md](model_b/docs/b_assembly.md) |
| **Specifikation** | Detaljerad funktionslista och tekniska specifikationer för Modell B. | [b\_spec.md](model_b/docs/b_spec.md) |
| **Komponentlista (BOM)** | Fullständig lista över alla nödvändiga komponenter. | [b\_bom.md](model_b/docs/b\_bom.md) |
| **Kretsschema (PDF)** | Detaljerat *hardware* kretsschema. | [b\_schematic.pdf](model_b/docs/b\_schematic.pdf) |
| **Bilder** | Foton på kretskort (fram/bak) och komponenter. | [images/](model_b/images/) |

### Allmänna Projektfiler (Rotkatalog)

| Fil | Beskrivning av Innehåll | Sökväg (Klicka för att se) |
| :--- | :--- | :--- |
| **Beställningsguide** | Information om priser och hur du kan köpa din byggsats. | [ordering.md](ordering.md) |
| **Svensk Dokumentation** | Ingången till all projektdokumentation på svenska. | [sv/](sv/README.md) |
| **Licens** | Information om mjukvaru- och *hardware*-licens. | [LICENSE.md](LICENSE.md) |

---

**Intresserad av att köpa en byggsats?** Läs mer i [**Beställningsguiden**](ordering.md)!

---

> **En anmärkning om garanti:** Detta är ett hobbyprojekt. Även om jag strävar efter hög kvalitet, lämnar jag ingen formell garanti. Vänligen kontakta mig direkt om du stöter på problem med enheten eller monteringsprocessen.

/Oskar Fornander
