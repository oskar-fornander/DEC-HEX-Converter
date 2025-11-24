# DEC–HEX Converter: Elektronikbyggsats (Modell A & B)

Denna *repository* innehåller *firmware*, *hardware-design* och komplett dokumentation för **DEC–HEX Converter**, ett roligt och funktionellt byggsats-projekt. Den är utformad för att ge hobbyister chansen att öva sina lödskunskaper samtidigt som de skapar ett användbart, fysiskt verktyg för omvandling mellan talsystem.

**Snabblänkar:**
* [**Documentation in English**](../README.md)
* **Redo att köpa?** Se [**Beställningsguiden**](beställning.md)

## 💡 Huvudfunktioner (Gemensamma för båda modellerna)

DEC–HEX Converter är byggd kring ett enkelt och taktilt gränssnitt:

* **Snabb konvertering:** Omvandla snabbt mellan *decimala* och *hexadecimala* talbaser (och *binära* beroende på modell).
* **Rotationsenkoder:** Använd den responsiva och exakta rotationsenkodern för att öka/minska värden. Den ger tydlig taktil *feedback* med distinkta steg för varje ändring.
* **Intuitiv styrning:** **Vrid** för att räkna uppåt/nedåt, och **tryck** för att nollställa räkneverket.
* **Hjärnan:** Båda enheterna använder den pålitliga **ATtiny85/45 mikrokontrollern** för bearbetning.
* **Kapsling:** Elektroniken är monterad i en robust **3D-printad inbyggnadslåda**.
* **Byggsatsen:** Levereras som ett komplett kit med ett **specialdesignat kretskort (PCB)**, alla nödvändiga komponenter och en **förprogrammerad mikrokontroller** – idealiskt för **lödövning**.
* **Anpassning:** Mikrokontrollern är förprogrammerad, men den kan **programmeras om**. Källkoden finns tillgänglig i denna *repository*.

## 🧰 De Två Modellerna

DEC–HEX Converter finns i två varianter, anpassade efter olika önskemål kring displaytyp och lödkomplexitet:

### Modell A: Sjusegmentdisplay
* Visar värden med ljusa, klassiska **sjusegmentdisplayer**.
* Konvertera mellan **decimal** och **hexadecimal**.
* Har en **dedikerad strömbrytare** och ett strömsparläge.
* *Innebär mer lödning*.

### Modell B: LCD-display
* Visar värden på en **LCD (Liquid Crystal Display)**.
* Konverterar mellan **decimal**, **hexadecimal** och **binär**.
* **Ingen strömbrytare**; använder ett *extremt strömsnålt* viloläge och vaknar omedelbart när vredet vrids eller trycks in.
* *Innebär mindre lödning*.

## 📂 Repositoryts innehåll

Denna *repository* är organiserad i specifika kataloger för varje modell. Alla projektfiler, dokumentation och källkod tillhandahålls nedan.

* **Allmänna Filer:** Här hittar du priser, licenser och ingången till den svenska dokumentationen.
* **Modellspecifikt:** Dokumentation, *hardware*-filer och *firmware* finns i de dedikerade katalogerna `/model_a/` och `/model_b/`.

---

### Modell A filer (`/model_a/`)

Följande filer är unika för modellen med **sjusegmentdisplay**:

| Dokument/fil | Beskrivning av innehåll | Sökväg (klicka för att se) |
| :--- | :--- | :--- |
| **Firmware (.ino)** | Arduino källkod för mikrokontrollern. | [model\_a.ino](../model_a/model_a.ino) |
| **Monteringsanvisning** | Steg-för-steg guide för lödning och slutmontering. | [a\_assembly.md](a_montering.md) |
| **Specifikation** | Detaljerad funktionslista och tekniska specifikationer för Modell A. | [a\_spec.md](a_beskrivning.md) |
| **Komponentlista (BOM)** | Fullständig lista över alla nödvändiga komponenter. | [a\_parts_list.md](a_komponentlista.md) |
| **Kretsschema (PDF)** | Detaljerat *hardware* kretsschema. | [a\_schematic.pdf](../model_a/docs/a_schematic.pdf) |
| **Bilder** | Foton på kretskort (fram/bak) och komponenter. | [images/](../model_a/images/) |

### Modell B filer (`/model_b/`)

Följande filer är unika för modellen med **LCD-display**:

| Dokument/fil | Beskrivning av innehåll | Sökväg (klicka för att se) |
| :--- | :--- | :--- |
| **Firmware (.ino)** | Arduino källkod för mikrokontrollern. | [model\_b.ino](../model_b/model_b.ino) |
| **Monteringsanvisning** | Steg-för-steg guide för lödning och slutmontering. | [b\_assembly.md](b_montering.md) |
| **Specifikation** | Detaljerad funktionslista och tekniska specifikationer för Modell B. | [b\_spec.md](b_beskrivning.md) |
| **Komponentlista (BOM)** | Fullständig lista över alla nödvändiga komponenter. | [b\_parts_list.md](b_komponentlista.md) |
| **Kretsschema (PDF)** | Detaljerat *hardware* kretsschema. | [b\_schematic.pdf](../model_b/docs/b_schematic.pdf) |
| **Bilder** | Foton på kretskort (fram/bak) och komponenter. | [images/](../model_b/images/) |

### Allmänna Projektfiler (Rotkatalog)

| Fil | Beskrivning av innehåll | Sökväg (klicka för att se) |
| :--- | :--- | :--- |
| **Beställningsguide** | Information om priser och hur du kan köpa din byggsats. | [beställning.md](beställning.md) |
| **English Documentation** | Projektdokumentation på engelska. | [README.md](../README.md) |
| **Licens** | Information om licens. | [LICENSE.md](../LICENSE.md) |

---

**Intresserad av att köpa en byggsats?** Läs mer i [**Beställningsguiden**](beställning.md)!

---

> **En anmärkning om garanti:** Detta är ett hobbyprojekt. Även om jag strävar efter hög kvalitet, lämnar jag ingen formell garanti. Vänligen kontakta mig direkt om du stöter på problem med enheten eller monteringsprocessen.

/Oskar Fornander
