# DEC–HEX Converter: Byggsats Elektronik (Modell A & B)

Denna *repository* innehåller *firmware*, *hardware-design* och komplett dokumentation för **DEC–HEX Converter**, ett roligt och funktionellt byggsats-projekt. Syftet är att ge hobbylödare en chans att öva sina lödskunskaper samtidigt som de skapar ett användbart fysiskt verktyg för talbasomvandling.

**Snabblänkar:**
* [**Ready for the English documentation?**](README.md)
* **Vill du köpa?** Se vår [**Beställningsguide**](BUY.md)

## 💡 Gemensamma Huvudfunktioner

DEC–HEX Converter är byggd kring ett enkelt och taktilt gränssnitt:

* **Snabb Konvertering:** Omvandla snabbt mellan olika talbaser (Decimal, Hexadecimal, och Binär/Oktal beroende på modell).
* **Rotationsenkoder:** Använd den responsiva och exakta rotationsenkodern för att öka/minska värden. Den ger tydlig taktil *feedback* med fasta steg.
* **Intuitiv Styrning:** **Vrid** för att räkna uppåt/nedåt, och **Tryck** för att nollställa räkneverket.
* **Hjärnan:** Båda enheterna använder den pålitliga **mikrokontrollern ATtiny85/45** för all bearbetning.
* **Inbyggnadslåda:** Elektroniken monteras i en stabil **3D-printad inbyggnadslåda**.
* **Byggsatsen:** Levereras som ett komplett kit med ett **specialdesignat kretskort (PCB)**, alla nödvändiga komponenter och en **förprogrammerad mikrokontroller** – utmärkt för att öva **lödning**.
* **Anpassning:** Mikrokontrollern är förprogrammerad men kan enkelt **programmeras om**. Källkoden finns tillgänglig i denna *repository*.

## 🧰 De Två Modellerna

DEC–HEX Converter finns i två varianter, anpassade efter olika önskemål kring displaytyp och lödkomplexitet:

### Modell A: Den Klassiska Displayen
* Visar värden med ljusa, klassiska **sjusegmentdisplayer**.
* Konverterar mellan **Decimal** och **Hexadecimal**.
* Har en **dedikerad strömbrytare** och ett strömsparläge.
* *Innebär något mer lödning (använder skiftregister för att driva displayerna).*

### Modell B: Multi-Bas Displayen
* Visar värden på en **LCD (Liquid Crystal Display)**.
* Konverterar mellan **Decimal**, **Hexadecimal** och **Binär**.
* **Ingen strömbrytare**; använder ett *ultra-low-power* viloläge och vaknar omedelbart när vredet vrids eller trycks in.
* *Innebär något mindre lödning.*

## 📂 Innehåll i *Repositoryn*

Denna GitHub *repository* tillhandahåller allt du behöver för att bygga, modifiera och förstå DEC–HEX Converter:

| Katalog/Fil | Beskrivning av Innehåll |
| :--- | :--- |
| `/firmware` | Källkod och kompilerade filer för ATtiny85/45 mikrokontrollern. |
| `/hardware` | PCB designfiler (Scheman, Layouter) och komponentlista (BOM). |
| `/enclosure` | 3D-modeller (STL-filer) för de printade inbyggnadslådorna för både Modell A och B. |
| `/docs` | Detaljerade monteringsanvisningar och användarguider. |
| **`BUY.md`** | Information om priser och hur du beställer din byggsats. |
| **`README.md`** | Projektintroduktion och dokumentation på engelska. |

---

**Intresserad av att köpa en byggsats?** Läs mer i vår [**Beställningsguide**](BUY.md)!

---

> **En anmärkning om garanti:** Detta är ett hobbyprojekt. Även om jag strävar efter hög kvalitet, lämnar jag ingen formell garanti. Vänligen kontakta mig direkt om du stöter på problem med enheten eller monteringsprocessen.

/Oskar Fornander
