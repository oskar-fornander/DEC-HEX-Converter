# Monteringsguide: DEC–HEX Converter, modell A

Denna guide beskriver de enkla steg som krävs för att montera din byggsats av modell A. Tack vare det specialdesignade kretskortet (PCB) är komponenternas placering tydligt utmärkt, vilket gör detta till ett okomplicerat bygge och ett utmärkt projekt för lödövning.

**Beräknad tid:** 1–2 timmar (beroende på lödvana).

**Verktyg som krävs:** Lödkolv, lödtenn, sidavbitare (för att trimma ben), liten insexskruvmejsel, och hobbyklister (t.ex. limpistol eller superlim).

Det är avgörande att följa den rekommenderade lödordningen för att säkerställa enkel åtkomst till alla lödpunkter. Se [**Komponentlista**](a_komponentlista.md) (namn och antal) för att korrekt identifiera varje komponent. Alla genomhålskomponenter sticks in från komponentsidan (sidan med  markeringar) och löds på motsatt sida (lödsidan). Alla komponenters placering är tydligt markerade på komponentsidan.

### 1. Löda alla komponenter, börja med lågprofilkomponenter

Börja med att löda komponenterna platt mot kretskortet, lämpligen från de lägsta till de högsta.

* **Avkopplingskondensatorer (100nF, 6 st):** Dessa små keramiska kondensatorer ska lödas först. Detta är avgörande för att komma åt att löda dem då de delvis är placerade under IC-socklarna.
* **IC-socklar (fem DIP-16 och en DIP-8):** Löda socklarna för mikrokontrollern (ATtiny85/45) och skiftregistren (74HC595). Var uppmärksam på skåran (orienteringsmarkeringen) på sockeln; den måste matcha markeringen tryckt på kretskortet. Se till att avkopplingskondensatorerna är lödda före detta steg.
* **Motstånd:** Löda motstånden, följ markeringarna på kretskortet för deras värden (kontrollera komponentlistan för färgkoder om du är osäker).
* **Kondensatorer, diod och transistor:** Löda resten av de mindre komponenterna på plats enligt deras värden. Säkerställ att transistorn, dioden och elektrolytkondensatorn placeras med korrekt orientering/polaritet. Se kretskortets markeringar (t.ex. diodens rand, kondensatorns minusben).
* **Strömbrytare och batterilåda:** Löda kablarna för strömbrytaren och batterilådan, antingen direkt till kretskortet eller till stiftlister.
* **Sjusegmentsdisplayer:** Stick i och löda de fem 7-segmentsdisplayerna. Säkerställ att de är placerade platt mot kretskortet och i korrekt orientering.
* **Rotationsenkoder:** Placera och löda rotationsenkoderns fem stift samt dess fästflikar för att säkra den stadigt.

### 2. Sätt i mikrokontroller och integrerade kretsar

* Sätt försiktigt i den **förprogrammerade ATtiny85/45 mikrokontrollern** och alla **Skiftregister (74HC595)** i deras socklar. Skåran på IC-kretsens kropp måste matcha skåran på sockeln och trycket på kretskortet.

### 3. Förbered batterilådan

* Sätt i tre AAA-batterier.
* Säkerställ att batterilådans strömbrytare är satt till läge **PÅ**. Enheten kräver att denna brytare är permanent på, eftersom huvudströmbrytaren på kretskortet styr strömmen.
* Applicera hobbyklister (eller dubbelhäftande tejp) på den fasta botten av **batterilådan** (sidan motsatt det öppningsbara locket) och limma fast den säkert på det avsedda området på kapslingens **bottenplatta**.

### 4. Montera kretskortet

* Placera det färdigmonterade kretskortet i huvuddelen av den **3D-printade kapslingen**.
* Använd de medföljande **M3-skruvarna** och **distansbrickorna** för att fästa kretskortet säkert i hålen på kapslingen. Distansbrickorna säkerställer korrekt skruvdjup och ger ett säkert fäste för kretskortet. Skruva fast rotationsenkodern från framsidan och sätt på ratten.
* Montera **strömbrytaren** i sin öppning på kapslingen och anslut dess kablar till strömbrytaren.

### 5. Stäng kapslingen

* Placera bottenplattan (med den fastlimmade batterilådan) på huvudkapslingen.
* Säkra kapslingen med **M4-skruvarna** genom bottenplattan och in i huvudkapslingen.

**Grattis! Din DEC–HEX Converter, modell A, är nu färdigmonterad och redo att användas!**

---

/Oskar Fornander
