# Monteringsguide: DEC–HEX Converter, modell B

Denna guide beskriver de enkla steg som krävs för att montera din byggsats av modell B. Tack vare det specialdesignade kretskortet (PCB) är komponenternas placering tydligt utmärkt, vilket gör detta till ett okomplicerat bygge och ett utmärkt projekt för lödövning.

**Beräknad tid:** ca 1 timme (beroende på lödvana).

**Verktyg som krävs:** Lödkolv, lödtenn, sidavbitare (för att trimma ben), liten insexskruvmejsel, och hobbyklister (t.ex. limpistol eller superlim).

Se [**Komponentlista**](b_komponentlista.md) (namn och antal) för att korrekt identifiera varje komponent. Alla genomhålskomponenter sticks in från komponentsidan (sidan med  markeringar) och löds på motsatt sida (lödsidan). Alla komponenters placering är tydligt markerade på komponentsidan.

### 1. Löda alla komponenter, börja med lågprofilkomponenter

Börja med att löda komponenterna platt mot kretskortet, lämpligen från de lägsta till de högsta.

* **Motstånd:** Löda motstånden, följ markeringarna på kretskortet för deras värden (kontrollera komponentlistan för färgkoder om du är osäker).
* **Kondensatorer:** Löda de keramiska kondensatorerna enligt deras värden. Löda elektrolytkondensatorn och säkerställ att den placeras med korrekt orientering/polaritet enligt kretskortets markeringar. Denna kan vinklas ner mot kretskortet för att ta mindre plats på höjden.
* **Transistor och LDO-regulator**: Löda Transistor och LDO-regulatorn på respektive plats. Notera att de båda använder liknande kapslar; jämför beteckningarna på komponenterna för att skilja dem åt och säkerställ rätt orientering.
* **IC-sockel (DIP-8):** Löda sockeln för mikrokontrollern (ATtiny85/45). Var uppmärksam på orienteringsmarkeringen på sockeln; den måste matcha markeringen tryckt på kretskortet. 
* **Rotationsenkoder:** Placera och löda rotationsenkoderns fem stift samt dess fästflikar för att säkra den stadigt. Skruva fast den från framsidan och sätt på ratten.
* **Stiftlist och batterilåda:** Löda stiftlisten (4 stift) för anslutning av LCD:n. Löda batteriets kablar direkt eller på separat stiftlist. Notera anslutningens polaritet.

### 2. Sätt i mikrokontroller

* Sätt försiktigt i den **förprogrammerade ATtiny85/45 mikrokontrollern** i sin sockel. Skåran på IC-kretsens kropp måste matcha skåran på sockeln och trycket på kretskortet.

### 3. Förbered batterilådan

* Sätt i fyra AAA-batterier.
* Säkerställ att batterilådans strömbrytare är satt till läge **PÅ**. Enheten kräver att denna brytare är permanent på; när den är inaktiv går den in i viloläge som drar minimalt med ström.
* Applicera hobbyklister (eller dubbelhäftande tejp) på den fasta botten av **batterilådan** (sidan motsatt det öppningsbara locket) och limma fast den säkert på det avsedda området på kapslingens **bottenplatta**.

### 4. Montera kretskort och LCD

* Placera det färdigmonterade kretskortet i huvuddelen av den **3D-printade kapslingen**.
* Använd två av de medföljande **M3-skruvarna** och **distansbrickorna (2mm)** för att fästa kretskortet säkert i hålen på kapslingen. Distansbrickorna säkerställer korrekt skruvdjup och ger ett säkert fäste för kretskortet. Skruva fast rotationsenkodern från framsidan och sätt på ratten.
* Montera LCD:n med fyra av **M3-skruvarna** och **distansbrickorna (5mm)**.
* Anslut medföljande **kablar (4 stift)** till LCD och PCB. **Viktigt:** Säkerställ korrekt anslutning genom att jämföra tryckt text vid stiften; stiften är i samma ordning på pcb:n även om beteckningarna kan vara olika (VSS = GND, VDD = VCC, SCK = SCL). Felaktig polaritet kan skada komponenter.

### 5. Stäng kapslingen

* Placera bottenplattan (med den fastlimmade batterilådan) på huvudkapslingen.
* Säkra kapslingen med **M4-skruvarna** genom bottenplattan och in i huvudkapslingen.

**Grattis! Din DEC–HEX Converter, modell B, är nu färdigmonterad och redo att användas!**

---

/Oskar Fornander
