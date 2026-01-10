# Vision-System: Design-Entscheidungen & Technische Herleitung

<!-- Written, maintained and owned by Linus Braun (MURI DEVELOPMENT TEAM) -->

## Übersicht

Diese Dokumentation erläutert die Begründungen für technische Design-Entscheidungen in der Vision-Pipeline. Für Implementierungs-Details siehe [vision_implementation.md](vision_implementation.md).

---

## Architektur-Entscheidungen

### 1. Zwei separate ROS2-Nodes + ausgelagerte OpenCV-Logik

**Entscheidung**: CameraReadOut (Bilderfassung) und ImageProcessing (Verarbeitung) als getrennte ROS2-Nodes, mit OpenCV-Logik in separate AMD-Klasse ausgelagert

**Begründung**:
- **Skalierbarkeit**: Verarbeitungsteil kann auf anderem Rechner ausgeführt werden
- **Parallelisierung**: Bilderfassung und Verarbeitung laufen asynchron
- **Code-Trennung**: OpenCV-Logik (AMD) ist unabhängig von ROS2, wiederverwendbar in anderen Kontexten
- **Wartbarkeit**: Getrennte Zuständigkeiten (Single Responsibility Principle)
- **Fehlertoleranz**: Ausfall eines Nodes beeinflusst den anderen nicht direkt


---

### 2. Grayscale-Konvertierung in CameraReadOut

**Entscheidung**: RGB → Grayscale im CameraReadOut-Node, nicht im ImageProcessing

**Begründung**:

| Aspekt | Benefit |
|---|---|
| **Bandbreite** | 66% weniger Daten bei Übertragung |
| **Bildrate** | Höherer Durchsatz von Bildern möglich (30 Hz vs. mögliche Drosselung bei RGB) |
| **Speicher** | Weniger RAM-Nutzung |
| **ArUco-Detektion** | Funktioniert gleichwertig mit Grayscale (Kontraste reichen aus) |

---

### 3. Timer-Frequenz 30 Hz in CameraReadOut

**Entscheidung**: Bilderfassung mit maximaler Kamera-Frequenz (30 Hz)

**Begründung**:
- **Maximale Aktualität**: Neuste verfügbare Bilder für Detektion
- **Kamera-Spezifikation**: Roboter-Kamera liefert max. 30 fps
- **Echtzeit-Anforderung**: Roboter-Steuerung benötigt niedrige Latenz
- **Pufferoptimierung**: Buffer-Size=1 verhindert veraltete Frames

---

### 3.1 CAP_PROP_BUFFERSIZE = 1 in CameraReadOut

**Entscheidung**: OpenCV-Kamera-Buffer auf Größe 1 setzen

**Begründung**:
- **Minimale Latenz**: Nur der aktuellste Frame wird gepuffert, keine veralteten Frames
- **Echtzeitnavigation**: Roboter-Regler erhält stets die neueste Sensorposition
- **Konsistente Steuerung**: Verhindert Instabilität durch verzögerte Sensordaten

**Problematik ohne diese Einstellung**:
- OpenCV speichert standardmäßig mehrere Frames intern
- Veraltete Frames werden mit Verzögerung ausgegeben
- Roboter-Regler reagiert auf alte Positionen, nicht auf aktuelle
- P-Regler wird instabil, Navigation unpräzise

**Implementierung**:
```python
cap = cv.VideoCapture(device)
cap.set(cv.CAP_PROP_BUFFERSIZE, 1)  # Kritisch für Echtzeit!
```

---

## Marker-Erkennungs-Entscheidungen

### 4. ArUco-Dictionary DICT_5X5_1000

**Entscheidung**: 5×5-Marker mit bis zu 1000 verschiedenen IDs

**Vergleich mit Alternativen**:

| Dictionary | Marker-Größe | ID-Bereich | Erkennungs-Qualität |
|---|---|---|---|
| **4X4_50** | 4×4 Bits | 0-49 | Weniger robust bei Distanz/Bewegungsunschärfe |
| **5X5_1000** | 5×5 Bits | 0-999 | **Optimal** ✓ |
| **6X6_250** | 6×6 Bits | 0-249 | Bessere Reichweite, aber weniger IDs verfügbar |

**Begründung für 5X5_1000**:
- ✓ Bessere Erkennbarkeit aus Distanz/niedriger Auflösung als 4X4
- ✓ Ausreichend viele IDs für Mehrroboter-Szenarien (nicht begrenzt auf 2-3 Roboter)
- ✓ Schnellere Verarbeitung als 6X6
- ✓ Balance zwischen Robustheit und Vielfalt

---

### 5. Marker-Prioritäts-Logik (ID 69 > ID 0)

**Entscheidung**: Wenn beide Marker 0 und 69 erkannt → Marker 69 bevorzugen

**Begründung**:

- **Performance**: Priorität vor solvePnP entscheiden (spart unnötige Pose-Berechnungen)
- **Szenario**: Roboter kann mehrere Marker sehen, muss aber dem anderen Roboter (69) Priorität geben
- **Flexibilität**: Prioritäts-Reihenfolge lässt sich durch Änderung des Prioritäts-Index leicht anpassen und erweitern

**Implementierungs-Pseudocode**:
```python
if Marker 69 erkannt:
    Verwende Marker 69
else if Marker 0 erkannt:
    Verwende Marker 0
else:
    Fehlerfall
```
---


---

## Distanz- und Winkelberechnungen

### 6. solvePnP mit SOLVEPNP_IPPE_SQUARE

**Entscheidung**: Pose-Estimation via `cv.solvePnP(obj_points, img_points, ...)`

**Begründung**:

| Aspekt | Benefit |
|---|---|
| **Genauigkeit** | Nutzt vorkalibrierte Kameraparameter |
| **Robustheit** | Funktioniert mit 4 Bildpunkten pro Marker |
| **Etabliert** | OpenCV-Standard für Marker-Tracking |


**Warum IPPE_SQUARE**:
- ArUco-Marker sind quadratisch und planar
- Spezialisiert auf diesen Anwendungsfall
- Höhere Präzision und Geschwindigkeity

**Output**: Translationsvektor [x, y, z]
```
tvec[0] = X-Offset (horizontal, mm)
tvec[1] = Y-Offset (vertikal, mm)
tvec[2] = Z-Distanz (Tiefe, mm)
```

---

### 7. Manuelle Winkelberechnung statt rvec

**Entscheidung**: Winkel via `atan2(tvec[0], tvec[2])` statt aus Rotations-Matrix (rvec)

**Problem mit rvec**:
```
solvePnP gibt auch rvec (Rotationsvektor) zurück, aber:
- Sprunghafte Werte bei Grenzfällen
    --> zu instabil für P-Regler
```

**Lösung: Manuelle Berechnung aus tvec**

```python
angle = atan2(x_offset, distance)
```

**Vorteile**:
- ✓ Stabile, vorhersagbare Werte
- ✓ Direkt für P-Regler nutzbar
- ✓ Mathematisch einfach

**Mathematik**:
```
         Kamera
          *
         /|
        / | z_distance (tvec[2])
  x_off/  |
      /   |
     -----+
    Marker
    
tan(angle) = x_offset / z_distance
angle = atan2(x_offset, z_distance)
```

**Bereich**: [-π, π] (alle 4 Quadranten)

---

### 8. Last-Valid-Value-Filter (3-Wert-Buffer)

**Entscheidung**: Distanz-Filterung mit 3-Wert-Schiebe-Buffer

**Problem ohne Filter**:
- Lichtwechsel: Temporäre Erkennungsfehler
- Marker-Verdeckung: -1000.0 (Fehler-Wert) springt durcheinander

**Lösung: Last-Valid-Value-Filter**

**Logik**:
1. Schiebe Werte im Buffer
2. Gib neuesten gültigen (≠ -1.0) Wert
3. Falls alle ungültig → -1.0


---

## Fehlerbehandlung-Entscheidungen

### 9. Fehlercodes (-1000.0, π, 9999)

**Entscheidung**: Spezifische Fehlerwerte statt Exception werfen, welche extern erkenntlich sind

**Fehlerwerte**:

| Komponente | Wert | Begründung |
|---|---|---|
| **Distanz** | -1000.0 | Negative Distanz ist physikalisch unmöglich |
| **Winkel** | π (180°) | Roboter kann bei 180° kein Marker sehen |
| **Marker-ID** | 9999 | Nicht im DICT_5X5_1000 (0-999), eindeutiger Fehler |

**Warum keine Exceptions**:
- ✓ ROS2-Nodes sollten robust sein (keine Crashes)
- ✓ Logik-Module können Fehlercode erkennen und behandeln

---

### 10. Error-Counter & Schwellwert > 10

**Entscheidung**: After 10 consecutive failed frames → error=True

**Logik**:
- Unterscheidet zwischen **Kurzzeitfehler** (Lichtwechsel) und **Permanentfehlern** (Kamera-Ausfall)
- Nach 10 Fehlern: error=True wird publiziert
- Logik-Module können dann in sicheren Zustand fahren
---

## Kalibrierungs-Entscheidungen

### 11. CharUco statt Chess-Board für Kalibrierung

**Problem mit Chess-Board-Kalibrierung**:
- Benötigt sehr saubere, scharfe Bilder
- Empfindlich gegen Verzerrungen und Beleuchtung
- Hohe Bildanzahl notwendig

**Lösung: CharUco-Kalibrierung**


**Vorteile CharUco**:
- ✓ Funktioniert mit weniger Bildern
- ✓ Robuster gegen Verzerrungen
- ✓ Bessere Ecken-Erkennung (vordefinierte Marker-Ecken)

---

## Lessons Learned - Technische Erkenntnisse

### 12. Custom Marker sind zu aufwendig - OpenCV ArUco ist deutlich einfacher

**Idee**: Eigene Marker zur Position- und Distanz-Verfolgung zu erstellen

**Problem**: 
- Design des Markers notwendig
- Druck und physische Fertigung aufwendig
- Test und Kalibrierung zeitintensiv
- Insgesamt sehr zeitaufwendig und fehleranfällig

**Lösung**: OpenCV ArUco verwenden
- ✓ Sofort einsatzbereit (vordefinierte Marker)
- ✓ Einfache Implementierung
- ✓ Dokumentation und Funktionen existieren
- ✓ Zuverlässig getestet

**Entscheidung**: Standard ArUco-Marker für Position und Distanz-Tracking verwenden

---

### 13. Kreis-Erkennung funktioniert nicht in der Röhre

**Versuch**: Kreiserkennung zum Tracken des Wegs durch die Röhre

**Problem**:
- Das Rohr ist nicht kreisrund - ein Brett liegt darin
- Durch das Brett entsteht nur ein Halbkreis im Bild
- Der Halbkreis ist nicht konstant erkennbar (je nach Position des Roboters unterschiedlich)
- Macht das Tracking unmöglich und unzuverlässig

**Entscheidung**: ArUco-Marker sind deutlich besser für zuverlässiges Tracking geeignet 

---

### 14. solvePnP rvec-Werte sind zu sprunghaft für den Roboter

**Beobachtung**: Bei anfänglichen Tests mit solvePnP springt der ausgegebene Winkel extrem
- Winkel springt ruckartig von links nach rechts
- Keine stabilen Werte für die Roboter-Steuerung
- P-Regler kann damit nicht arbeiten

**Ursache**: Der Rotationsvektor (rvec) hat bei der mathematischen Konvergenz mehrere Lösungen

**Lösung**: Winkel manuell aus dem Translationsvektor (tvec) berechnen
- Verwendet nur die X- und Z-Komponenten: `angle = atan2(tvec[0], tvec[2])`
- Mathematisch stabil und vorhersagbar

---

### 15. Kalibrierungs-Qualität & Bildmittelpunkt-Präzision

**Problem**: Standardkalbrierungen (Chess-Board) lieferten falsche Bildmittelpunkt-Werte
- Optischer Mittelpunkt (cx, cy) in der Kamera-Matrix war systematisch daneben
- Folge: Roboter steuerte mit konstant falschen Distanz/Winkel-Werten
- Dauerhafte Positions-Fehler ohne erkennbare Ursache

**Experiment**: Verschiedene Kalibrierungs-Methoden getestet
- Chess-Board: Fehlerhafter Mittelpunkt
- **CharUco-Marker**: Saubere, präzise Bildmittelpunkte erhalten

**Entscheidung**: CharUco-Kalibrierung mit mindestens 10-20 hochqualitativen Bildern
- Bessere Ecken-Erkennung führt zu genauem optischen Mittelpunkt
- Direkte Auswirkung auf Pose-Berechnung (solvePnP verwendet cx, cy)
- **Lesson**: Wenn der Bildmittelpunkt falsch ist, führt das zu hartnäckigen Positions-Fehlern, deren Ursache schwer zu finden ist

---

### 16. OpenCV Camera-Buffer-Größe (CAP_PROP_BUFFERSIZE)

**Problem**: OpenCV speichert standardmäßig mehrere Frames intern
- Zu große Buffer → veraltete Frames werden ausgegeben
- Verzögerung zwischen Erfassung und Verarbeitung
- Roboter reagiert auf alte Positionen statt aktuellen

**Experiment**: Verschiedene Buffer-Größen getestet
- **Buffer > 1**: Roboter lenkt manchmal zu weit zur Seite, obwohl er schon weiter gerade aus fahren könnte
  - Grund: Veraltete Frame mit alten Positionen wird ausgegeben
  - P-Regler bekommt verzögerte Sensordaten
  - Navigation wird instabil und inpräzise

- **Buffer = 1**: Nur aktuellster Frame wird gespeichert
  - Minimale Latenz
  - Aktuelle Position wird immer verwendet
  - Roboter reagiert auf echte aktuelle Daten

**Code in CameraReadOut**:
```python
cap = cv.VideoCapture(device)
cap.set(cv.CAP_PROP_BUFFERSIZE, 1)  # Kritisch für Echtzeit!
```

**Lesson**: OpenCV Camera-Buffer auf Größe 1 setzen um Echtzeit-Navigation zu ermöglichen. Größere Buffer führen zu Verzögerungen und Navigation-Problemen.

---

### 17. Raspberry Pi Performance & Auflösungs-Optimierung

**Problem**: Raspberry Pi zu schwach für Full-HD-Bilder (1920×1080)
- Kamera liefert theoretisch HD-Qualität
- Verarbeitung wird CPU-limitiert
- Bildrate kann nicht gehalten werden

**Experiment**: Verschiedene Auflösungen getestet


**Kritische Erkenntnisse**:
1. **Auflösungs-Reduktion**: 640×480 ist Standard-OpenCV-Größe
2. **RGB→Grayscale**: Skaliert Performanz erheblich (3 Kanäle → 1 Kanal)
3. **Timing der Konvertierung**: Je früher die Konvertierung erfolgt, desto besser
   - Konvertierung im CameraReadOut spart Bandbreite
   - ImageProcessing kann dann kleinere Frames zu verarbeiten

**Lesson**: Bildauswahl muss spezifisch zu dem verwendeten System angepasst werden
- Niedrige Auflösung wählen (640×480 statt HD)
- Grayscale früh im Pipeline konvertieren

---


