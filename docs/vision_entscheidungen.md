# Vision-System: Design & Technische Entscheidungen

<!-- Written, maintained and owned by Linus Braun (MURI DEVELOPMENT TEAM) -->

## Systemarchitektur

### Zwei separate Nodes: CameraReadOut + ImageProcessing

**Decision**: Aufteilung in Bilderfassung und Verarbeitung

**Begründung**:
- Ermöglicht verteiltes Computing auf unterschiedlichen Geräten
- CameraReadOut kann auf Roboter-Hardware laufen
- ImageProcessing (mit AMD-Detektor) kann extern auf stärkerer Rechenkapazität laufen
- Flexibilität bei Hardware-Upgrades

**Konsequenzen**:
- Bandbreite-Optimierung erforderlich (Grayscale statt RGB)
- ROS2 Topic `/muri_image_raw` als Schnittstelle

---

## ArUco-Detektor (AMD)

### Warum ArUco-Marker?

**Alternativen evaluiert**:
- ✗ **Eigene Marker**: Sehr aufwendig zu erstellen
- ✗ **Kreis-Erkennung**: Extrem inkonstant (auch halbe Kreise problematisch)
- ✓ **ArUco**: OpenCV-Funktionen bereits hochoptimiert

---

### Warum DICT_5X5_1000?

**Decision**: 5×5-Marker Dictionary (nicht 6×6 oder andere)

**Begründung**:
- Optimaler Kompromiss zwischen:
  - Erkennungsrobustheit aus größerer Entfernung
  - Erkennbarkeit bei niedriger Auflösung
  - Genügend IDs für mehrere Roboter/Marker

**Alternativen**:
- 6×6-Marker: Zu klein bei Ferne/schlechter Auflösung
- Größere Dictionaries: Unköthese bessere Erkennungsrate

---

### Warum solvePnP mit IPPE_SQUARE?

**Decision**: Infinitesimal Plane-based Pose Estimation (IPPE) für quadratische Objekte

**Begründung**:
- ArUco-Marker sind perfekt quadratisch → IPPE optimal
- Höhere Genauigkeit und Robustheit als generische Algorithmen
- Stabilere tvec-Werte als andere Methoden

**Erkenntnis aus Tests**:
- Standard-solvePnP-Flags liefern sprung-hafte rvec-Werte → unpraktisch
- tvec-Werte hingegen konstant und zuverlässig

---

### Marker-Prioritätslogik: ID 69 > ID 0

**Decision**: Wenn beide Marker erkannt, priorisiere ID 69

**Begründung**:
- Ermöglicht Roboter, andere Roboter zu erkennen und zu verfol gen
- ID 69: Anderer Roboter (höchste Priorität)
- ID 0: Standard-Zielmarker (niedrigere Priorität)
- **Use Case**: Sobald Roboter seinen "Partner" erkennt, konzentriert sich darauf statt Ziel zu verfolgen

**Ablauf**:
1. Suche nach ID 69 und ID 0 im gleichen Frame
2. Falls beide existieren → verwende 69
3. Falls nur eine existiert → verwende die eine
4. Falls keine → Fehler

---

### Warum diese Fehlercodes?

**Decision**: Rückgabe (-1000.0, math.pi, 9999) bei Fehler

**Begründung**:

| Wert | Grund |
|---|---|
| **z_pos = -1000.0** | Negative Distanz ist physikalisch unmöglich → sofort erkennbar als Fehler |
| **y_rot = π** | Roboter kann bei 180° kein Marker sehen (Kamera zeigt weg) → unmöglich |
| **marker_id = 9999** | ID 9999 existiert nicht im DICT_5X5_1000 → Fehler-ID |

**Vorteil**: Steuerungslogik kann all drei Fehler-Erkennungszeichen prüfen

---

### Warum solvePnP vor Marker-Priorität?

**Decision**: Priorisierung NACH detectMarkers(), aber VOR solvePnP()

**Begründung**:
- **Ineffizient**: solvePnP für beide Marker durchführen und dann Ergebnis wählen
- **Effizient**: Marker-ID prüfen, dann nur solvePnP für Gewinner-Marker

**Einsparung**: ~40% Rechenzeit bei Dual-Marker-Szenarios

---

### Kamera-Kalibrierung

#### CharUco > Schachbrett

**Evaluiert**:
- ✗ **Schachbrett-Kalibrierung**: Funktional, aber erfordert sehr saubere Bilder
- ✓ **CharUco-Kalibrierung**: Verlässlicher mit weniger/schlechteren Bildern

**Erkenntnis**: CharUco-Boards kombinieren Schachbrett mit ArUco-Markern → robuster

#### Parameter in config.py

Einmal pro Kamera kalibrieren:
```
CAMERA_MATRIX_RAW: 3×3 Matrix (Brennweite, optischer Mittelpunkt)
DISTANCE_COEFFICIENT: Verzerrungskoeffizienten (radial + tangential)
```

Einmal speichern → permanent in der Konfiguration

---

## Image-Processing Node

### Grayscale-Konvertierung

**Decision**: RGB → Grayscale in CameraReadOut, nicht in ImageProcessing

**Begründung**:
- **Datenreduktion**: 66% weniger Bytes zu übertragen
- **Bandbreite**: Bei 30 Hz sind das merkliche Einsparungen
- **Durchsatz**: Mehr Bilder pro Sekunde möglich
- **AMD-Anforderung**: Benötigt ohnehin nur Grayscale

**Konsequenzen**:
- CameraReadOut komplexer, ImageProcessing einfacher
- Gesamtsystem effizienter

---

### 3-Wert Last-Valid-Value-Filter

**Decision**: Buffer aus 3 Distanzwerten

**Begründung**:
- **Stabilität**: Kurzzeitige Fehler werden überbrückt
- **Robustheit**: Bei schlechten Lichtverhältnissen oder temporär verdecktem Marker
- **Trägheit**: 3 Werte = minimal, aber effektiv (bei 30 Hz = ~100ms Puffer)

**Logik**:
```
first_data = neuster Wert
second_data = vorletzter Wert
third_data = drittletzter Wert

→ Gib neuesten gültigen (≠ -1.0) zurück, oder -1.0
```

**Alternative nicht gewählt**:
- ✗ Kalman-Filter: Zu komplex für diesen Use-Case
- ✗ Durchschnitt: Würde bei Fehler nicht reagieren
- ✓ Last-Valid: Robust + einfach

---

### Error-Counter mit Schwellwert 10

**Decision**: error = True nach >10 konsekutiven Frame-Verlusten

**Begründung**:
- **Balance**: 10 Frames bei 30 Hz ≈ 333 ms
- **Toleranz**: Kurzzeitige Probleme (Lichtwechsel) werden toleriert
- **Reaktion**: Permanente Fehler (Kamera-Ausfall) werden erkannt

**Alternativen**:
- ✗ 5 Frames: Zu empfindlich gegenüber Rauschen
- ✓ 10 Frames: Empirisch optimiert
- ✗ 20+ Frames: Zu träge bei echten Problemen

---

## Winkelberechnung

### Warum händisch mit atan2 statt solvePnP-rvec?

**Evaluierung**:
- solvePnP liefert: rvec (Rotationsvektor)
- Problem: rvec-Werte sind sprung-haft und unpräzise
- Lösung: Berechne Winkel manuell aus tvec

**Berechnung**:
```
distance = tvec[2][0]  (Z-Komponente, Tiefe)
x_offset = tvec[0][0]  (X-Komponente, horizontaler Versatz)

angle = atan2(x_offset, distance)
```

**Warum atan2?**
- Vollständiger Wertebereich: [-π, π]
- Keine Division-by-Zero-Probleme
- Automatische Vorzeichenbehandlung
- Höhere Numerische Genauigkeit

**Ergebnis**: Konstante, vorhersagbare Winkelwerte → P-Regler kann damit arbeiten

---

## Zusammenfassung: Optimierungsprinzipien

| Aspekt | Entscheidung | Nutzen |
|---|---|---|
| **Architektur** | 2 Nodes | Verteiltes Computing |
| **Marker-Typ** | ArUco DICT_5X5_1000 | Robust + hochoptimiert |
| **Pose-Estimation** | solvePnP IPPE_SQUARE | Hochgenau für Quadrate |
| **Marker-Priorität** | 69 > 0 | Robot-to-Robot Erkennung |
| **Fehler-Codes** | (-1000, π, 9999) | Eindeutig erkennbar |
| **Bildformat** | Grayscale | Bandbreite-Optimierung |
| **Filterung** | 3-Wert-Buffer | Robustheit bei Störungen |
| **Error-Detection** | Counter > 10 | Balance: Toleranz vs. Reaktion |
| **Winkelberechnung** | Manual mit atan2 | Stabilität über rvec |

