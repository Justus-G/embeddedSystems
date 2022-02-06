# A low-cost energy-efficient Real-Time-Analyzer based on an ESP32
- exact model: TTGO ESP32-LoRa
- used microphone: SPH0645


## Issues:
#### TODO:
    -
#### DONE:
    - CSV-Output automatisch an/aus schalten, wenn SD-Karte eingelegt bzw. nicht eingelegt ist
    - Normalisierung des RTA über gespeicherten durchschnitts-max-value
        - max-value nutzen, evtl. Anzahl der gespeicherten max-Values noch erhöhen (20 -> 200 ?)
    - RTA-multiplier anpassen

#### WON'T DO:
    - eventuell Durchschnitt statt Maximum der einzelnen FFT-Bins für RTA-bin-Auswertung nehmen 
        (Maximum ist schon gut so)
    - eventuell RTA-Frequenz-Bins anpassen:
        - weniger bins < 200Hz
        - mehr bins 250Hz - 2k Hz
        - weniger > 10k Hz
        (nicht mehr unbedingt notwendig seit angepassten RTA-Multiplier, entspricht außerdem nicht log. Frequenzskala)
    - eventuell queue hinzufügen -> queue geht nicht, weil FFT nicht unterschiedlich viele Elemente erwarten kann
    --> Ringpuffer hinzufügen, feste Größe z.b. 4       (aktuell wird nur jedes 2. bis 3. gelesene sample analysiert)
        - i2s_read() pusht gelesene samples in puffer
        - analyze() popt alle verfügbaren (4) samples aus queue und berechnet FFT
        (geht nicht, Analyse dauert auch hier zu lange)

#### OBSOLET:
    - herausfinden, warum Display manchmal für ca 0,25s schwarz ist