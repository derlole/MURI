Design Entscheidungen technische Herleitung

	Allgemein:

		zwei getrennte Nodes zum Auslesen und Verarbeiten/publishen,
			---> Ermöglicht es den Berechnungs teil extern auf anderem Gerät zu machen


	camera_read_out:

		in camera_read_out konvertiert das BIld vorab in Graustufen zur DAtenminderung

		Publishen mit der maximalen Wiedergabegeschwindkeit der KAmera ---> maximal verwendbare Bilde zu kriegen

		Bilder werden von RGB zu Grayscale konvertiert um die zu übertragende DAtenmenge zu mindern

	image_data_processing:

		Der Filter dient zur Hausfüllterung einzelner Fehler, die bei der Direktion bei nicht erkenntnisbildes oder ähnliches entstehen könnten. es wählt dauerhaft den neuesten Wert aus
		er such hierbei aus 3 Werten raus


	aruco_marker_detection

	Marker-Prioritätslogik: 
		vor DIstanzberechnugn um unnötige Berechnungen zu vermeiden
		unterscheidung 
		Bei der Marker Prioritätslohe ich soll hierbei ein Marker immer dauerhaft durchgereicht werden, wenn wir nächstes hier nicht der andere. Die sollen immer die ermöglichen bieten, dass der Robert, dass sobald ein anderen Robert erkennt, diesen verwendet, auf diesen zufährt und nicht weiter durchs Golf fährt.
		

	Fehlerbehandlung: Warum gerade diese Werte (-1000.0, math.pi, 9999)?
		-1000 ist kein Distanz erreichbarer WErt 
		math.pi Winkel kann physikalisch nie von der Kamera ausgelesen werden
		9999 ist keine verfügbare Marker ID
	

	Distanz- /Winkelberechnung
		SolvePnP zur Distanzberechnung, da es genaue Werte
		
		händische Winkelberechnung
			für genauere Werte
			Mit der händischen Berechnung der Winkel haben, bekommen wir auf Akkuarte und genauerer Werte als mit 12PMP. Hierbei können wir jetzt zu meinen besseren Winkel bestimmen.
	

lessons learned:	
	
	eigene MArker erstellung ist möglich, jedoch sehr aufwendig 
	eine Kreiserkennung benötigt möglich, jedoch mit einem annäherdem halb kreis, äußerst inkonstant 
		---> funktionen von opencv schon sehr effizient
	
	
	Kalibrierung mit chess board ist funktionell, jedoch muss sehr saubere Bilder bei der Kalibrierung gemacht werden
		---> charuco Kalibrierung funktioniert um einiges verlässlicher bei weniger Bildern
	
	
	solve PnP liefert sehr sprunghafte und ungenauer rvec WErte 
		---> händische Berechnungaus aus tvec, da die Werte sehr kosntant sind
		
	
	BIlddatengröße
		- sollte per GRaustufen gesendet werden um die DAtenmenge beim verschicken zu verkleinern und einen höheren Bilderdurchsatz zuerreichen
		- BIldauflösung auch so gering halten wie möglich um daten zu mindern		
	
		
	saubere Kalibrierung
	
	

	

	
