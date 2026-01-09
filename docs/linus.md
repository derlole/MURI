<!--  Written, maintained and owned by Linus Braunee (MURI DEVELOPMENT TEAM) -->

Design Entscheidungen technische Herleitung

	in camera_read_out konvertiert das BIld vorab in Graustufen zur DAtenminderung


	Marker-Prioritätslogik: 
		vor DIstanzberechnugn um unnötige Berechnungen zu vermeiden
		
	

	Fehlerbehandlung: Warum gerade diese Werte (-1000.0, math.pi, 9999)?
		-1000 ist kein Distanz erreichbarer WErt 
		math.pi Winkel kann physikalisch nie von der Kamera ausgelesen werden
		9999 ist keine verfügbare Marker ID
	

	Distanz- /Winkelberechnung
		SolvePnP zur Distanzberechnung, da es genaue Werte
		
		händische Winkelberechnung
			für genauere Werte
	

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
	
		
		
	
	

	

	
