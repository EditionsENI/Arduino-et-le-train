# Arduino-et-le-train
Programmes du livre "Arduino : faites-le jouer au train" avec mises à jour

Pourquoi ne pas utiliser ces microcontrôleurs très populaires et peu coûteux pour automatiser intégralement un réseau de trains? Les cartes à microcontrôleurs de type Arduino offrent de nombreuses possibilités d'automatisation au sein d'un réseau de trains miniatures. Leur faible prix permettant de les utiliser en masse à tous les niveaux.

### dicicino-multi.ino
Version améliorée du programme "dicicino-uno-totale.ino" du chapitre 12.
Il intègre les fonctions suivantes : pilotage DCC, programmeur de CV, scanner I²C, testeur I²C, changeur d'adresse I²C, moniteur I²C, testeur PCA9685, Testeur/programmeur d'aiguillages à électro-aimants ou à servomoteurs.

### i2c-echo.ino
Enregistrement d'une série d'octets, puis renvoi de celle-ci. Ne sert que pour tester la fiabilité d'un bus I²C.
Version améliorée du programme "i2c-test-slave.ino" du chapitre 4.
Le programme "i2c-test-master.ino", qui dialogue avec, est dorénavant intégré en version améliorée au programme "dicicino-multi.ino".

### i2c-aiea.ino
Pilotage d'aiguillages commandés par électro-aimants.
Version améliorée du programme "aiguillage-electro.ino" du chapitre 10.

### i2c-aisv.ino
Pilotage d'aiguillages commandés par servomoteurs.
Version améliorée du programme "aiguillage-servo.ino" du chapitre 10.
