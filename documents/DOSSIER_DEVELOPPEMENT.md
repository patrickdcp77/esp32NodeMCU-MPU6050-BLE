# Dossier de Developpement - ESP32 + MPU6050 + BLE

Ce document sert de base projet pour reprendre rapidement, comprendre les choix techniques, et evoluer sans repartir de zero.

## 1. Objectif Projet

Mesurer des mouvements rapides (jusqu'a des impacts type chute de judo) avec MPU6050 sur ESP32, puis transmettre les donnees via BLE vers une application Android (nRF Connect pour test, app dediee ensuite).

## 2. Etat Actuel

Le projet contient plusieurs variantes de firmware.

### Fichiers principaux

- `src/esp32-BLE-MPU5-DHT.cpp`
  - Variante courante pour integration Android.
  - Donnees envoyees: ID hardware + acceleration dynamique + temperature DHT22.
  - Trame compacte: `id,mg,temp`.
  - `id`: numero systeme (1..10), `mg`: acceleration dynamique en milli-g, `temp`: temperature absolue en degC.
  - DHT22 lue toutes les 5s (derniere valeur valide reutilisee entre 2 lectures).

- `esp32-BLE-MPU4-identiteHardware.cpp`
  - Variante actuelle pour deploiement multi-capteurs sans reflash par unite.
  - ID materiel lu via GPIO en `INPUT_PULLUP` (straps vers GND).
  - Trame BLE explicite: `id=<n>;acc_mg=<value>`.
  - Acceleration dynamique envoyee: `abs(|a| - 1g)` en milli-g (objet immobile proche de 0).
  - `notify()` envoye uniquement si le client est abonne (CCCD actif).

- `esp32-BLE-MPU3-EnvoiSommeAccelero.cpp`
  - Version active recommandee pour flux simple et rapide.
  - Envoie uniquement la norme accel (`|a|`) en milli-g via BLE Notify.
  - Cadence d'echantillonnage elevee (500 Hz en acquisition locale).
  - Envoi BLE compact (payload texte court, ex: `981`).
  - `DEBUG` compile-time (`0/1`) et notify conditionne a l'abonnement client.

- `esp32-BLE-MPU2-ImpactJudo.cpp`
  - Variante avec logique impact cote ESP32.
  - Detection d'evenement par seuil et fenetre.
  - Utile pour prototypage, moins ideal si la decision impact doit etre personnalisee par utilisateur dans Android.

- `esp32-BLE-envoiSimpleSalut.cpp`
  - Version BLE minimale de reference (validation liaison BLE/nRF Connect).

- `esp32-BLE-MPU1.cpp`
  - Variante MPU de base (historique de reference).

- `platformio.ini`
  - Environnement principal: `esp32_classic`.
  - `default_envs = esp32_classic`.
  - `CORE_DEBUG_LEVEL=0` pour limiter les logs framework parasites.

## 3. Architecture Technique

### Materiel

- ESP32 NodeMCU-32S
- MPU6050 (I2C)
- Cablage standard:
  - SDA -> GPIO21
  - SCL -> GPIO22
  - VCC -> 3V3 (recommande)
  - GND -> GND

### BLE (profil UART-like)

- Service UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- TX (ESP32 -> Android, notify): `6e400003-b5a3-f393-e0a9-e50e24dcca9e`
- RX (Android -> ESP32, write): `6e400002-b5a3-f393-e0a9-e50e24dcca9e`

### Acquisition capteur

- Echelle accel: `+/-16g`
- Conversion accel en g: `raw / 2048.0`
- Norme accel: `sqrt(ax^2 + ay^2 + az^2)`
- Acceleration dynamique (version identite hardware): `abs(norme - 1.0)`

## 4. Decisions Importantes Deja Prises

1. Eviter les grosses trames JSON quand on vise des evenements rapides.
2. Preferer payload compact et explicite pour un parsing Android robuste.
3. Cote perf:
- reduire les `Serial.print`
- utiliser `DEBUG=0` en exploitation
4. N'envoyer `notify()` que si le client a active les notifications (CCCD), pour eviter les erreurs `esp_ble_gatts_send_notify rc=-1`.
5. Encoder l'identite en hardware (GPIO) pour eviter le reflash unitaire.
6. Envoyer l'acceleration dynamique (`|a|-1g`) pour obtenir une base proche de 0 a l'immobile.
7. Pour la calibration long terme, prioriser la logique dans l'app Android (plus flexible, pas besoin de reflasher ESP32).

## 5. Procedure Standard (Build / Flash / Test)

## 5.1 Build

- Commande: `platformio run`
- VS Code: Task `Build`

## 5.2 Upload

- Commande: `platformio run --target upload`
- VS Code: Task `Upload`

## 5.3 Test BLE dans nRF Connect

1. Scanner et connecter le device ESP32.
2. Ouvrir le service `6e400001-...`.
3. Activer `Notify` sur la caracteristique TX `6e400003-...`.
4. Verifier reception des valeurs.
5. Si besoin, demander un MTU plus grand dans menu `...` -> `Request MTU` (ex: 247).

## 6. Problemes Rencontres et Solutions

### 6.1 Erreur notify `rc=-1 Unknown ESP_ERR`

Cause frequente:
- `notify()` appele alors que le client n'est pas abonne notifications.

Solution appliquee:
- Verification de l'etat CCCD (`BLE2902::getNotifications()`) avant `notify()`.

### 6.2 Trames tronquees dans nRF Connect

Cause:
- payload trop long pour MTU/echange BLE courant.

Solution:
- payload plus court (format compact)
- eventuellement augmenter MTU dans l'app.

### 6.3 Bruit terminal qui degrade la perf

Cause:
- sorties serie trop frequentes.

Solution:
- `DEBUG=0`
- desactiver logs framework (`CORE_DEBUG_LEVEL=0`).

## 7. Backlog D'amelioration (Priorise)

### P1 - Robustesse flux BLE

- Ajouter compteur interne:
  - notif envoyees
  - notif sautees (non abonne)
  - erreurs lecture I2C
- Exposer ces stats periodiquement en mode debug.

### P1 - Calibration Android

- Envoi ESP32 des donnees necessaires (minimum `accMag`, idealement `ax,ay,az` + timestamp).
- Cote app:
  - phase repos (baseline)
  - phase impacts tests
  - calcul seuil personnalise
  - cooldown anti-double-detection

### P2 - Format binaire haut debit

- Passer de texte (`"981"`) a 2 octets binaires (uint16 milli-g)
- Gain de debit et latence BLE.

### P2 - Horodatage monotone

- Ajouter `t_ms` dans la trame (ou sequence number) pour reconstituer exact timing cote app.

### P3 - Stabilite capteur

- Option filtrage numerique simple (EMA) si necessaire.
- Option calibration offset gyro/accel au demarrage.

## 8. Conventions Projet

1. Garder une variante principale stable dans `src/`.
2. Garder les variantes experimentales a la racine avec nom explicite.
3. Toute nouvelle variante doit documenter:
- objectif
- format de trame
- cadence
- decisions de debug

## 8.1 Identite hardware (10 systemes)

Pour identifier les systemes sans reflasher, on utilise des straps resistifs vers GND lus par l'ESP32.

Important:
- 3 GPIO donnent 8 combinaisons (2^3), insuffisant pour 10 IDs.
- La version `esp32-BLE-MPU-identiteHardware.cpp` utilise donc 4 GPIO.

GPIO retenus:
- BIT0: GPIO25
- BIT1: GPIO26
- BIT2: GPIO32
- BIT3: GPIO33

Regle de lecture:
- pull-up interne ESP32 uniquement (pas de pull-up externe 3.3V requis)
- pin ouverte (INPUT_PULLUP) = bit 0
- pin reliee directement a GND (jumper/interrupteur) = bit 1

Table de selection (ID 1 a 10):
- ID1  -> 0000 : aucun strap
- ID2  -> 0001 : GPIO25
- ID3  -> 0010 : GPIO26
- ID4  -> 0011 : GPIO26 + GPIO25
- ID5  -> 0100 : GPIO32
- ID6  -> 0101 : GPIO32 + GPIO25
- ID7  -> 0110 : GPIO32 + GPIO26
- ID8  -> 0111 : GPIO32 + GPIO26 + GPIO25
- ID9  -> 1000 : GPIO33
- ID10 -> 1001 : GPIO33 + GPIO25

Format BLE associe (version actuelle DHT):
- trame compacte: `id,mg,temp`
- exemple: `3,981,24.7`
- ordre des champs fixe pour parser Android: `id` puis `mg` puis `temp`

## 8.2 Validation recemment confirmee

- Identification hardware validee sur IDs `1`, `2`, `3`.
- Connexion BLE simultanee validee avec 2 capteurs (`ID 1` et `ID 3`).

## 8.3 Procedure de recette 10 capteurs (checklist)

Pre-requis:
- Firmware: `src/esp32-BLE-MPU5-DHT.cpp`
- Application de test: nRF Connect
- Notify active sur TX `6e400003-...`

Controles a faire pour chaque capteur:
1. Le nom BLE annonce l'ID attendu (ex: `ESP32-BLE-MPU-SUM-03`).
2. La trame recue est au format `id,mg,temp`.
3. Le champ `id=<n>` correspond bien au cablage GPIO du capteur teste.
4. A l'immobile, `mg` reste proche de 0 (petit bruit accepte).
5. En mouvement, `mg` augmente puis redescend.
6. La temperature `temp` est presente et evolue lentement (mise a jour toutes les 5 s).
7. Connexion stable pendant 2 minutes sans deconnexion.

Checklist par ID:
- [ ] ID1 valide
- [ ] ID2 valide
- [ ] ID3 valide
- [ ] ID4 valide
- [ ] ID5 valide
- [ ] ID6 valide
- [ ] ID7 valide
- [ ] ID8 valide
- [ ] ID9 valide
- [ ] ID10 valide

Recette multi-capteurs:
- [ ] Test simultane ID1 + ID3 (deja valide)
- [ ] Test simultane ID2 + ID4
- [ ] Test simultane ID5 + ID6
- [ ] Test simultane ID9 + ID10
- [ ] Test 3 capteurs simultanes (au choix) pendant 5 minutes

## 9. Reprise Rapide (Checklist)

1. Ouvrir le projet dans VS Code + PlatformIO.
2. Verifier fichier actif principal:
- `src/esp32-BLE-MPU5-DHT.cpp`
3. Build, puis Upload.
4. Dans nRF Connect, activer `Notify` sur TX.
5. Confirmer reception stable des valeurs.
6. Verifier format trame Android: `id,mg,temp`.
7. Si objectif impact final: basculer logique decision dans l'app Android.

## 10. Note Strategique

Pour un produit evolutif, l'ESP32 doit surtout etre un "acquisiteur-transmetteur" rapide et fiable, et l'application Android doit porter la calibration metier et la decision impact.
