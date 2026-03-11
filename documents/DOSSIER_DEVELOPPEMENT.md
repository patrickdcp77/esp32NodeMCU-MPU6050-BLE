# Dossier de Developpement - ESP32 + MPU6050 + BLE

Ce document sert de base projet pour reprendre rapidement, comprendre les choix techniques, et evoluer sans repartir de zero.

## 1. Objectif Projet

Mesurer des mouvements rapides (jusqu'a des impacts type chute de judo) avec MPU6050 sur ESP32, puis transmettre les donnees via BLE vers une application Android (nRF Connect pour test, app dediee ensuite).

## 2. Etat Actuel

Le projet contient plusieurs variantes de firmware.

### Fichiers principaux

- `src/esp32-BLE-MPU-EnvoiSommeAccelero.cpp`
  - Version active recommandee pour flux simple et rapide.
  - Envoie uniquement la norme accel (`|a|`) en milli-g via BLE Notify.
  - Cadence d'echantillonnage elevee (500 Hz en acquisition locale).
  - Envoi BLE compact (payload texte court, ex: `981`).
  - `DEBUG` compile-time (`0/1`) et notify conditionne a l'abonnement client.

- `esp32-BLE-MPU-ImpactJudo.cpp`
  - Variante avec logique impact cote ESP32.
  - Detection d'evenement par seuil et fenetre.
  - Utile pour prototypage, moins ideal si la decision impact doit etre personnalisee par utilisateur dans Android.

- `esp32-BLE-envoiSimpleSalut.cpp`
  - Version BLE minimale de reference (validation liaison BLE/nRF Connect).

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

## 4. Decisions Importantes Deja Prises

1. Eviter les grosses trames JSON quand on vise des evenements rapides.
2. Preferer payload compact pour limiter la saturation BLE.
3. Cote perf:
- reduire les `Serial.print`
- utiliser `DEBUG=0` en exploitation
4. N'envoyer `notify()` que si le client a active les notifications (CCCD), pour eviter les erreurs `esp_ble_gatts_send_notify rc=-1`.
5. Pour la calibration long terme, prioriser la logique dans l'app Android (plus flexible, pas besoin de reflasher ESP32).

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

## 9. Reprise Rapide (Checklist)

1. Ouvrir le projet dans VS Code + PlatformIO.
2. Verifier fichier actif principal:
- `src/esp32-BLE-MPU-EnvoiSommeAccelero.cpp`
3. Build, puis Upload.
4. Dans nRF Connect, activer `Notify` sur TX.
5. Confirmer reception stable des valeurs.
6. Si objectif impact final: basculer logique decision dans l'app Android.

## 10. Note Strategique

Pour un produit evolutif, l'ESP32 doit surtout etre un "acquisiteur-transmetteur" rapide et fiable, et l'application Android doit porter la calibration metier et la decision impact.
