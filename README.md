# esp32-mpu6050-ble (PlatformIO)

Exemple minimal BLE pour ESP32 classique :
- envoi d'un message texte simple vers Android (Notify)
- réception d'un message texte depuis Android (Write)

## UUID BLE utilisées

- Service : `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- TX (ESP32 -> Android, READ + NOTIFY) : `6e400003-b5a3-f393-e0a9-e50e24dcca9e`
- RX (Android -> ESP32, WRITE) : `6e400002-b5a3-f393-e0a9-e50e24dcca9e`

Nom BLE annoncé : `ESP32-BLE-MSG`

## Comportement du firmware

- Quand Android est connecté, l'ESP32 envoie toutes les 2 secondes :
  - `Bonjour Android #1`, `Bonjour Android #2`, etc.
- Si Android écrit un texte dans la caractéristique RX, le message est affiché sur le moniteur série.

## Build / Upload (PlatformIO)

Ouvre ce dossier avec PlatformIO, puis :
- Build
- Upload
- Monitor série (`115200` bauds)

## Solution simple pour recevoir sur téléphone Android

### Option recommandée : application **nRF Connect for Mobile**

1. Installer l'application **nRF Connect for Mobile** depuis le Play Store.
2. Activer Bluetooth (et localisation si demandé par Android).
3. Ouvrir l'app, scanner, puis se connecter au périphérique `ESP32-BLE-MSG`.
4. Ouvrir le service `6e400001-b5a3-f393-e0a9-e50e24dcca9e`.
5. Sur la caractéristique TX `6e400003-...` : activer les notifications (bouton `Notify`).
6. Les messages `Bonjour Android #...` apparaissent dans le log de l'app.

### Tester l'envoi Android -> ESP32

1. Dans la caractéristique RX `6e400002-...`, appuyer sur `Write`.
2. Envoyer un texte (exemple : `Salut ESP32`).
3. Vérifier le message reçu dans le moniteur série PlatformIO.

