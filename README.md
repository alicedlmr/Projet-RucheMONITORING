# Projet Ruche MONITORING

Système de surveillance autonome pour ruche connecté en **LoRaWAN / TTN**, basé sur un **ESP32**, plusieurs capteurs environnementaux et une alimentation par **batterie + panneau solaire**.

## Objectif du projet

Le but du projet est de surveiller l’état d’une ruche à distance à l’aide de plusieurs capteurs afin de :

- suivre la **température** et l’**humidité**
- mesurer la **luminosité**
- suivre le **poids de la ruche**
- surveiller le **niveau de batterie**
- détecter un possible **essaimage** via une chute brutale du poids
- transmettre les données vers **The Things Network (TTN)**

---

## Fonctionnalités principales

- Lecture de plusieurs capteurs :
  - **HX711** + cellule de charge pour le poids
  - **BH1750** pour la luminosité
  - **DS18B20** pour les températures internes
  - **DHT22** pour température et humidité
- Transmission des mesures par **LoRaWAN**
- Gestion d’un **mode nuit**
- Gestion d’un **mode batterie faible**
- **Deep sleep** pour réduire la consommation
- Prise en compte de **downlinks TTN**
- Détection d’**alerte essaimage**

---

## Matériel utilisé

- **ESP32**
- **Module LoRa** piloté en AT commands via `Serial2`
- **HX711** + capteur de poids
- **BH1750**
- **2 x DS18B20**
- **2 x DHT22**
- Batterie + panneau solaire
- Buzzer

---

## Brochage utilisé

### HX711
- `DOUT` → GPIO **25**
- `SCK` → GPIO **26**

### BH1750
- `SDA` → GPIO **21**
- `SCL` → GPIO **22**

### DS18B20
- Bus OneWire → GPIO **33**

### DHT22
- DHT1 → GPIO **27**
- DHT2 → GPIO **19**

### Batterie
- mesure batterie → GPIO **35**

### LoRa
- RX2 → GPIO **16**
- TX2 → GPIO **17**

### Buzzer
- GPIO **23**

### Load Switch capteurs
- GPIO **2**

---

## Bibliothèques nécessaires

À installer dans l’IDE Arduino :

- `HX711`
- `BH1750`
- `OneWire`
- `DallasTemperature`
- `DHT sensor library`
- bibliothèques ESP32 intégrées (`WiFi.h`, `esp_bt.h`, `esp_sleep.h`)

---

## Structure du dépôt

Exemples de fichiers présents dans le dépôt :

- `EI4_Projet_ruche_BEEZ_Code.ino` : firmware principal
- `code_principale.ino` : autre version du code
- `codetestfinal.ino` : version de test
- `Code-Jalon-1.ino` : première étape du projet
- `payload_ttn.json` : decoder TTN simple
- `EI4_Projet_ruche_BEEE_Code_ttn.json` : decoder TTN étendu
- `Ubidots Decoder` : éléments liés à l’intégration Ubidots
- `test batterie` : tests liés à la mesure batterie

---

## Fonctionnement du programme

À chaque cycle :

1. **Réveil de l’ESP32**
2. **Mise sous tension des capteurs**
3. **Lecture des mesures**
   - poids
   - luminosité
   - températures
   - humidités
   - batterie
4. **Traitement local**
   - calcul de la tension batterie
   - calcul du pourcentage batterie
   - détection jour / nuit
   - détection d’un possible essaimage
5. **Construction du payload**
6. **Connexion / vérification LoRa**
7. **Envoi du payload vers TTN**
8. **Lecture éventuelle d’un downlink**
9. **Calcul de la prochaine période d’envoi**
10. **Passage en deep sleep**

---

## Gestion énergétique

Le système adapte son temps de veille selon l’état du système.

### Mode normal
Période d’envoi dépendante de la batterie :

- batterie `>= 80%` → **10 min** (ou **15 min** si modifié par downlink)
- batterie `>= 60%` → **20 min**
- batterie `>= 40%` → **30 min**
- batterie `>= 20%` → **45 min**
- batterie `< 20%` → **60 min**

### Mode nuit
Le mode nuit est activé si :

- `lux < 20`

Dans ce cas :

- période d’envoi = **45 min**

### Forçage du mode nuit
Le mode nuit peut aussi être forcé par downlink.

---

## Détection d’essaimage

Une alerte d’essaimage est générée si :

- la lecture HX711 est valide
- une mesure précédente existe
- la précédente mesure est suffisamment récente
- la différence de poids est inférieure à `-1.5 kg`

Cela met le champ `swarm = 1`.

---

## Mesure batterie

La tension batterie est calculée à partir de l’ADC de l’ESP32 avec un facteur de calibration :

const float VBAT_RATIO = 1.36f;

Le pourcentage batterie est calculé à partir de la tension mesurée en mV.

Exemple de seuils :
	-	4200 mV → 100%
	-	4100 mV → 90%
	-	4000 mV → 80%
	-	3900 mV → 60%
	-	3800 mV → 40%
	-	3700 mV → 20%
	-	3600 mV → 10%
	-	3500 mV → 5%
	-	3300 mV → 0%

---

## Payload envoyé vers TTN

Le firmware envoie un payload enrichi contenant :
	- 	températures DS18B20
	-   températures / humidités DHT
	-	luminosité
	-	poids
	-	pourcentage batterie
	-	alerte essaimage
	-	mode de fonctionnement
	-	tension batterie
	-	informations sur le dernier downlink reçu

**Champs décodés côté TTN**

Exemple de champs obtenus selon le decoder utilisé :
	-	t_1
	-	t_3
	-	t_i
	-	h_i
	-	t
	-	h
	-	l
	-	weight_kg
	-	bv
	-	swarm
	-	run_mode
	-	run_mode_txt
	-	vm
	-	last_dl_cmd
	-	last_dl_arg1
	-	last_dl_arg2
	-	last_dl_status
	-	last_dl_status_txt

---

## Signification de run_mode
	-	0 → normal
	-	1 → override
	-	2 → low_battery
	-	3 → night

---

## Downlinks TTN disponibles

Les downlinks se font sur FPort 1.

## 1. Override de la période d’envoi

Format :

01 mm

où mm est le nombre de minutes en hexadécimal.

Exemples :
	-	01 01 → 1 min
	-	01 0A → 10 min
	-	01 1E → 30 min
	-	01 3C → 60 min

---

## 2. Forcer le mode nuit ON

03

Effet :
	-	force nightActive = true

---

## 3. Forcer le mode nuit OFF

04

Effet :
	-	force nightActive = false

---

## 4. Changer la période haute batterie

Format :

05 mm

Valeurs acceptées :
	-	05 0A → 10 min
	-	05 0F → 15 min

---

## 5. Reset de la configuration

07

Effet :
	-	reset de l’override
	-	reset du forçage nuit
	-	remise des paramètres par défaut

---

## 6. Supprimer seulement l’override

08

Effet :
	-	supprime periodOverride
	-	remet la période calculée automatiquement

---

## Comment envoyer un downlink sur TTN

Dans TTN :
	1.	Ouvrir le device
	2.	Aller dans Messaging
	3.	Ajouter un Downlink
	4.	Choisir :
	-	FPort = 1
	-	Payload type = Bytes
	5.	Entrer par exemple :

03

ou :

01 1E


---

## Installation / utilisation

**1. Cloner le dépôt**

git clone https://github.com/alicedlmr/Projet-RucheMONITORING.git

**2. Ouvrir le projet**

Ouvrir le fichier principal dans l’IDE Arduino :

EI4_Projet_ruche_BEEZ_Code.ino

**3. Vérifier les bibliothèques**

Installer toutes les bibliothèques nécessaires.

**4. Vérifier les paramètres matériels**

Adapter si besoin :
	-	broches
	-	calibration batterie
	-	calibration poids
	-	seuil de luminosité

**5. Compiler et téléverser**

Sélectionner la carte ESP32 correspondante puis téléverser.

---

## Calibration

**Calibration poids**

Le poids est calculé avec :
```cpp
float w = (float)(SCALE_SIGN * (rawAvg - SCALE_OFFSET)) / SCALE_FACTOR;
w = WEIGHT_A * w + WEIGHT_B;
```
Paramètres à ajuster si nécessaire :
	-	SCALE_OFFSET
	-	SCALE_FACTOR
	-	WEIGHT_A
	-	WEIGHT_B

**Calibration batterie**

Le paramètre à ajuster est :
```cpp
const float VBAT_RATIO = ...
```
Il doit être recalé à partir d’une mesure au multimètre.

---

## Intégration TTN / Webhooks

Le projet peut être connecté à :
	-	TTN
	-	BEEP
	-	Ubidots

Les fichiers de decoders fournis dans le dépôt permettent d’adapter les noms des champs selon la plateforme utilisée.

---

## Améliorations possibles
	-	amélioration du filtrage des données envoyées aux webhooks
	-	meilleure gestion du RSSI / SNR
	-	optimisation supplémentaire de la consommation
	-	ajout de journaux de debug séparés
	-	simplification des payloads selon la plateforme cible

---

## Auteurs :

Ahmed ELSHAZLY | Alice DELMAR | Georges SKAF | Madjid CHIKHI

Groupe 3 BEEZ EI4 FISA

Projet réalisé dans le cadre du projet Ruche Monitoring à Polytech.
