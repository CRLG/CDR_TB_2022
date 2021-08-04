# CDR_TB_2022
Programme 2022 des Têtes Briquées pour la Coupe de France de Robotique

Compilable sous IDE Arduino 1.8.15 avec Arduino Mega

Le robot utilise le shield arduino "[adafruit motor shield v2](https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino)". Tous les ponts H sont utilisés par les moteurs pas à pas du robot.

Il y a quelques limitations d'utilisations des entrées/sorties (certaines sont utilisées par le shield). Ces limitations sont expliquées dans le code.

Le code est également prêt pôur une utilisation de:
NeoPixel un bandeau de leds
PixyCam la dernière version de la cmucam (site [PixyCam](https://pixycam.com/pixy-cmucam5/))

Les librairies suivantes sont nécessaires pour compiler le programme:   
 - **Wire.h** : pour l'i2c utilisé par le shield  
 - **Pixy2.h** : pour piloter la caméra  
 - **Adafruit_MotorShield.h** : pour commander le motor shield   
 - **Servo.h** : pour commander les servomoteurs  
 - **Adafruit_NeoPixel.h** : pour piloter le bandeau de leds
 - **Adafruit_VL53L0X.h** : librairie pour les capteurs de distance
 - **LiquidCrystal_I2C.h** : librairie pour l'écran
