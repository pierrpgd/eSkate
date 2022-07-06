// MAC Address pour Nunchuck : 40:91:51:B9:F6:14
// MAC Address pour eSkate : 9C:9C:1F:C2:8A:C4

#include <esp_now.h>
#include <WiFi.h>

// Librairie ESP32 pour commander le moteur brushless
#include <ESP32Servo.h>
Servo ESC;
ESP32PWM pwm;

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xB9, 0xF6, 0x14}; //Adresse du Nunchuck, à téléverser sur eSkate

// Variables des messages
int var1;
int var2;
int var3;
int var4;
int var5;

// Variables du nunchuck
int iter; // iter : compteur des messages
int xAxis = 0;  // xAxis : valeur lue par le smartphone sur l'axe X du nunchuck (130 = joystick au centre)
int yAxis = 0;  // yAxis : valeur lue par le smartphone sur l'axe Y du nunchuck (130 = joystick au centre)
int yAxis_prec = 0;
int diff_yAxis;
int zBtn = 0; // zBtn :  valeur lue par le smartphone sur le bouton Z du nunchuck
int cBtn = 0; // cBtn :  valeur lue par le smartphone sur le bouton C du nunchuck

//Structure du message envoyé
typedef struct struct_message {
  int var1;
  int var2;
  int var3;
} struct_message;

//Structure du message reçu
typedef struct struct_incomingMessage {
  int var1;
  int var2;
  int var3;
  int var4;
  int var5;
} struct_incomingMessage;

struct_message message; // Message contenant les variables à envoyer
struct_incomingMessage incomingMessage; // Message contenant les variables reçues
esp_now_peer_info_t peerInfo;

int ESCValue = 90;  // ESCValue : commande de la vitesse "brute" du moteur intialisée à 90 (vitesse nulle)
float pwr = 0;  // pwr : commande de la vitesse en % du moteur initialisée à 0 (vitesse nulle soit ESCValue = 90)
float accel = 0;  // accel : variable d'accélération appliquée au moteur initialisée à 0
int dureeAccel = 5000;  // dureeAccel : variable définissant la durée d'accélération pour passer de V0 à Vmax
int pwrMin = 20;  // pwrMin : variable définissant la vitesse en % minimale lorsque le moteur est en marche avant 
int pwrMax = 50;  // pwrMax : variable définissant la vitesse en % maximale lorsque le moteur est en marche avant
int Vin = 5; // Vin : variable définissant la tension de la batterie
unsigned long timerRecepBT = millis(); // timerRecepBT : timer de lecture des données réceptionnées par l'arduino
unsigned long delaiRecepBT; // delaiRecepBT : délai de réception du message

// Boucle de vérification du message reçu
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  yAxis_prec = yAxis;
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  iter = incomingMessage.var1;
  xAxis = incomingMessage.var2;
  yAxis = incomingMessage.var3;
  zBtn = incomingMessage.var4;
  cBtn = incomingMessage.var5;
  
  delaiRecepBT = millis() - timerRecepBT; // Calcul du délaips entre les 2 derniers messages reçus
  timerRecepBT = millis();  // Réinitialisation du timer de lecture
  
  String texte = "iter : " + String(iter) + " xAxis : " + String(xAxis) + " yAxis : " + String(yAxis) + " zBtn : " + String(zBtn) + " cBtn : " + String(cBtn) + " diff_yAxis : " + String(diff_yAxis);
  Serial.println(texte);
}

void setup() {
  
  delay(2000);  // Délai pour l'initialisation de l'ESC
  ESP32PWM::allocateTimer(0); // Allocation des timers pour la commande de l'ESC
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  ESC.setPeriodHertz(50); // Définition de la fréquence du signal de commande de l'ESC
  ESC.attach(15,1000,2000); // Définition des paramètres du signal ESC (pin, min pulse width, max pulse width in microseconds) 
  delay(10);
  ESC.write(90);  // Initialisation de la vitesse du moteur
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);  // Enregistrement de l'adresse de réception
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Appairage à l'adresse de réception       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Enregistrement de la boucle de vérification de la réception
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

  diff_yAxis = yAxis - yAxis_prec;
  // ENVOI DES COMMANDES MOTEUR
  if (delaiRecepBT > 1500 && pwr != 0){
    // Ecriture dans le bus série pour diagnostic
    Serial.println("FREINAGE D'URGENCE !!!");
    // Appel de la fonction de freinage d'urgence
    FDU();
  }
  else{
    // Etape 1 : lecture des boutons Z et C (override des commandes précédentes)
    // Si le bouton Z est activé...
    if (zBtn == 1) {
      // Activation du freinage
      ESC.write(0);
      // Définition de la ESCValue à 0 (freinage maximal)
      //ESCValue = 0;
      // Définition de la puissance à 0 (pas d'accélération)
      pwr = 0;
    }
    // Si le bouton C est activé...
    else if (cBtn == 1) {
      // Annulation des commandes moteurs (roue libre)
      ESC.write(90);
      // Définition de la puissance à 0 (pas d'accélération)
      //pwr = 0;
    }
    // Si aucun bouton n'est activé...
    else {
      // Si le joystick est dirigé vers le haut...
      if (yAxis > 30 && diff_yAxis < 20) {
        // Si le joystick n'est pas poussé au maximum... (yAxis entre 30 et 90)
        if (yAxis < 90) {
          // La variable d'accélération est ajustée en fonction de la pression exercée sur yAxis (max = 1)
          accel = float(yAxis - 30) / 60;
        }
        // Si le joystick est poussé au maximum... (yAxis entre 90 et 100)
        // NB : si le smartphone perd le lien avec le joystick, la valeur envoyée sera aberrante (environ 1000)
        // La valeur de comparaison choisie est 230 par sécurité
        else if (yAxis < 100) {
          // La variable d'accélération est ajustée en fonction de la pression exercée sur yAxis (max = 2)
          accel = 1 + float(yAxis - 90) / 10;
        }
        // Si yAxis > 100 (valeur aberrante), alors le smartphone a perdu le contact avec le joystick...
        else {
          // Message d'alerte pour diagnostic
          Serial.println("FREINAGE D'URGENCE !!!");
          // Appel de la fonction de freinage d'urgence
          FDU();
        }
      }
      // Si le joystick est dirigé vers le bas...
      else if (yAxis < -10) {
        // Si le joystick n'est pas poussé au maximum... (yAxis entre -10 et -90)
        if (yAxis > -90) {
          // La valeur d'accélération est fixée à -1
          accel = -1;
        }
        // Si le joystick est poussé au maximum... (yAxis entre -90 et -100)
        else if (yAxis > -100) {
          // La valeur d'accélération est fixée à -2
          accel = -2;
        }
        // Si yAxis est < 30 (valeur aberrante), alors le smartphone a perdu le contact avec le joystick...
        else {
          // Message d'alerte pour diagnostic
          Serial.println("FREINAGE D'URGENCE !!!");
          // Appel de la fonction de freinage d'urgence
          FDU();
        }
      }
      // Si le joystick est centré... (yAxis entre 100 et 140)
      else {
        // La valeur d'accélération est fixée à 0
        accel = 0;
      }
  
      // Si la puissance envoyée au moteur est nulle (arrêt ou freinage) et que l'accélération demandée est positive...
      if (pwr == 0 && accel > 0) {
        // La valeur pwr est fixée à pwrMin
        // Nécessaire pour battre l'inertie à l'arrêt (coup de pouce du moteur)
        pwr = pwrMin;
      }
      // Si la puissance envoyée au moteur est positive (moteur en marche avant) OU si l'accélération demandée nulle ou négative...
      else {
        // La valeur de l'accélération est recalculée :
        // L'équation permet de lisser l'accélération en fonction du délai entre les 2 derniers messages reçus.
        // Avec une accélération = 1, le skate passe de pwrMin à pwrMax en un temps défini par la variable dureeAccel (ms).
        accel = accel * (float(pwrMax - pwrMin) / dureeAccel) * delaiRecepBT;
        // La puissance envoyée au moteur est augmentée de la valeur d'accélération recalculée à l'étape précédente.
        pwr = pwr + accel;
      }
  
      // Fonction de limitation de la variable pwr entre -100 (freinage maximal) et pwrMax
      if (pwr < -100) {
        pwr = -100;
      }
      if (pwr > pwrMax) {
        pwr = pwrMax;
      }
  
      // La valeur de ESCValue est définie sur pwr :
      // Valeur minimale = 0 si pwr = -100
      // Valeur maximale = 180 si pwr = 100
      ESCValue = map(pwr, -100, 100, 0, 180);
  
      // Envoi de la puissance au moteur
      ESC.write(ESCValue);
    }
  }
  
  
  // Stockage des valeurs à envoyer dans le message
  message.var1 = iter;
  message.var2 = pwr;
  message.var3 = Vin;

  // Envoi du message
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));

  delay(100);
}

// Fonction de freinage d'urgence :
void FDU() {
  // Commande de freinage envoyée au moteur
  ESC.write(0);
  // Attente du freinage complet
  delay(3000);
  // Réinitialisation de la puissance à 0
  pwr = 0;
  // Réinitialisation de la commande envoyée au moteur (roue libre)
  ESC.write(90);
}
