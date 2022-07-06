// MAC Address pour Nunchuck : 40:91:51:B9:F6:14
// MAC Address pour eSkate : 9C:9C:1F:C2:8A:C4

#include <esp_now.h>
#include <WiFi.h>

#include "wii_i2c.h"
#define PIN_SDA  04//32 // Fil vert
#define PIN_SCL  27//33 // Fil jaune
#define WII_I2C_PORT 0

uint8_t broadcastAddress[] = {0x9C, 0x9C, 0x1F, 0xC2, 0x8A, 0xC4}; //Adresse du eSkate, à téléverser sur Nunchuck

// Variables des messages
int var1;
int var2;
int var3;
int var4;
int var5;

//Structure du message envoyé
typedef struct struct_message {
  int var1;
  int var2;
  int var3;
  int var4;
  int var5;
} struct_message;

//Structure du message reçu
typedef struct struct_incomingMessage {
  int var1;
  int var2;
  int var3;
} struct_incomingMessage;

struct_message message; // Message contenant les variables à envoyer
struct_message incomingMessage; // Message contenant les variables reçues
esp_now_peer_info_t peerInfo;
int iter = 0;
int pwr;
int Vin;

// Boucle de vérification du message reçu
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  iter = incomingMessage.var1;
  pwr = incomingMessage.var2;
  Vin = incomingMessage.var3;
  
  String texte = "iter : " + String(iter) + " pwr : " + String(pwr) + " Vin : " + String(Vin) + " xAxis : " + String(message.var2) + " yAxis : " + String(message.var3) + " zBtn : " + String(message.var4) + " cBtn : " + String(message.var5);
  Serial.println(texte);
}

void setup() {

  Serial.begin(115200);

  if (wii_i2c_init(WII_I2C_PORT, PIN_SDA, PIN_SCL) != 0) {
    Serial.printf("ERROR initializing wii i2c controller\n");
    return;
  }
  else {
    Serial.printf("Wii i2c controller initialized\n");
  }
  wii_i2c_request_state();
  
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

  const unsigned char *data = wii_i2c_read_state();
  wii_i2c_request_state();
  wii_i2c_nunchuk_state state;
  wii_i2c_decode_nunchuk(data, &state);
  
  // Stockage des valeurs à envoyer dans le message
  message.var1 = iter + 1;
  message.var2 = state.x;
  message.var3 = state.y;
  message.var4 = state.z;
  message.var5 = state.c;

  // Envoi du message
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));

  delay(100);
}
