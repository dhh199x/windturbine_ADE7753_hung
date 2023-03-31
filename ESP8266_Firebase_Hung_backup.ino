#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

int count = 0;
String data;
String data1;
String command[6] = {"VBAT", "VTAI", "ITAI", "PTAI", "VIN0", "IIN0"};
float   commandNum[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float y;
String rl;

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "TVT HOUSE"
#define WIFI_PASSWORD "66668888"

// Insert Firebase project API Key
#define API_KEY "AIzaSyBFUGlWTFW1ZEytlwM7c_1bIW_fN1RaY40"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://stm32andesp8266-default-rtdb.firebaseio.com/" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

//unsigned long sendDataPrevMillis = 0;
//int count = 0;
bool signupOK = false;

void setup(){
  // Khởi tạo Cổng serial RX TX
  Serial.begin(9600);
  // Kết nối wifi 
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();   // In ra cổng serial 
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  
  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  // kết nối fire base và duy trì kết nối
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop(){
  if (Serial.available()) {
    data = Serial.readString();
//    Serial.println(data);
    checkData();
    count++;
    Serial.println(count);
  }
  if( count >= 6){
    upData();
    count = 0;
  }
  // lấy tín hiệu từ firebase điều khiển relay
    Firebase.RTDB.getString(&fbdo, "RL1", &rl);
    //Serial.println(rl);
    if(rl == "1")
      Serial.println("T11");
      else Serial.println("T10");
      
    Firebase.RTDB.getString(&fbdo, F("RL2"), &rl);
    //Serial.println(rl);
    if(rl == "1")
      Serial.println("T21");
      else Serial.println("T20");
      
    Firebase.RTDB.getString(&fbdo, F("RL3"), &rl);
    //Serial.println(rl);
    if(rl == "1")
      Serial.println("T31");
      else Serial.println("T30");
      
    Firebase.RTDB.getString(&fbdo, F("RL4"), &rl);
    //Serial.println(rl);
    if(rl == "1")
      Serial.println("T41");
      else Serial.println("T40");
//  Serial.println("______________________________");
}

bool checkData(){
  // tach du lieu
  data1 = data.substring(0, 4);
  //Serial.println(data1);
  y = readDataNumber();
  //Serial.println(y);
  
  for(int i=0; i<6;i++){
    if(data1.compareTo(command[i]) == 0){ // tra ve 0 la bang
      commandNum[i] = y;
//      Serial.print(command[i]);
//      Serial.print(" = ");
//      Serial.print(commandNum[i]);      
      return 1;
    } 
  }
  return 0;
}

float readDataNumber(){
  return (data[4]-48)*100 +(data[5]-48)*10 +(data[6]-48) +(data[7]-48)/10.0 +(data[8]-48)/100.0;
}

void upData(){
  // Kết nối firebase và truyền dữ liệu  
  if (Firebase.ready() && signupOK ) {
    for(int i=0; i<6;i++){
      if (Firebase.RTDB.setFloat(&fbdo, command[i], commandNum[i])){
//        Serial.println("PASSED");
//        Serial.print(command[i]);
//        Serial.print(" : ");
//        Serial.println(commandNum[i]);
        
      }else {
//        Serial.println("FAILED");
//        Serial.println("REASON: " + fbdo.errorReason());
      }
    }
  }
}
