
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>



int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
int servoPin = 2;

const char* ssid = "Notienenombre24";
const char* password = "Esmuydificil";

// Initialize Telegram BOT
String BOTtoken = "****************";  // your Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you
String CHAT_ID = "784196842";

bool sendPhoto = false;

WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

#define FLASH_LED_PIN 4
bool flashState = LOW;

//Checks for new messages every 1 second.
int botRequestDelay = 500;
unsigned long lastTimeBotRan;

//CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


void configInitCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    //Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  // Drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
}

void handleNewInput(int numNewMessages) {

  for (int i = 0; i < numNewMessages; i++) {
    bool notFound = true;
    String chat_id = String(bot.messages[i].chat_id);


    //chat_id2 = chat_id;
    String text = bot.messages[i].text;

    String from_name = bot.messages[i].from_name;
    if (from_name == "") from_name = "Guest";

    if (text.indexOf("Hola") >= 0 || text.indexOf("hola") >= 0 || text.indexOf("Eu!") >= 0 && notFound) {
      notFound = false; 
      String welcome = "Hola paper";
      bot.sendMessage(chat_id, welcome);
    }

    if (text.indexOf("activa") >= 0 && text.indexOf("sensor") >= 0 && notFound) {
      notFound = false; 
      Serial.print("activar sensor");
    }

    if (text.indexOf("oto") >= 0 && text.indexOf("puerta") >= 0 && notFound) {
      notFound = false; 
      bot.sendMessage(chat_id, "Dale, te mando una foto de la puerta");
      Serial.print("foto derecha");
    }

    if (text.indexOf("oto") >= 0 && text.indexOf("ventana") >= 0 && notFound) {
      notFound = false; 
      bot.sendMessage(chat_id, "Dale, te mando una foto de la ventana");
      Serial.print("foto izquierda");
    }

    if (text.indexOf("oto") >= 0 && text.indexOf("planta") >= 0 && notFound) {
      notFound = false; 
      bot.sendMessage(chat_id, "Dale, te mando una foto de la planta");
      Serial.print("foto centro");
    }

    if (text.indexOf("stado") >= 0 || text.indexOf("stado actual") >= 0 || text.indexOf("oto panoramica") >= 0 && notFound) {
      notFound = false; 
      bot.sendMessage(chat_id, "Updates...");
      fotoPanoramica();
    }


    if (text.indexOf("oto") >= 0 && text.indexOf("planta") == -1 && text.indexOf("ventana") == -1 && text.indexOf("puerta") == -1 && notFound) {
      notFound = false; 
      String keyboardJson = "[[\"Foto ventana\", \"Foto planta\"],[\"Foto puerta\"]]";
      bot.sendMessageWithReplyKeyboard(chat_id, "Me pediste una foto? De donde?", "", keyboardJson, true);
    }
  }
}

String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    // Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }

  //Serial.println("Connect to " + String(myDomain));


  if (clientTCP.connect(myDomain, 443)) {
    //Serial.println("Connection successful");

    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;

    clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    clientTCP.println();
    clientTCP.print(head);

    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        clientTCP.write(fbBuf, remainder);
      }
    }

    clientTCP.print(tail);

    esp_camera_fb_return(fb);

    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;

    while ((startTimer + waitTime) > millis())
    {
      // Serial.print(".");
      delay(100);
      while (clientTCP.available())
      {
        char c = clientTCP.read();
        if (c == '\n')
        {
          if (getAll.length() == 0) state = true;
          getAll = "";
        }
        else if (c != '\r')
          getAll += String(c);
        if (state == true) getBody += String(c);
        startTimer = millis();
      }
      if (getBody.length() > 0) break;
    }
    clientTCP.stop();
    //Serial.println(getBody);
  }
  else {
    getBody = "Connected to api.telegram.org failed.";
    //Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;
}

void setup() {



  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  // Init Serial Monitor
  Serial.begin(9600);

  // Set LED Flash as output
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, flashState);

  // Config and init the camera
  configInitCamera();

  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  //  Serial.println();
  //  Serial.print("Connecting to ");
  //  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    //Serial.print(".");
    delay(500);
  }
  //  Serial.println();
  //  Serial.print("ESP32-CAM IP Address: ");
  //  Serial.println(WiFi.localIP());
}

void tomarFoto() {
  flashState = !flashState;
  digitalWrite(FLASH_LED_PIN, flashState);
  sendPhotoTelegram();
  flashState = !flashState;
  digitalWrite(FLASH_LED_PIN, flashState);
  Serial.print("centrar");
  delay(1000);
}

void fotoPanoramica() {
  Serial.print("posicionderecha");
  delay(4000);
  tomarFoto();
  delay(500);
  Serial.print("centrar");
  delay(4000);
  tomarFoto();
  delay(500);
  Serial.print("posicionizquierda");
  delay(4000);
  tomarFoto();
  delay(500);
  Serial.print("centrar");
  delay(4000);
}

void loop() {


  if (millis() > lastTimeBotRan + botRequestDelay)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      handleNewInput(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }

  while (Serial.available() > 0 ) {
    String str = Serial.readString();
    if (str.indexOf("Capturar") > -1) {
      tomarFoto();
    }
    if (str.indexOf("msj") == 0 ) {
      bot.sendMessage(CHAT_ID, str.substring(3));
    }
    if (str.indexOf("hm") == 0) {
      String tempMsj = "La humedad actual en la planta es: " + str.substring(2);
      bot.sendMessage(CHAT_ID, tempMsj );
      delay(1000);
    }
    if (str.indexOf("movimientodetectado") == 0) {
      fotoPanoramica();
    }

  }
}
