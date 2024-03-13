//more details on robohub instructables page//

#define PWMAL D5  
#define PWMAR D6  

#define PWMBL D7   
#define PWMBR D8   

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

String command;      //String to store app command state.
int speedCar = 100;  // 400 - 1023.
int speed_Coeff = 3;

const char* ssid = "NodeMCU Car";
ESP8266WebServer server(80);

void setup() {
  stopRobot();


  Serial.begin(115200);

  // Connecting WiFi

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  // Starting WEB-server
  server.on("/", HTTP_handleRoot);
  server.onNotFound(HTTP_handleRoot);
  server.begin();
}

void goAhead() {

  analogWrite(PWMAR, speedCar);
  analogWrite(PWMAL, 0);


  analogWrite(PWMBR, speedCar);
  analogWrite(PWMBL, 0);



  Serial.println("Forward");
}

void goBack() {
  analogWrite(PWMAR, 0);
  analogWrite(PWMAL, speedCar);


  analogWrite(PWMBR, 0);
  analogWrite(PWMBL, speedCar);

  Serial.println("Backward");


}

void goRight() {
  analogWrite(PWMAR, 0);
  analogWrite(PWMAL, speedCar);


  analogWrite(PWMBR, speedCar);
  analogWrite(PWMBL, 0);

}

void goLeft() {
  analogWrite(PWMAR, speedCar);
  analogWrite(PWMAL, 0);


  analogWrite(PWMBR, 0);
  analogWrite(PWMBL, speedCar);


}




void stopRobot() {
  analogWrite(PWMAL, 0);
  analogWrite(PWMAR, 0);


  analogWrite(PWMBL, 0);
  analogWrite(PWMBR, 0);

}

void loop() {
  server.handleClient();

  command = server.arg("State");
  if (command == "F") goAhead();
  else if (command == "B") goBack();
  else if (command == "L") goLeft();
  else if (command == "R") goRight();
  else if (command == "0") speedCar = 400;
  else if (command == "1") speedCar = 470;
  else if (command == "2") speedCar = 540;
  else if (command == "3") speedCar = 610;
  else if (command == "4") speedCar = 680;
  else if (command == "5") speedCar = 750;
  else if (command == "6") speedCar = 820;
  else if (command == "7") speedCar = 890;
  else if (command == "8") speedCar = 960;
  else if (command == "9") speedCar = 1023;
  else if (command == "S") stopRobot();
}

void HTTP_handleRoot(void) {

  if (server.hasArg("State")) {
    Serial.println(server.arg("State"));
  }
  server.send(200, "text/html", "");
  delay(1);
}
