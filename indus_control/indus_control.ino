#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

const char *ssid =  "abc";     // replace with your wifi ssid and wpa2 key
const char *pass =  "12345678";

WiFiClient client;
double i = 2;

 
void setup() 
{
       Serial.begin(9600);
       delay(10);
               
       Serial.println("Connecting to ");
       Serial.println(ssid); 
 
       WiFi.begin(ssid, pass); 
       while (WiFi.status() != WL_CONNECTED) 
          {
            delay(500);
            Serial.print(".");
          }
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: "); 
      Serial.println(WiFi.localIP()); 
}

void loop() {
  
  i = i * i;
  double x = sin(i);
  
  // put your main code here, to run repeatedly:
  if(WiFi.status()== WL_CONNECTED){ 
    HTTPClient http;
    String url = "http://192.168.43.60:8000/plotter/?speed=" + String(i) + "&position=0.5";
    http.begin(url);
    http.addHeader("Content-Type", "html/text");
    int httpCode = http.GET();   //Send the request
    String payload = http.getString();
    Serial.println(httpCode);   //Print HTTP return code
    Serial.println(payload);    //Print request response payload
    http.end();  //Close connection
  }else{
 
   Serial.println("Error in WiFi connection");   
 }
 
  delay(.5);  //Send a request every 30 seconds
 
  

}
