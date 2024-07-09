//#include <MPU6050.h>

#include <Adafruit_MPU6050.h>
#include <math.h> // For mathematical functions

// Define I2C address and other constants
// #define MPU_ADDR 0x68
// #define ACCEL_SENSITIVITY 16384.0  // Sensitivity of accelerometer
// #define GYRO_SENSITIVITY 131.07    // Sensitivity of gyroscope

// Thresholds for fall detection (adjust these values based on testing)
#define FALL_ACCEL_THRESHOLD 1.5     // g-force threshold for initial impact
#define FALL_DURATION 10     
//float ax, ay, az, gx, gy, gz;



#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
//#include <HTTPClient.h>
#include<base64.h>
#include <WiFiClientSecure.h>
#include <MPU6050.h>

#include <Wire.h>
#include<math.h>

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D6;
const uint8_t sda = D7;

// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384.0;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;


const char* ssid = "CHENAB";
const char* password = "44zMf3QqdU&KC3Mv";

//IMPORTANT CHANGE FOR TOMMORROW COMMENT OUT THE TOP ONE AND CHANGE THE BOTTOM

//const char* ssid = "IITRPR";

//const char* password = "V#6qF?pyM!bQ$%NX";

const char * authToken = "e069704ba3e90f51d00cfd4b3ebbec31";
const char* accountSID = "AC02348f0af028bea9facf4e4e57c3f3c4"; //AC02348f0af028bea9facf4e4e57c3f3c4


const char* twilioPhoneNumber = "+13012731935";
//const char* recipient = "+918619552765";
const char* recipient = "+917007690337";
int lastTime = 0;

bool buttonInterrupted = false;
const int debounceTime = 50; // Time in milliseconds to debounce the button press
volatile unsigned long startTime = 0; // Stores the time the buzzer starts buzzing
volatile bool buttonPressed = false;
volatile unsigned long prevDebounceTime = 0; // Stores the previous debounce time


void setup()
{
  pinMode(D3,OUTPUT);
  pinMode(D2,INPUT);
  Serial.begin(115200);
  Serial.println();
  Wire.begin(sda, scl);
  MPU6050_Init();

  WiFi.begin(ssid, password);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
  int c = 1 ;
  //while(c-- > 0) sendSMS(recipient , "I will always choose mohit");
}
int c =1 ;
void loop() {
  
  double mag = ReadSensor();
  //  if (mag > 2.00) {
  //   Serial.println(mag);
  //   digitalWrite(D3, HIGH);  // Turn on buzzer
  //   startTime = millis();          // Record start time
  //   buttonPressed = false;         // Reset button press flag
  // }

  // // Button debouncing and press detection
  // unsigned long currentTime = millis();
  // if (digitalRead(D2) && (currentTime - startTime) >= debounceTime) {
  //   buttonPressed = true;
  //   prevDebounceTime = currentTime;
  //   Serial.println("fall not detected");
  // }

  // // Check if buzzer is on and 5 seconds have passed
  // if (digitalRead(D3) == HIGH && (millis() - startTime) >= 5000) {
  //   if (!buttonPressed) {
  //     sendSMS(recipient, "Fall detected!"); // Send SMS if button not pressed
  //   }
  //   digitalWrite(D3, LOW); //

  //    }
     int flag = 1;

     if(mag > 2 ){
      int startTime = millis();
      digitalWrite(D3, 1);
      //delay(50);
      while( millis()-startTime <= 5000){
        if(digitalRead(D2)){
          long long button_pressed = millis();
          delay(50);
        if(digitalRead(D2)){

          Serial.println("fall not detected");
          digitalWrite(D3, LOW);
          flag = 0;
          delay(25); 
          break;
        }
        }
         delay(25); 
      }
      digitalWrite(D3, LOW);
      if(flag) sendSMS(recipient, "Fall detected!");
     }
  




    delay(25);

}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

double ReadSensor(){
  //Serial.println(digitalRead(D7));
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  T = (double)Temperature/340+36.53; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;

  // Serial.print("Ax: "); Serial.print(Ax);
  // Serial.print(" Ay: "); Serial.print(Ay);
  // Serial.print(" Az: "); Serial.print(Az);
  // Serial.print(" T: "); Serial.print(T);
  // Serial.print(" Gx: "); Serial.print(Gx);
  // Serial.print(" Gy: "); Serial.print(Gy);
  // Serial.print(" Gz: "); Serial.println(Gz);


  double mag = sqrt(Ax*Ax + Ay*Ay + Az*Az );

  return mag ;


  delay(100);

}




void sendSMS(const char* recipient, const char* message) {
  HTTPClient http;
   WiFiClientSecure client;
  client.setInsecure();
  delay(50);

  // Twilio API URL (replace with your Twilio region if needed)
 // String url = "https://api.twilio.com/Accounts/" + String(accountSID) + 
               //"/Messages.json";

                //https://api.twilio.com/2010-04-01/Accounts
    //String url =   "https://api.twilio.com/2010-04-01/Accounts/";
    String url = "https://api.twilio.com/2010-04-01/Accounts/AC02348f0af028bea9facf4e4e57c3f3c4/Messages.json";
              

  http.begin(client ,  url.c_str());
  delay(50);
  //http.addHeader("Authorization", "Basic " + base64::encode((String(accountSID) + ":" + String(authToken)).c_str()));
  http.addHeader("Authorization", "Basic " + base64::encode(String(accountSID) + ":" + String(authToken).c_str()));
  delay(50);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  delay(50);
  int httpResponseCode = http.POST("From=" + String(twilioPhoneNumber) + "&To=" + String(recipient) + "&Body=" + String(message));
  delay(50);

//  int httpResponseCode = http.ResponseCode();

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.print("SMS Sent! Code: ");
    Serial.print(httpResponseCode);
    Serial.println(" Response: " + response);
  } else {
    Serial.print("Error sending SMS: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

