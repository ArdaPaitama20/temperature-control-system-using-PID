  #include "max6675.h"
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>

  LiquidCrystal_I2C lcd(0x27, 16, 2);  

  //Pin untuk membaca potensiometer
  int potentiometerPin = A0;

  //Pin untuk sensor ultrasonik
  #define echoPin 4   
  #define trigPin 3  
  int baca;
  long waktu, jarak;

  //Pin untuk sensor thermocouple tipe K
  int thermoDO = 6;
  int thermoCS = 7;
  int thermoCLK = 8;
  MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

  //Definisikan pin untuk pwm
  int heater = 11;

  //Definisikan pin untuk relay yang mengontrol solenoid valve
  #define relayPin  10
 
  float kp = 3 ; 
  float ki = 4 ; 
  float kd = 5 ; 

  float PID_p,PID_i,PID_d,Suhu,pid;
  float error,errorx,sumerr;
  float Setpoint;

  // Batas atas dan bawah untuk kontrol relay
  #define batas_atas 10
  #define batas_bawah 10

  void setup() {
  //Inisialisasi komunikasi serial untuk debugging
  Serial.begin(9600);
  pinMode(heater, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); 
  delay(500);

  //Inisialisasi LCD
  lcd.init();
  lcd.backlight();
  }

  void loop() {
  float Suhu1 = thermocouple.readCelsius();
  float Suhu2 = (Suhu1 * 0.9987) - 0.1377;

  error = Setpoint - Suhu2;
  PID_p = kp * error;
  sumerr = error + errorx;
  PID_i = ki * sumerr;
  PID_d = error - errorx;
  pid = PID_p + PID_i + PID_d;

  if (pid > 255) {
   pid = 255;
  }
  if (pid < 0) {
   pid = 0;
  }

  analogWrite(heater,pid);

  //Mengirimkan data ke Serial Monitor
  Serial.print(Setpoint);
  Serial.print(", ");
  Serial.print(Suhu2);
  Serial.print(", ");
  Serial.print(pid);
  Serial.print(", ");
  Serial.println(jarak);

  delay(100);
  
  errorx = error;

  //Baca nilai potensiometer dan konversi ke Setpoint (suhu yang diinginkan)
  int potValue = analogRead(A0);
  Setpoint = map(potValue, 0, 1023, 0, 40); 

  //Pengukuran level air
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  waktu = pulseIn(echoPin, HIGH);
  jarak = waktu / 29 / 2;
  baca = jarak;

  if (jarak >= batas_atas){             
    digitalWrite(relayPin, LOW);       
  } else if (jarak <= batas_bawah) {  
    digitalWrite(relayPin, HIGH);    
  }

  //menampilkan data di LCD
  lcd.setCursor(0, 0); 
  lcd.print("Setpoint: ");
  lcd.print(Setpoint);
  lcd.setCursor(0, 1); 
  lcd.print("Suhu: ");
  lcd.print(Suhu2);

  delay(500);
  }
