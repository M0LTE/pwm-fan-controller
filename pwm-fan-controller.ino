#include <OneWire.h> 
#include <DallasTemperature.h>

#define COOLING_LED 13
#define FAULT_LED 8
#define ONE_WIRE_BUS 10
#define OC1A_PIN 9
#define setpoint 25.0
#define sensitivityFactor 3
#define maxDuty 50

const word PWM_FREQ_HZ = 25000; //Adjust this value to adjust the PWM frequency
const word TCNT1_TOP = 16000000/(2*PWM_FREQ_HZ);

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);
int deviceCount;

void setup() {
  
  pinMode(OC1A_PIN, OUTPUT);
  pinMode(FAULT_LED, OUTPUT);
  pinMode(COOLING_LED, OUTPUT);

  // Clear Timer1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Set Timer1 configuration
  // COM1A(1:0) = 0b10   (Output A clear rising/set falling)
  // COM1B(1:0) = 0b00   (Output B normal operation)
  // WGM(13:10) = 0b1010 (Phase correct PWM)
  // ICNC1      = 0b0    (Input capture noise canceler disabled)
  // ICES1      = 0b0    (Input capture edge select disabled)
  // CS(12:10)  = 0b001  (Input clock select = clock/1)
  
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;

  sensors.begin(); 
  deviceCount = sensors.getDeviceCount();
  
  setPwmDuty(0);
  Serial.begin(9600); 
  Serial.println("<25kHz PWM signal on Pin 9. Connect directly to blue wire on JC972 fans. One or more DS18B20 expected on Pin 10, 4k7 resistor from VCC to Data. When there are no sensors, pin 8 goes high and PWM goes to 100% duty. When above the setpoint, pin 13 (built-in LED) comes on.>");
}

void loop() {

  sensors.requestTemperatures();

  float tempC = -50;
  for (int i = 0; i < deviceCount; i++){
    float t = sensors.getTempCByIndex(i);
    if (t > tempC){
      tempC = t;
    }
  }

  if (tempC == -50){
    setPwmDuty(100);
    digitalWrite(FAULT_LED, HIGH);
    Serial.println("Fault - no sensors. Fan set to max.");
  } else {
    digitalWrite(FAULT_LED, LOW);
    
    float amountAboveSetpoint = tempC - setpoint;
  
    Serial.print("Sensors: ");
    Serial.print(deviceCount);
    Serial.print(", highest temp: ");
    Serial.print(tempC);
    Serial.print("C, setpoint: ");
    Serial.print(setpoint);
    Serial.print("C, ");
  
    int duty = 0;
    if (amountAboveSetpoint < 0){
      Serial.println("duty: 0%");
      digitalWrite(COOLING_LED, LOW);
    } else {
      digitalWrite(COOLING_LED, HIGH);
      duty = amountAboveSetpoint * sensitivityFactor;
      bool capped = false;
      if (duty > maxDuty){
        capped = true;
        duty = maxDuty;
      }
      Serial.print("excursion: ");
      Serial.print(amountAboveSetpoint);
      Serial.print(", duty: ");
      Serial.print(duty);
      Serial.print("%");
      if (capped){
        Serial.println(" (capped)");
      } else {
        Serial.println();
      }
    }

    setPwmDuty(duty);
  }

  delay(1000);
}

void setPwmDuty(byte duty) {
  OCR1A = (word) (duty*TCNT1_TOP)/100;
}
