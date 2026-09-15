const int TPS_PIN = A0;

const float ADC_REFERENCE = 5.0;      // Max ADC voltage
const float TPS_MIN_VOLTAGE = 0.684;   // Calibrate to your sensor's closed-throttle voltage
const float TPS_MAX_VOLTAGE = 1.711;   // Calibrate to your sensor's wide-open-throttle voltage

float voltageToPercent(float voltage)
{
  float percent = (voltage - TPS_MIN_VOLTAGE) /
                  (TPS_MAX_VOLTAGE - TPS_MIN_VOLTAGE) * 100.0;

  return constrain(percent, 0.0, 100.0);
}

void setup()
{
  Serial.begin(115200);
  pinMode(TPS_PIN, INPUT);
}

void loop()
{
  int raw = analogRead(TPS_PIN);
  float voltage = raw * (ADC_REFERENCE / 1023.0);
  float percent = voltageToPercent(voltage);

  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print("  Voltage: ");
  Serial.print(voltage, 3);
  Serial.print("  Throttle: ");
  Serial.print(percent, 1);
  Serial.println("%");

  delay(100); // adjust or remove for faster sampling
}