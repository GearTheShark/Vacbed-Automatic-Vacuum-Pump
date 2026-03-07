#include <Arduino.h>
#include <avr/wdt.h>

// Пины
const uint8_t PRESSURE_PIN = A0;    // Датчик давления
const uint8_t POT_PIN = A1;         // Потенциометр
const uint8_t PUMP_PWM_PIN = 6;     // MOSFET насоса (PWM)
const uint8_t SAFETY_RELAY_PIN = 8; // MOSFET защитного реле

// Желаемый вакуум дефолтное значение
float dynamicTargetVacuumKPa = 25.0;
const float MAX_ALLOWED_VACUUM = 40.0;

// Гистерезис
const float PUMP_ON_DELTA = 5.0;  // Включение ниже цели
const float PUMP_OFF_DELTA = 2.0; // Выключение выше цели

// таймеры и задержки
const unsigned long RESTART_DELAY_MS = 15000; // задержка повторного включения
const unsigned long MAX_RUN_TIME_MS = 60000;  // максимум 1 минута непрерывной работы
const unsigned long STARTUP_DELAY_MS = 2000;  // задержка включения реле после старта

// АЦП
const float ADC_REFERENCE_VOLTAGE = 5.0;
const float ADC_MAX_VALUE = 1023.0;
const float SENSOR_MAX_VACuum_VOLT = 0.5;
const float SENSOR_ATM_VOLTAGE = 4.5;

// Аварийные границы датчика давления (УТОЧНИ)
const float SENSOR_ERROR_LOW = 0.4;
const float SENSOR_ERROR_HIGH = 4.8;

// сколько сэмплов брать для фильтрации сигнала (задержку вывести сюда надо бы)
const uint8_t FILTER_SAMPLES = 20;

// скважность PWM (две скорости насоса)
const uint8_t PWM_FULL = 255; // 100% duty cycle сигнала
const uint8_t PWM_LOW = 150;  // где то 11В при 12В питании (НАДО БУДЕТ СКОРРЕКТИРОВАТЬ ПОСЛЕ ТЕСТОВ)

// переменные

bool firstStartCompleted = false; // флаг первого запуска насоса
bool pumpState = false;
bool relayEnabled = false;

unsigned long lastPumpStopTime = 0;
unsigned long pumpStartTime = 0;

// управленче реле

void enableRelay()
{
  digitalWrite(SAFETY_RELAY_PIN, HIGH);
  relayEnabled = true;
  Serial.println("Safety relay ON");
}

void disableRelay()
{
  digitalWrite(SAFETY_RELAY_PIN, LOW);
  relayEnabled = false;
  Serial.println("Safety relay OFF");
}

//  аварийное выкл

void emergencyStop(const char *message)
{
  // авайрийно выключаем насос
  analogWrite(PUMP_PWM_PIN, 0);
  pumpState = false;

  // отключаем защитное реле
  disableRelay();

  Serial.println("!!! ERROR !!!");
  Serial.println(message);
  while (true)
  {
    // HALTED Полная остановка
    wdt_reset();
    delay(100);
  }
}

// считывание потенциометра

float readPotentiometerKPa()
{
  int raw = analogRead(POT_PIN); // 0–1023
  // Переводим в 10 - 40 кПа
  float ratio = (float)raw / 1023.0f;
  // float mapped = 30; //тест
  return constrain(10.0f + ratio * 20.0f, 10.0f, 30.0f); // linear 10 → 30 kPa
}

// считывание давления

float readPressureKPa()
{
  uint32_t sum = 0;

  for (uint8_t i = 0; i < FILTER_SAMPLES; i++)
  {
    sum += analogRead(PRESSURE_PIN);
    // float sensorNOW = analogRead(PRESSURE_PIN);
    delay(5);
  }
 //Serial.println(sum);
  float adcValue = sum / (float)FILTER_SAMPLES;
  // Serial.println(adcValue);
  float voltage = adcValue * ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE;
  //Serial.println(voltage);

  if (voltage < SENSOR_ERROR_LOW || voltage > SENSOR_ERROR_HIGH)
  {
    emergencyStop("Sensor voltage out of range");
  }

  float pressureKPa = (SENSOR_ATM_VOLTAGE - voltage) * (100.0f / (SENSOR_ATM_VOLTAGE - SENSOR_MAX_VACuum_VOLT));
  // Ограничение значений
  pressureKPa = constrain(pressureKPa, 0.0f, 100.0f);

  return pressureKPa;
}


// Управление скоростью насоса через pwm

void setPumpSpeed(float currentVacuum, float targetVacuum)
{
  float halfTarget = targetVacuum / 2.0;

  if (currentVacuum < halfTarget)
  {
    analogWrite(PUMP_PWM_PIN, PWM_FULL);
    Serial.println("Pump FULL Speed");
  }
  else
  {
    analogWrite(PUMP_PWM_PIN, PWM_LOW);
    Serial.println("Pump LOW Speed");
  }
}

// управление насосом вкл выкл
void controlPump(float currentVacuum, float targetVacuum)
{
  float pumpOnThreshold = targetVacuum - PUMP_ON_DELTA;
  float pumpOffThreshold = targetVacuum + PUMP_OFF_DELTA;

  unsigned long now = millis();

  // проверка времени непрерывной работы
  if (pumpState && firstStartCompleted &&
      (now - pumpStartTime) >= MAX_RUN_TIME_MS)
  {
    emergencyStop("Pump exceeded maximum continuous runtime");
  }

  if (!pumpState)
  {
    if (currentVacuum <= pumpOnThreshold && (!firstStartCompleted || (now - lastPumpStopTime) >= RESTART_DELAY_MS))
    {
      pumpState = true;
      pumpStartTime = now;
      Serial.println("Pump ON");
    }
  }
  else
  {
    if (currentVacuum >= pumpOffThreshold)
    {

      pumpState = false;
      lastPumpStopTime = now;

      // Сначала отключаем PWM
      analogWrite(PUMP_PWM_PIN, 0);

      // Затем отключаем реле
      // disableRelay();

      firstStartCompleted = true;

      Serial.println("Pump OFF");
    }
  }

  if (pumpState)
  {
    setPumpSpeed(currentVacuum, targetVacuum);
  }
}

// сетап

void setup()
{
  Serial.begin(57600);
  MCUSR = 0;     // Сброс флагов причины перезапуска
  wdt_disable(); // Отключаем watchdog пока что

  pinMode(PUMP_PWM_PIN, OUTPUT);
  pinMode(SAFETY_RELAY_PIN, OUTPUT);

  digitalWrite(SAFETY_RELAY_PIN, LOW);
  analogWrite(PUMP_PWM_PIN, 0);

  Serial.println("System booting...");
  delay(STARTUP_DELAY_MS); // ожидания пере запуском
  // Watchdog 2 секунды
  wdt_enable(WDTO_2S);

  Serial.println("Startup delay complete. System ready.");
  enableRelay();
  delay(1000);
}

// loop

void loop()
{
  wdt_reset();
  float currentVacuum = readPressureKPa();
  if (currentVacuum > MAX_ALLOWED_VACUUM && relayEnabled)
  {
    emergencyStop("Over-pressurized — possible backflow");
  }
  dynamicTargetVacuumKPa = readPotentiometerKPa();
  Serial.print("Vacuum: ");
  Serial.print(currentVacuum);
  Serial.print(" kPa / Target: ");
  Serial.print(dynamicTargetVacuumKPa);
  Serial.println(" kPa");
  controlPump(currentVacuum, dynamicTargetVacuumKPa);
}