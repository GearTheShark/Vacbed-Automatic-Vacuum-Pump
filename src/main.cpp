#include <Arduino.h>
#include <avr/wdt.h>

// Пины
const uint8_t PRESSURE_PIN = A0;    // Датчик давления
const uint8_t POT_PIN = A1;         // Потенциометр
const uint8_t PUMP_PWM_PIN = 6;     // MOSFET насоса (PWM)
const uint8_t SAFETY_RELAY_PIN = 8; // MOSFET защитного реле

// Желаемый вакуум дефолтное значение
float dynamicTargetVacuumKPa = 25.0;
const float MAX_ALLOWED_VACUUM = 35.0;

// Гистерезис
const float PUMP_ON_DELTA = 5.0;  // Включение ниже цели
const float PUMP_OFF_DELTA = 2.0; // Выключение выше цели

// таймеры и задержки
const unsigned long RESTART_DELAY_MS = 15000;       // задержка повторного включения
const unsigned long MAX_RUN_TIME_MS = 60000;        // максимум 1 минута непрерывной работы
const unsigned long STARTUP_DELAY_MS = 1000;        // задержка включения реле после старта
const unsigned long PUMP_RAMP_TIME_MS = 250;        // время плавного разгона
const unsigned long PUMP_SPEED_RAMP_TIME_MS = 500; // время плавного изменения скорости

// АЦП
const float ADC_REFERENCE_VOLTAGE = 5.0;
const float ADC_MAX_VALUE = 1023.0;
const float SENSOR_MAX_VACUUM_VOLT = 0.5;
const float SENSOR_ATM_VOLTAGE = 4.5;

// Аварийные границы датчика давления (УТОЧНИ)
const float SENSOR_ERROR_LOW = 0.4;
const float SENSOR_ERROR_HIGH = 4.8;

// сколько сэмплов брать для фильтрации сигнала (задержку вывести сюда надо бы)
const uint8_t FILTER_SAMPLES = 31;
// коэффициент сглаживания потенциометра
const float POT_SMOOTH_ALPHA = 0.1f;

// кольцевой буфер для медианного фильтра
uint16_t pressureBuffer[FILTER_SAMPLES] = {0}; // заполняем нулями во избежании мусора
uint8_t pressureIndex = 0;
uint8_t pressureCount = 0;

// переменные изменения скорости насосаё
uint8_t rampStartPWM = 0;
uint8_t rampTargetPWM = 0;
unsigned long speedRampStartTime = 0;
uint8_t currentPWM = 0;

// скважность PWM (две скорости насоса)
const uint8_t PWM_FULL = 255; // 100% duty cycle сигнала
const uint8_t PWM_LOW = 150;  // где то 11В при 12В питании (НАДО БУДЕТ СКОРРЕКТИРОВАТЬ ПОСЛЕ ТЕСТОВ)
const uint8_t PWM_MIN = 80;   // начало разгона

// переменные
float filteredPotKPa = 20.0f;     // стартовое значение потенциометра
bool firstStartCompleted = false; // флаг первого запуска насоса
bool pumpState = false;
bool relayEnabled = false;
bool pumpRamping = false;  // мотор разгоняется?
bool speedRamping = false; // мотор меняет скорость?

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
  currentPWM = 0;
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
  int raw = analogRead(POT_PIN);

  float ratio = (float)raw / 1023.0f;
  float target = constrain(10.0f + ratio * 20.0f, 10.0f, 30.0f);

  // экспоненциальное сглаживание
  filteredPotKPa += POT_SMOOTH_ALPHA * (target - filteredPotKPa);

  return filteredPotKPa;
}

// вычесление медианы из буффера

uint16_t medianOfBuffer(uint16_t *data, uint8_t size)
{
  uint16_t temp[FILTER_SAMPLES];

  for (uint8_t i = 0; i < size; i++)
    temp[i] = data[i];

  // сортировка
  for (uint8_t i = 0; i < size - 1; i++)
  {
    for (uint8_t j = i + 1; j < size; j++)
    {
      if (temp[j] < temp[i])
      {
        uint16_t t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  return temp[size / 2];
}

// считывание давления

float readPressureKPa()
{
  // читаем одно значение
  uint16_t raw = analogRead(PRESSURE_PIN);

  // пишем в кольцевой буфер
  pressureBuffer[pressureIndex] = raw;
  pressureIndex = (pressureIndex + 1) % FILTER_SAMPLES;

  if (pressureCount < FILTER_SAMPLES)
  {
    pressureCount++;
  }

  // в первый запуск надо сначал заполнить буффер
  uint16_t adcValue;

  if (pressureCount < FILTER_SAMPLES)
  {
    adcValue = raw;
  }
  else
  {
    // высчитывем медиану
    adcValue = medianOfBuffer(pressureBuffer, FILTER_SAMPLES);
  }

  float voltage = adcValue * ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE;

  if (voltage < SENSOR_ERROR_LOW || voltage > SENSOR_ERROR_HIGH)
  {
    emergencyStop("Sensor voltage out of range");
  }

  float pressureKPa = (SENSOR_ATM_VOLTAGE - voltage) * (100.0f / (SENSOR_ATM_VOLTAGE - SENSOR_MAX_VACUUM_VOLT));
  // ограничения
  pressureKPa = constrain(pressureKPa, 0.0f, 100.0f);

  return pressureKPa;
}

// Управление скоростью насоса через pwm

void setPumpSpeed(float currentVacuum, float targetVacuum)
{
  float halfTarget = targetVacuum / 2.0;
  uint8_t desiredPWM = (currentVacuum < halfTarget) ? PWM_FULL : PWM_LOW;

  unsigned long now = millis();

  if (pumpRamping)
  {
    unsigned long elapsed = now - pumpStartTime;

    if (elapsed >= PUMP_RAMP_TIME_MS)
    {
      pumpRamping = false;
      speedRamping = false;    // сброс второго ramp
      currentPWM = desiredPWM; // синхронизация
      analogWrite(PUMP_PWM_PIN, desiredPWM);
      return;
    }

    float progress = (float)elapsed / PUMP_RAMP_TIME_MS;
    uint8_t pwm = (uint8_t)(desiredPWM * progress);

    if (pwm < PWM_MIN)
      pwm = PWM_MIN;
    currentPWM = pwm;
    analogWrite(PUMP_PWM_PIN, pwm);
    return;
  }

  // ПЛАВНОЕ ИЗМЕНЕНИЕ СКОРОСТИ
  // если нужно изменить скорость — запускаем ramp
  if (!speedRamping && currentPWM != desiredPWM)
  {
    speedRamping = true;
    rampStartPWM = currentPWM;
    rampTargetPWM = desiredPWM;
    speedRampStartTime = now;
  }

  if (speedRamping)
  {
    unsigned long elapsed = now - speedRampStartTime;

    if (elapsed >= PUMP_SPEED_RAMP_TIME_MS)
    {
      speedRamping = false;
      currentPWM = rampTargetPWM;
      analogWrite(PUMP_PWM_PIN, currentPWM);
      return;
    }

    float progress = (float)elapsed / PUMP_SPEED_RAMP_TIME_MS;

    currentPWM = rampStartPWM + (uint8_t)((rampTargetPWM - rampStartPWM) * progress);

    analogWrite(PUMP_PWM_PIN, currentPWM);
    return;
  }

  // обычный режим
  currentPWM = desiredPWM;
  analogWrite(PUMP_PWM_PIN, currentPWM);
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
      pumpRamping = true;
      Serial.println("Pump ON (ramping)");
    }
  }
  else
  {
    if (currentVacuum >= pumpOffThreshold)
    {
      pumpRamping = false;
      pumpState = false;
      lastPumpStopTime = now;

      // Сначала отключаем PWM
      analogWrite(PUMP_PWM_PIN, 0);
      currentPWM = 0;

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
  wdt_reset();
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