#include <Arduino.h>
#include <avr/wdt.h>

// Пины
const uint8_t PRESSURE_PIN = A0;    // Датчик давления
const uint8_t PUMP_PWM_PIN = 9;     // MOSFET насоса (PWM)
const uint8_t SAFETY_RELAY_PIN = 8; // MOSFET защитного реле

// Желаемый вакуум дефолтное значение
const float TARGET_VACUUM_KPA = 25.0;

// Гистерезис
const float PUMP_ON_DELTA = 5.0;  // Включение ниже цели
const float PUMP_OFF_DELTA = 2.0; // Выключение выше цели

// таймеры и задержки
const unsigned long RESTART_DELAY_MS = 15000; // задержка повторного включения
const unsigned long MAX_RUN_TIME_MS = 60000;  // максимум 1 минута непрерывной работы
const unsigned long STARTUP_DELAY_MS = 3000;  // задержка включения реле после старта

// АЦП
const float ADC_REFERENCE_VOLTAGE = 5.0;
const float SENSOR_MIN_VOLT = 0.5;
const float SENSOR_MAX_VOLT = 4.5;

// Аварийные границы датчика давления (УТОЧНИ)
const float SENSOR_ERROR_LOW = 0.4;
const float SENSOR_ERROR_HIGH = 4.6;

// сколько сэмплов брать для фильтрации сигнала (задержку вывести сюда надо бы)
const uint8_t FILTER_SAMPLES = 20;

// скважность PWM (две скорости насоса)
const uint8_t PWM_FULL = 255; // 100%
const uint8_t PWM_11V = 234;  // ~11В при 12В питании

// переменные

bool pumpState = false;
bool relayEnabled = false;

unsigned long lastPumpStopTime = 0;
unsigned long pumpStartTime = 0;

//  аварийное выкл

void emergencyStop(const char *message)
{
    // авайрийно выключаем насос
    analogWrite(PUMP_PWM_PIN, 0);
    pumpState = false;

    // отключаем защитное реле
    digitalWrite(SAFETY_RELAY_PIN, LOW);
    relayEnabled = false;

    Serial.println("!!! ERROR !!!");
    Serial.println(message);

    wdt_disable();

    while (true)
    {
        // HALTED Полная остановка
    }
}

// считывание давления

float readPressureKPa()
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < FILTER_SAMPLES; i++)
    {
        sum += analogRead(PRESSURE_PIN);
        delay(5);
    }

    float adcValue = sum / (float)FILTER_SAMPLES;
    float voltage = adcValue * ADC_REFERENCE_VOLTAGE / 1023.0;

    if (voltage < SENSOR_ERROR_LOW || voltage > SENSOR_ERROR_HIGH)
    {
        emergencyStop("Sensor voltage out of range");
    }

    float pressureKPa = (voltage - SENSOR_MIN_VOLT) *
                        (100.0 / (SENSOR_MAX_VOLT - SENSOR_MIN_VOLT));

    if (pressureKPa < 0)
        pressureKPa = 0;
    if (pressureKPa > 100)
        pressureKPa = 100;

    return pressureKPa;
}

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

// Управление скоростью насоса через pwm

void setPumpSpeed(float currentVacuum)
{
    float halfTarget = TARGET_VACUUM_KPA / 2.0;

    if (currentVacuum < halfTarget)
    {
        analogWrite(PUMP_PWM_PIN, PWM_FULL);
    }
    else
    {
        analogWrite(PUMP_PWM_PIN, PWM_11V);
    }
}

// управление насосом вкл выкл
void controlPump(float currentVacuum)
{
    float pumpOnThreshold = TARGET_VACUUM_KPA - PUMP_ON_DELTA;
    float pumpOffThreshold = TARGET_VACUUM_KPA + PUMP_OFF_DELTA;

    unsigned long now = millis();

    // проверка времени непрерывной работы
    if (pumpState && (now - pumpStartTime) >= MAX_RUN_TIME_MS)
    {
        emergencyStop("Pump exceeded maximum continuous runtime");
    }

    if (!pumpState)
    {
        if (currentVacuum <= pumpOnThreshold &&
            (now - lastPumpStopTime) >= RESTART_DELAY_MS)
        {
            if (!relayEnabled)
            {
                enableRelay();
                delay(100); // пауза для стабилизации контактов
            }

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

            // отключаем PWM
            analogWrite(PUMP_PWM_PIN, 0);

            // отключаем реле
            disableRelay();

            Serial.println("Pump OFF");
        }
    }

    if (pumpState)
    {
        setPumpSpeed(currentVacuum);
    }
}

// сетуп

void setup()
{
    Serial.begin(115200);

    pinMode(PUMP_PWM_PIN, OUTPUT);
    pinMode(SAFETY_RELAY_PIN, OUTPUT);

    analogWrite(PUMP_PWM_PIN, 0);
    digitalWrite(SAFETY_RELAY_PIN, LOW);

    // Watchdog 2 сек
    wdt_enable(WDTO_2S);

    Serial.println("System booting...");
    delay(STARTUP_DELAY_MS); // 3 сек пауза

    Serial.println("Startup delay complete. System ready.");
}

void loop()
{
    wdt_reset();

    float vacuumKPa = readPressureKPa();

    Serial.print("Vacuum: ");
    Serial.print(vacuumKPa);
    Serial.println(" kPa");

    controlPump(vacuumKPa);

    delay(200);
}