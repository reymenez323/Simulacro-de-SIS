// SIS - Control de temperatura, puerta y ventilador (ESP32 + FreeRTOS)
// Ver README.md para la arquitectura completa, el conexionado y la
// justificación de los pines usados.

#include <Arduino.h>
#include <ESP32PWM.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#define USE_NTC 1              // 1 = termistor NTC 10K, 0 = LM35
#define DOOR_LOGIC_INVERTIDA 0 // 1 si el switch cierra contacto con la puerta abierta

// Pines (ver README.md: se evitan a propósito flash SPI, strapping y UART0)
static const int PIN_SENSOR         = 34;
static const int PIN_FAN_PWM        = 25;
static const int PIN_LED_OK         = 26;
static const int PIN_LED_HIGH       = 4;
static const int PIN_LED_TRIP       = 14;
static const int PIN_START_RESET_BT = 21; // pull-up externo ya conectado
static const int PIN_DOOR_NO        = 32;
static const int PIN_DOOR_NC        = 33;

// Parámetros del termistor NTC 10K (curva Beta)
static const float NTC_NOMINAL_RES  = 10000.0f;
static const float NTC_NOMINAL_TEMP = 25.0f;
static const float NTC_BETA         = 3950.0f;
static const float SERIES_RESISTOR  = 1020.0f; // calibrado con ADC en reposo ~3716 = 25 C
static const float ADC_MAX          = 4095.0f;

// Setpoints del SIS (°C)
static const float SP_HIGH      = 39.0f; // enciende ventilador / alarma amarilla
static const float SP_HIGH_HIGH = 35.0f; // disparo: para la máquina
static const float TEMP_AMBIENT = 30.0f; // umbral para apagar alarma y permitir reset

// PWM del ventilador (librería ESP32Servo / clase ESP32PWM)
static const int PWM_FREQ_HZ = 5000;
static const float DUTY_OFF  = 0.0f;
static const float DUTY_HIGH = 0.51f; // ~51% velocidad moderada
static const float DUTY_MAX  = 1.0f;  // 100% velocidad máxima

static const TickType_t BLINK_PERIOD = pdMS_TO_TICKS(300);

static QueueHandle_t tempQueue;
static EventGroupHandle_t sisEvents;
static const EventBits_t BIT_RUNNING   = BIT0; // sistema iniciado
static const EventBits_t BIT_STOPPED   = BIT1; // máquina detenida (temp y/o puerta)
static const EventBits_t BIT_DOOR_OPEN = BIT2; // puerta abierta o sensor con falla

static TaskHandle_t machineTaskHandle = NULL;
static ESP32PWM fanPwm;

float leerTemperaturaC() {
    int adc = analogRead(PIN_SENSOR);
    Serial.printf("[DEBUG] ADC crudo en GPIO%d = %d\n", PIN_SENSOR, adc);

#if USE_NTC
    // Ecuación de Beta: ADC -> resistencia -> temperatura
    // Módulo KY-013: R fija (10k) de VCC a S, NTC de S a GND
    float resistencia = SERIES_RESISTOR * ((float)adc / (ADC_MAX - (float)adc));
    float steinhart = resistencia / NTC_NOMINAL_RES;
    steinhart = log(steinhart);
    steinhart /= NTC_BETA;
    steinhart += 1.0f / (NTC_NOMINAL_TEMP + 273.15f);
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f;
    return steinhart;
#else
    float voltaje = (adc / ADC_MAX) * 3.3f; // LM35: 10 mV/°C
    return voltaje * 100.0f;
#endif
}

// Puerta confirmada cerrada solo si NO y NC leen valores opuestos;
// si coinciden hay una falla de cableado/sensor (se reporta en "fallo").
bool leerPuertaCerrada(bool &fallo) {
    bool no = digitalRead(PIN_DOOR_NO);
    bool nc = digitalRead(PIN_DOOR_NC);

    fallo = (no == nc);

#if DOOR_LOGIC_INVERTIDA
    bool cerrada = (no == HIGH && nc == LOW);
#else
    bool cerrada = (no == LOW && nc == HIGH);
#endif

    return (!fallo) && cerrada;
}

void TaskSensor(void *pvParameters) {
    const TickType_t periodo = pdMS_TO_TICKS(300);

    // Media móvil para estabilizar la lectura del ADC
    const int N = 8;
    float buffer[N] = {0};
    int idx = 0;
    bool bufferLleno = false;

    for (;;) {
        float lectura = leerTemperaturaC();
        buffer[idx] = lectura;
        idx = (idx + 1) % N;
        if (idx == 0) bufferLleno = true;

        int cuenta = bufferLleno ? N : idx;
        float suma = 0;
        for (int i = 0; i < cuenta; i++) suma += buffer[i];
        float promedio = suma / cuenta;

        xQueueOverwrite(tempQueue, &promedio);
        vTaskDelay(periodo);
    }
}

void TaskDoor(void *pvParameters) {
    const TickType_t periodo = pdMS_TO_TICKS(150);
    bool puertaCerradaAnterior = false; // fail-safe: se asume abierta al inicio

    for (;;) {
        bool fallo = false;
        bool cerrada = leerPuertaCerrada(fallo);

        if (cerrada != puertaCerradaAnterior) {
            if (cerrada) {
                Serial.println("[PUERTA] Cerrada.");
            } else if (fallo) {
                Serial.println("[PUERTA] FALLO: contactos NO/NC inconsistentes. Estado inseguro.");
            } else {
                Serial.println("[PUERTA] Abierta.");
            }
            puertaCerradaAnterior = cerrada;
        }

        if (cerrada) {
            xEventGroupClearBits(sisEvents, BIT_DOOR_OPEN);
        } else {
            EventBits_t estadoPrevio = xEventGroupGetBits(sisEvents);
            xEventGroupSetBits(sisEvents, BIT_DOOR_OPEN);

            bool yaEstabaAbierta = (estadoPrevio & BIT_DOOR_OPEN) != 0;
            bool corriendo       = (estadoPrevio & BIT_RUNNING) != 0;
            bool detenida        = (estadoPrevio & BIT_STOPPED) != 0;

            // Interrumpe la máquina de inmediato si la puerta se abre en marcha
            if (!yaEstabaAbierta && corriendo && !detenida) {
                Serial.println("[SIS] Puerta abierta durante la ejecucion -> deteniendo maquina.");
                xEventGroupSetBits(sisEvents, BIT_STOPPED);
                if (machineTaskHandle != NULL) {
                    vTaskSuspend(machineTaskHandle);
                }
            }
        }

        vTaskDelay(periodo);
    }
}

void TaskLogic(void *pvParameters) {
    float temp = 25.0f;
    const TickType_t periodo = pdMS_TO_TICKS(200);

    for (;;) {
        if (xQueuePeek(tempQueue, &temp, 0) != pdTRUE) {
            vTaskDelay(periodo);
            continue;
        }

        EventBits_t estado = xEventGroupGetBits(sisEvents);
        bool corriendo = (estado & BIT_RUNNING) != 0;
        bool detenida  = (estado & BIT_STOPPED) != 0;

        Serial.printf("[LOGIC] T=%.1f C | %s%s\n",
                       temp,
                       corriendo ? "RUNNING" : "IDLE",
                       detenida ? " | DETENIDA" : "");

        // Disparo del SIS: temperatura crítica -> detiene la máquina
        if (corriendo && !detenida && temp >= SP_HIGH_HIGH) {
            Serial.println("[SIS] !! TEMPERATURA CRITICA -> DISPARO !!");
            xEventGroupSetBits(sisEvents, BIT_STOPPED);
            if (machineTaskHandle != NULL) {
                vTaskSuspend(machineTaskHandle);
            }
        }

        vTaskDelay(periodo);
    }
}

void TaskActuators(void *pvParameters) {
    float temp = 25.0f;
    const TickType_t periodo = pdMS_TO_TICKS(100);
    bool alarmaAlta = false; // latch: se activa en SP_HIGH, se libera en TEMP_AMBIENT
    float dutyActual = 0.0f;
    const float RAMP_STEP = 0.15f; // por tick (100 ms): 0->100% en ~0.7 s, evita pico de arranque

    for (;;) {
        xQueuePeek(tempQueue, &temp, 0);
        EventBits_t estado = xEventGroupGetBits(sisEvents);
        bool corriendo = (estado & BIT_RUNNING) != 0;
        bool detenida  = (estado & BIT_STOPPED) != 0;

        if (!alarmaAlta && temp >= SP_HIGH) {
            alarmaAlta = true;
        } else if (alarmaAlta && temp <= TEMP_AMBIENT) {
            alarmaAlta = false;
        }

        // Ventilador: responde solo a la temperatura, sin importar el estado.
        // Se rampea el duty en vez de saltar de golpe, para suavizar el pico
        // de corriente de arranque del motor.
        float dutyObjetivo;
        if (temp >= SP_HIGH_HIGH) {
            dutyObjetivo = DUTY_MAX;
        } else if (temp >= SP_HIGH) {
            dutyObjetivo = DUTY_HIGH;
        } else {
            dutyObjetivo = DUTY_OFF;
        }

        if (dutyActual < dutyObjetivo) {
            dutyActual = min(dutyObjetivo, dutyActual + RAMP_STEP);
        } else if (dutyActual > dutyObjetivo) {
            dutyActual = max(dutyObjetivo, dutyActual - RAMP_STEP);
        }
        fanPwm.writeScaled(dutyActual);

        digitalWrite(PIN_LED_OK, (corriendo && !detenida) ? HIGH : LOW);

        if (alarmaAlta) {
            bool fase = ((xTaskGetTickCount() / BLINK_PERIOD) % 2) == 0;
            digitalWrite(PIN_LED_HIGH, fase ? HIGH : LOW);
        } else {
            digitalWrite(PIN_LED_HIGH, LOW);
        }

        digitalWrite(PIN_LED_TRIP, detenida ? HIGH : LOW);

        vTaskDelay(periodo);
    }
}

void TaskMachine(void *pvParameters) {
    vTaskSuspend(NULL); // arranca detenida, esperando el pulsador de inicio

    uint32_t ciclo = 0;
    for (;;) {
        ciclo++;
        Serial.printf("[MAQUINA] Ejecutando ciclo de proceso #%lu\n",
                       (unsigned long)ciclo);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TaskStartReset(void *pvParameters) {
    const TickType_t periodo = pdMS_TO_TICKS(100);
    bool ultimoEstadoBoton = HIGH;

    for (;;) {
        bool botonActual = digitalRead(PIN_START_RESET_BT);

        if (ultimoEstadoBoton == HIGH && botonActual == LOW) { // flanco de bajada
            EventBits_t estado = xEventGroupGetBits(sisEvents);
            bool corriendo     = (estado & BIT_RUNNING) != 0;
            bool detenida      = (estado & BIT_STOPPED) != 0;
            bool puertaAbierta = (estado & BIT_DOOR_OPEN) != 0;

            float temp = 25.0f;
            xQueuePeek(tempQueue, &temp, 0);

            if (!corriendo) {
                if (puertaAbierta) {
                    Serial.println("[START] Rechazado: cierra la puerta primero.");
                } else {
                    Serial.println("[START] Iniciando el sistema.");
                    xEventGroupSetBits(sisEvents, BIT_RUNNING);
                    if (machineTaskHandle != NULL) {
                        vTaskResume(machineTaskHandle);
                    }
                }
            } else if (detenida) {
                // Reset manual: exige puerta cerrada y, si aplica, temp segura
                if (puertaAbierta) {
                    Serial.println("[RESET] Rechazado: la puerta debe estar cerrada.");
                } else if (temp > TEMP_AMBIENT) {
                    Serial.printf("[RESET] Rechazado: temperatura aun alta (%.1f C).\n", temp);
                } else {
                    Serial.println("[RESET] Condiciones seguras. Reanudando maquina.");
                    xEventGroupClearBits(sisEvents, BIT_STOPPED);
                    if (machineTaskHandle != NULL) {
                        vTaskResume(machineTaskHandle);
                    }
                }
            }
        }

        ultimoEstadoBoton = botonActual;
        vTaskDelay(periodo);
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== SIS - Control de temperatura, puerta y ventilador (ESP32) ===");

    pinMode(PIN_LED_OK, OUTPUT);
    pinMode(PIN_LED_HIGH, OUTPUT);
    pinMode(PIN_LED_TRIP, OUTPUT);
    pinMode(PIN_START_RESET_BT, INPUT); // pull-up externo en el pulsador
    pinMode(PIN_DOOR_NO, INPUT_PULLUP);
    pinMode(PIN_DOOR_NC, INPUT_PULLUP);

    digitalWrite(PIN_LED_OK, LOW);
    digitalWrite(PIN_LED_HIGH, LOW);
    digitalWrite(PIN_LED_TRIP, LOW);

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_SENSOR, ADC_11db);

    ESP32PWM::allocateTimer(0);
    fanPwm.attachPin(PIN_FAN_PWM, PWM_FREQ_HZ);
    fanPwm.writeScaled(DUTY_OFF);

    tempQueue = xQueueCreate(1, sizeof(float));
    sisEvents = xEventGroupCreate();

    // Fail-safe: se asume la puerta abierta hasta que TaskDoor confirme lo contrario
    xEventGroupSetBits(sisEvents, BIT_DOOR_OPEN);

    float inicial = 25.0f;
    xQueueOverwrite(tempQueue, &inicial);

    xTaskCreatePinnedToCore(TaskSensor,     "TaskSensor",     4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskDoor,       "TaskDoor",       4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(TaskLogic,      "TaskLogic",      4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(TaskActuators,  "TaskActuators",  2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(TaskMachine,    "TaskMachine",    2048, NULL, 1, &machineTaskHandle, 1);
    xTaskCreatePinnedToCore(TaskStartReset, "TaskStartReset", 2048, NULL, 2, NULL, 1);

    Serial.println("Tareas FreeRTOS iniciadas. Sistema en espera (IDLE).");
    Serial.println("Cierra la puerta y presiona el pulsador para iniciar.");
    Serial.printf("Setpoints -> HIGH: %.1f C | HIGH-HIGH (trip): %.1f C | Ambiente: %.1f C\n",
                   SP_HIGH, SP_HIGH_HIGH, TEMP_AMBIENT);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000)); // todo el trabajo ocurre en las tareas
}
