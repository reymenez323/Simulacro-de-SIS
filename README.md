# SIS – Control de temperatura, puerta y ventilador (ESP32 + FreeRTOS)

Proyecto de **Sistema Instrumentado de Seguridad (SIS)**: un sensor de
temperatura enciende un ventilador por PWM, y si la temperatura llega a un
nivel crítico, detiene la "máquina" (proceso simulado) y lleva el
ventilador a velocidad máxima. Además incluye un segundo enclavamiento de
seguridad con un limit switch de puerta (contactos NO + NC) y arranque por
pulsador.

## Concepto: arquitectura tipo SIS

En un Sistema Instrumentado de Seguridad real se distinguen 3 elementos:
**sensor (transmisor) → lógica → elemento final**, organizados en dos
capas de protección independientes:

- **Capa 1 (BPCS, control normal):** al superar `SP_HIGH` (32 °C), el
  ventilador enciende a velocidad moderada para intentar enfriar. Esto
  es control normal, no una parada.
- **Capa 2 (SIS, protección de seguridad):** si la temperatura sigue
  subiendo hasta `SP_HIGH_HIGH` (40 °C), el ventilador pasa a 100% y se
  detiene ("trip") la ejecución de la máquina.

Hay un **segundo enclavamiento independiente**: la puerta con su limit
switch NO+NC. Usar ambos contactos da cobertura de diagnóstico (detecta
fallas de cableado/sensor), igual que la redundancia en un SIS real.

Tras cualquier parada (temperatura o puerta), el proceso **no se
reinicia solo**: se exige un reset manual y solo se acepta si todas las
condiciones son seguras. Esto replica el requisito real de "no reinicio
automático tras un disparo de seguridad" (IEC 61511).

## Requisitos

- [PlatformIO](https://platformio.org/) (extensión de VS Code o CLI)
- Placa ESP32 (DevKit genérica, `esp32dev`)
- Termistor NTC 10K (B=3950) + resistencia de 10 kΩ — bulbo pequeño,
  responde rápido: sube de temperatura soplando aire caliente (secador
  a distancia, ~35-45 °C) o frotándolo entre los dedos (fricción + calor
  corporal, +5-10 °C sobre ambiente)
- Transistor NPN 2N2222A (como interruptor de baja lateral) + resistencia
  de base ~1 kΩ, resistencia de 10 kΩ y diodo 1N4007
- Ventilador DC 5V, corriente ≤ ~150-200 mA (ver nota de límite de
  corriente del 2N2222A más abajo)
- 3 LEDs (verde, amarillo, rojo) + resistencias limitadoras (~220-330 Ω)
- Pulsador momentáneo (inicio/reset)
- Limit switch con contactos NO y NC (para la puerta)

## Compilar / cargar

El PWM del ventilador usa la librería [ESP32Servo](https://github.com/madhephaestus/ESP32Servo)
(clase `ESP32PWM`) en vez de llamar directamente a la API LEDC. PlatformIO
la descarga solo, según `lib_deps` en `platformio.ini`.

```bash
cd sis_ventilador_esp32
pio run              # compila (descarga ESP32Servo la primera vez)
pio run -t upload    # compila y sube al ESP32
pio device monitor    # abre el monitor serial (115200 baud)
```

## Estados del sistema

| Estado    | LED verde | LED amarillo        | LED rojo | Ventilador          | Máquina  |
|-----------|-----------|----------------------|----------|----------------------|----------|
| IDLE      | apagado   | según temperatura*    | apagado  | según temperatura*   | detenida |
| RUNNING   | fijo      | parpadea si T≥32°C    | apagado  | ON si T≥32°C, MAX si T≥40°C | corriendo |
| STOPPED   | apagado   | parpadea hasta T≤27°C | fijo     | MAX si sigue caliente | detenida |

\* El ventilador y el LED amarillo responden a la temperatura de forma
independiente del estado de arranque de la máquina (protección térmica
siempre activa).

**Setpoints** (ajustables en `src/main.cpp`):

- `SP_HIGH = 32.0 °C` → arranca ventilador a velocidad moderada (~51%) y
  empieza a parpadear el LED amarillo.
- `SP_HIGH_HIGH = 40.0 °C` → disparo del SIS: ventilador a 100% y la
  máquina se detiene.
- `TEMP_AMBIENT = 27.0 °C` → umbral para que el LED amarillo deje de
  parpadear y se habilite el reset.

Estos valores son alcanzables soplando aire caliente (secador a
distancia) o frotando el sensor entre los dedos.

## Lógica del pulsador (inicio / reset)

Un mismo pulsador (GPIO21, a GND, con resistencia de pull-up externa)
cumple dos funciones:

1. **En IDLE**: arranca el sistema (BIT_RUNNING), solo si la puerta está
   cerrada.
2. **En STOPPED** (por temperatura y/o puerta abierta): resetea y reanuda
   la máquina, solo si la puerta está cerrada **y** la temperatura ya
   volvió al ambiente (≤ 27 °C). Si alguna condición falta, la pulsación
   se ignora y el monitor serial indica cuál falta.

Esto implementa el principio de **no reinicio automático tras una parada
de seguridad** (se requiere una acción deliberada del operador, con
condiciones seguras verificadas).

## Enclavamiento de puerta (limit switch NO + NC)

La máquina **no puede arrancar ni seguir corriendo con la puerta
abierta**. Se usa un limit switch con dos contactos para tener
**cobertura de diagnóstico**: en operación normal, NO y NC siempre deben
leer valores opuestos. Si ambos leen lo mismo, es una falla de cableado o
del sensor, y el sistema lo trata como "puerta no confirmada cerrada"
(fail-safe).

Conexionado:

```
        3V3
         |
        NTC
         |
GPIO34 --+-- R 10k -- GND      (sensor de temperatura)

GPIO25 --1k-- Base (2N2222A)
Base --10k-- GND (pull-down, evita encendidos falsos al arrancar)
Colector -> terminal (-) del ventilador
Emisor -> GND común (ESP32 y fuente del ventilador)
Fuente externa (+) -> terminal (+) del ventilador
Diodo flyback (1N4007, cátodo al +) en antiparalelo sobre el ventilador

COM(switch) -- GND
NO(switch)  -- GPIO32 (pull-up interno)
NC(switch)  -- GPIO33 (pull-up interno)

Pulsador -- GPIO21 -- GND (pull-up externo)

LED verde    -- GPIO26 -- R 330 -- GND
LED amarillo -- GPIO4  -- R 330 -- GND
LED rojo     -- GPIO14 -- R 330 -- GND
```

Si tu limit switch queda presionado con la puerta **abierta** en vez de
cerrada, cambia `#define DOOR_LOGIC_INVERTIDA 1` en `src/main.cpp` en
lugar de recablear.

### Límite de corriente del 2N2222A

El 2N2222A es un transistor de señal, no de potencia: soporta hasta
~600 mA de colector en el mejor de los casos (y bastante menos de forma
continua sin disipador). Con `Rb = 1 kΩ` la corriente de base es
~2.6 mA, suficiente para saturarlo (hFE ≥ 100) hasta ~200-260 mA de
colector con margen seguro. Verifica la corriente nominal de tu
ventilador antes de conectarlo:

- Ventilador de 5V pequeño (≤150-200 mA): funciona bien con el 2N2222A.
- Ventilador que consuma más de ~300 mA, o de 12V: el 2N2222A se
  calentará o saturará mal. En ese caso conviene un transistor de mayor
  corriente (TIP31, BD139) o un MOSFET, y bajar `Rb` si el hFE del
  transistor usado es menor.

### Pines evitados a propósito (ESP32-WROOM-32)

Ningún pin de este proyecto cae en las categorías riesgosas del
WROOM-32:

- **GPIO6-11**: conectados internamente a la memoria flash SPI. Usarlos
  para cualquier otra cosa cuelga el arranque del chip. **Nunca usar.**
- **GPIO0, GPIO2, GPIO5, GPIO12, GPIO15**: pines de *strapping* que el
  ESP32 lee al arrancar para decidir el modo de boot (y en el caso de
  GPIO12, el voltaje de la flash). Un LED o resistencia externa en estos
  pines puede impedir que la placa arranque o la meta en modo descarga.
  Se evitaron aunque GPIO2 suele traer el LED de la placa de fábrica.
- **GPIO1, GPIO3**: son el UART0, usado por el cable USB para programar
  y ver el monitor serial. Usarlos como GPIO normal rompe la
  comunicación con la PC.
- **GPIO34-39**: son solo de entrada (no se pueden usar como salida ni
  tienen pull-up/down interno). Por eso el sensor de temperatura, que
  solo necesita leer, va en GPIO34.

Todos los pines usados (4, 14, 21, 25, 26, 32, 33, 34) son GPIO de
propósito general sin ninguna de estas restricciones.

## Arquitectura FreeRTOS

Seis tareas independientes, comunicadas por una `Queue` (temperatura) y
un `EventGroup` (estado del sistema: `BIT_RUNNING`, `BIT_STOPPED`,
`BIT_DOOR_OPEN`):

- **TaskSensor** – lee el NTC y publica una temperatura filtrada (media
  móvil de 8 muestras).
- **TaskDoor** – lee el limit switch (NO+NC), detecta fallas de
  cableado, y detiene la máquina si la puerta se abre durante RUNNING.
- **TaskLogic** – compara la temperatura contra los setpoints y dispara
  la parada de seguridad al llegar a `SP_HIGH_HIGH`.
- **TaskActuators** – controla el PWM del ventilador y los 3 LEDs
  (incluye el parpadeo no bloqueante del LED amarillo).
- **TaskMachine** – simula el proceso protegido; arranca suspendida y
  solo se reanuda por inicio/reset manual.
- **TaskStartReset** – gestiona el pulsador físico de inicio/reset,
  validando todas las condiciones de seguridad antes de reanudar.

## Prueba física sugerida

1. Alimenta el ESP32 y abre el monitor serial.
2. Con la puerta abierta, confirma que el pulsador rechaza el arranque.
3. Cierra la puerta y presiona el pulsador → LED verde fijo, "MAQUINA"
   imprime ciclos en el monitor.
4. Sopla aire caliente (o frota el sensor) hasta pasar 32 °C → LED
   amarillo empieza a parpadear, ventilador arranca a velocidad
   moderada.
5. Sigue calentando hasta pasar 40 °C → ventilador a máxima velocidad,
   LED rojo fijo, la máquina se detiene (deja de imprimir ciclos).
6. Abre la puerta mientras está corriendo (en otra prueba, sin
   calentar) → confirma que también detiene la máquina inmediatamente.
7. Deja enfriar el sensor por debajo de 27 °C, cierra la puerta, y
   presiona el pulsador → el sistema se reanuda.
