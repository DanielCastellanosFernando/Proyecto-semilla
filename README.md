# Proyecto-semilla
Este proyecto implementa un sistema de monitoreo ambiental basado en ESP32, integrando un sensor de humedad de suelo, un sensor de lluvia y un módulo MPU6050. La información de los sensores se visualiza en una pantalla OLED mediante una interfaz de menús interactiva. El sistema permite supervisar en tiempo real las condiciones del entorno de forma sencilla y eficiente.




Enlace del archivo .rar de Altium
https://drive.google.com/drive/folders/1Iz4U5BRyFNcIZlU8DwyHgjQNg9sfKK7b?usp=drive_link


Fotos del proceso y de altium


<img width="899" height="1599" alt="image" src="https://github.com/user-attachments/assets/e002e42c-2f3c-4c8b-bdd9-b15b5d8b8af3" />
<img width="655" height="1600" alt="WhatsApp Image 2026-05-27 at 6 20 46 PM" src="https://github.com/user-attachments/assets/d6dc0cab-2165-41c7-9932-6774e5f7bc39" />
<img width="655" height="1600" alt="WhatsApp Image 2026-05-27 at 6 20 46 PM (1)" src="https://github.com/user-attachments/assets/5b3cd143-46b2-4604-911a-bfd62853cbd0" />
<img width="518" height="783" alt="WhatsApp Image 2026-05-31 at 1 14 16 PM" src="https://github.com/user-attachments/assets/a2891ab6-665c-46c3-9222-e9ed1f54c9aa" />
<img width="490" height="782" alt="image" src="https://github.com/user-attachments/assets/aa30d8b0-ead8-497d-93b5-2ab345d7c543" />
<img width="486" height="781" alt="image" src="https://github.com/user-attachments/assets/4d59adb9-9794-4934-b25d-e3b3081a00d0" />
<img width="535" height="822" alt="image" src="https://github.com/user-attachments/assets/989cbe34-e787-45cc-aa20-4d23fe8907cd" />
<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/576e9966-d63d-41ff-ae22-fd01afe28a14" />
<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/d57ff7dd-2724-47b7-8d2b-2751c4592d2c" />








Programa utilizado 


/*
 * =============================================================================
 * FIRMWARE SEMILLA v1.1
 * Sistema de Medición de Humedad e Infiltración de Suelo
 * Plataforma: ESP32 en Arduino IDE
 * =============================================================================
 *
 * DIAGRAMA FSM — ESTADOS Y TRANSICIONES:
 *
 *  [MENU_PRINCIPAL]
 *    ├─ SELECT (índ. 0) ──────────────────────► [CAMPO_INSTRUCCIONES]
 *    └─ SELECT (índ. 1) ──────────────────────► [LAB_MENU]
 *
 *  [CAMPO_INSTRUCCIONES]
 *    ├─ SELECT ───────────────────────────────► [CAMPO_ESPERANDO_DESPLAZAMIENTO]
 *    └─ BACK ─────────────────────────────────► [MENU_PRINCIPAL]
 *
 *  [CAMPO_ESPERANDO_DESPLAZAMIENTO]
 *    ├─ distancia >= 1 m ─────────────────────► [CAMPO_MIDIENDO]
 *    └─ BACK ─────────────────────────────────► [MENU_PRINCIPAL]
 *
 *  [CAMPO_MIDIENDO]
 *    └─ 25 lecturas listas ───────────────────► [CAMPO_PUNTO_COMPLETO]
 *
 *  [CAMPO_PUNTO_COMPLETO]
 *    ├─ puntos < 10 + SELECT ─────────────────► [CAMPO_ESPERANDO_DESPLAZAMIENTO]
 *    ├─ puntos == 10 + SELECT ────────────────► [CAMPO_RESULTADO]
 *    └─ BACK ─────────────────────────────────► [MENU_PRINCIPAL]
 *
 *  [CAMPO_RESULTADO]
 *    ├─ SELECT (Reiniciar) ───────────────────► [CAMPO_INSTRUCCIONES]
 *    └─ BACK ─────────────────────────────────► [MENU_PRINCIPAL]
 *
 *  [LAB_MENU]
 *    ├─ SELECT (índ. 0) ──────────────────────► [LAB_HUMEDAD_MIDIENDO]
 *    ├─ SELECT (índ. 1) ──────────────────────► [LAB_INFILTRACION_ACTIVA]
 *    └─ BACK ─────────────────────────────────► [MENU_PRINCIPAL]
 *
 *  [LAB_HUMEDAD_MIDIENDO]
 *    ├─ SELECT (Reiniciar) ───────────────────► [LAB_HUMEDAD_MIDIENDO]
 *    └─ BACK ─────────────────────────────────► [LAB_MENU]
 *
 *  [LAB_INFILTRACION_ACTIVA]
 *    ├─ 5 min transcurridos ──────────────────► [LAB_INFILTRACION_RESULTADO]
 *    └─ BACK ─────────────────────────────────► [LAB_MENU]
 *
 *  [LAB_INFILTRACION_RESULTADO]
 *    ├─ SELECT (Reiniciar) ───────────────────► [LAB_INFILTRACION_ACTIVA]
 *    └─ BACK ─────────────────────────────────► [LAB_MENU]
 *
 * =============================================================================
 */

#include <U8g2lib.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_MCP23X17.h>

// =============================================================================
// SECCIÓN 1 — CONSTANTES DE HARDWARE
// =============================================================================

// --- Pines ESP32 (sólo analógicos directos; I²C se maneja por Wire) ---
#define SENSOR_HUMEDAD_PIN    34   // YL-69 salida analógica
#define RAINDROPS_ANALOG_PIN  35   // Sensor lluvia: salida analógica
#define RAINDROPS_DIGITAL_PIN 27   // Sensor lluvia: salida digital (LOW = agua)

// --- Pines del MCP23017 (numeración 0-15) ---
#define MCP_BTN_UP      4    // GPA0 — Botón 1: navegar arriba
#define MCP_BTN_DOWN    5    // GPA1 — Botón 2: navegar abajo
#define MCP_BTN_SELECT  6    // GPA2 — Botón 3: confirmar/seleccionar
#define MCP_BTN_BACK    7    // GPA3 — Botón 4: regresar/cancelar
#define MCP_LED_ESTADO  8    // GPB0 — LED de estado

// Índices para el array de debounce (mapeo 1:1 con pines MCP de botones)
#define IDX_BTN_UP     4
#define IDX_BTN_DOWN   5
#define IDX_BTN_SELECT 6
#define IDX_BTN_BACK   7

// --- Calibración sensor de humedad YL-69 (ADC 12-bit) ---
#define HUMEDAD_SECO   3500   // Valor ADC en suelo seco (0%)
#define HUMEDAD_MOJADO 1500   // Valor ADC en suelo saturado (100%)

// --- Calibración sensor de lluvia para infiltración ---
// NOTA DE CALIBRACIÓN: Este valor convierte cada detección digital (flanco
// descendente en el pin digital) a milímetros de agua infiltrada. Para calibrar:
// 1) Verter un volumen conocido de agua (ej. 10 ml en un cilindro de 50 cm²).
// 2) Contar los flancos detectados durante el vertido.
// 3) Calcular mm = volumen_ml / area_cm² / detecciones y asignar aquí.
#define MM_POR_DETECCION_LLUVIA  0.10f

// --- Configuración Modo Campo ---
#define CAMPO_NUM_PUNTOS     10     // Puntos de medición totales
#define CAMPO_LECTURAS_PUNTO 25     // Lecturas YL-69 por punto
#define CAMPO_DISTANCIA_META 1.0f   // Metros requeridos entre puntos

// --- Configuración Modo Laboratorio ---
#define LAB_INFILTRACION_DURACION_MS  300000UL  // 5 minutos en ms
#define LAB_HUMEDAD_NUM_LECTURAS       20        // Lecturas para promedio en lab

// --- Parámetros del dead reckoning MPU6050 ---
#define MPU_INTERVALO_MS   50      // Intervalo entre integraciones (ms)
#define MPU_BIAS_MUESTRAS 100      // Muestras para calibración de bias
// UMBRAL DE RUIDO: El MPU6050 produce ~0.2–0.4 m/s² de ruido en reposo (normal).
// Valores bajo este umbral se descartan para evitar que el integrador derive
// cuando el sensor está quieto. Verificado con diagnóstico en hardware real.
#define MPU_UMBRAL_RUIDO  0.40f    // m/s² mínimos para considerar movimiento real

// Registros MPU6050 para reset/wake-up directo (sin depender de la librería)
#define MPU_ADDR         0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CFG    0x1C
#define REG_GYRO_CFG     0x1B

// =============================================================================
// SECCIÓN 2 — ENUMERACIÓN FSM
// =============================================================================

enum EstadoSistema {
  MENU_PRINCIPAL,
  CAMPO_INSTRUCCIONES,
  CAMPO_ESPERANDO_DESPLAZAMIENTO,
  CAMPO_MIDIENDO,
  CAMPO_PUNTO_COMPLETO,
  CAMPO_RESULTADO,
  LAB_MENU,
  LAB_HUMEDAD_MIDIENDO,
  LAB_INFILTRACION_ACTIVA,
  LAB_INFILTRACION_RESULTADO
};

// =============================================================================
// SECCIÓN 3 — OBJETOS DE HARDWARE
// =============================================================================

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
MPU6050 mpu;
Adafruit_MCP23X17 mcp;

// =============================================================================
// SECCIÓN 4 — VARIABLES DE ESTADO GLOBAL
// =============================================================================

EstadoSistema estadoActual = MENU_PRINCIPAL;

// --- Navegación de menús ---
int menuPrincipalIndex = 0;
int labMenuIndex       = 0;

// --- Modo Campo: datos de medición ---
float campo_mediciones[CAMPO_NUM_PUNTOS];
int   campo_puntosCompletados = 0;
int   campo_lecturasActuales  = 0;
long  campo_sumaLecturas      = 0;

// --- Dead reckoning MPU6050 ---
// LIMITACIONES conocidas del dead reckoning por integración de acelerómetro:
// - La doble integración acumula error rápidamente (drift numérico).
// - Vibraciones externas o inclinación constante generan falsos desplazamientos.
// - El enfoque funciona mejor con movimientos lineales en superficie plana.
// - No tiene precisión GPS: el objetivo es DETECTAR ~1 m, no medirlo exactamente.
// - El usuario puede ajustar CAMPO_DISTANCIA_META si el sistema dispara muy pronto
//   o muy tarde según las condiciones reales del terreno.
float dr_velX = 0, dr_velY = 0;       // Velocidad integrada (m/s)
float dr_posX = 0, dr_posY = 0;       // Posición integrada (m) desde último reset
float dr_biasAX = 0, dr_biasAY = 0;   // Bias del acelerómetro (m/s²), calibrado en reposo
unsigned long dr_ultimoTiempoMs = 0;   // Timestamp de la última integración

// --- Modo Lab — Humedad ---
float lab_humedadPorcentaje = 0;

// --- Modo Lab — Infiltración ---
unsigned long infil_tiempoInicio   = 0;
float         infil_mmAcumulados   = 0;
float         infil_mmPorMinuto    = 0;
bool          infil_estadoRainAnterior = true;  // true = seco (HIGH), para detectar flancos

// --- Debounce botones MCP23017 ---
unsigned long btn_ultimoTiempoMs[4] = {0, 0, 0, 0};
bool          btn_estadoAnterior[4] = {true, true, true, true};  // true = no presionado

// =============================================================================
// SECCIÓN 5 — PROTOTIPOS
// =============================================================================

// Hardware e inicialización
void inicializarHardware();
void mostrarErrorCritico(const char* linea1, const char* linea2);

// Entrada de botones
bool leerBoton(int indiceDebounce, int pinMCP);

// Sensor de humedad
int getHumedadPorcentaje();

// Dead reckoning
void calibrarBiasMPU();
void resetearDeadReckoning();
void actualizarDeadReckoning();
float getDistanciaRecorrida();

// Handlers FSM
void handleMenuPrincipal();
void handleCampoInstrucciones();
void handleCampoEsperandoDesplazamiento();
void handleCampoMidiendo();
void handleCampoPuntoCompleto();
void handleCampoResultado();
void handleLabMenu();
void handleLabHumedadMidiendo();
void handleLabInfiltracionActiva();
void handleLabInfiltracionResultado();

// Dibujo OLED
void dibujarBarraProgreso(int porcentaje, int x, int y, int w, int h);
void dibujarMenuPrincipal();
void dibujarCampoInstrucciones();
void dibujarCampoEsperando(float distancia);
void dibujarCampoMidiendo(int lecturas);
void dibujarCampoPuntoCompleto(int puntos, float promedioUltimo);
void dibujarCampoResultado(int puntos, float promedioFinal);
void dibujarLabMenu();
void dibujarLabHumedad(float porcentaje);
void dibujarInfiltracionActiva(unsigned long transcurridoMs, float mm);
void dibujarInfiltracionResultado(float mmTotal, float mmPorMin, unsigned long duracionMs);

// =============================================================================
// SECCIÓN 6 — SETUP
// =============================================================================

void setup() {
  inicializarHardware();
}

// =============================================================================
// SECCIÓN 7 — LOOP PRINCIPAL (dispatcher FSM)
// =============================================================================

void loop() {
  switch (estadoActual) {
    case MENU_PRINCIPAL:                   handleMenuPrincipal();                break;
    case CAMPO_INSTRUCCIONES:              handleCampoInstrucciones();           break;
    case CAMPO_ESPERANDO_DESPLAZAMIENTO:   handleCampoEsperandoDesplazamiento(); break;
    case CAMPO_MIDIENDO:                   handleCampoMidiendo();                break;
    case CAMPO_PUNTO_COMPLETO:             handleCampoPuntoCompleto();           break;
    case CAMPO_RESULTADO:                  handleCampoResultado();               break;
    case LAB_MENU:                         handleLabMenu();                      break;
    case LAB_HUMEDAD_MIDIENDO:             handleLabHumedadMidiendo();           break;
    case LAB_INFILTRACION_ACTIVA:          handleLabInfiltracionActiva();        break;
    case LAB_INFILTRACION_RESULTADO:       handleLabInfiltracionResultado();     break;
  }
}

// =============================================================================
// SECCIÓN 8 — INICIALIZACIÓN DE HARDWARE
// =============================================================================

// Inicializa todos los periféricos y detiene el sistema si alguno falla
void inicializarHardware() {
  Wire.begin(21, 22);
  analogReadResolution(12);

  // OLED primero para poder mostrar errores
  u8g2.begin();

  // MCP23017 — verificar presencia en bus I²C antes de configurar
  Wire.beginTransmission(0x20);
  if (Wire.endTransmission() != 0) {
    mostrarErrorCritico("MCP23017 ERROR", "Verifica I2C 0x20");
    while (true) { delay(1000); }
  }
  mcp.begin_I2C(0x20);  // Dirección completa I²C 0x20 (A0=A1=A2=GND) — API v2.x

  // Configurar botones: INPUT_PULLUP en una sola llamada (cambio de API en v2.x)
  mcp.pinMode(MCP_BTN_UP,     INPUT_PULLUP);
  mcp.pinMode(MCP_BTN_DOWN,   INPUT_PULLUP);
  mcp.pinMode(MCP_BTN_SELECT, INPUT_PULLUP);
  mcp.pinMode(MCP_BTN_BACK,   INPUT_PULLUP);

  // Configurar LED como salida (apagado inicialmente)
  mcp.pinMode(MCP_LED_ESTADO, OUTPUT);
  mcp.digitalWrite(MCP_LED_ESTADO, LOW);

  // Pin digital del sensor de lluvia (conectado directo al ESP32)
  pinMode(RAINDROPS_DIGITAL_PIN, INPUT);

  // MPU6050 — reset de hardware explícito antes de cualquier otra cosa.
  // FIX v1.1: El chip arranca en sleep mode (bit 6 de PWR_MGMT_1 = 1).
  // mpu.initialize() no garantiza liberar ese bit en todos los casos.
  // La secuencia correcta es: DEVICE_RESET → esperar → despertar con PLL.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x80);  // Bit 7 = DEVICE_RESET: reinicia todos los registros
  Wire.endTransmission();
  delay(100);        // El datasheet exige mínimo 100 ms después del reset

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x01);  // SLEEP=0, CLKSEL=1 (PLL con ref. giroscopio X — más estable)
  Wire.endTransmission();
  delay(50);

  // Configurar rangos explícitamente (no depender de defaults post-reset)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_CFG);
  Wire.write(0x00);  // ±2 g (16384 LSB/g)
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_CFG);
  Wire.write(0x00);  // ±250 °/s
  Wire.endTransmission();
  delay(20);

  // Ahora sí inicializar la librería (configura DLPF y otros registros)
  mpu.initialize();

  if (!mpu.testConnection()) {
    mostrarErrorCritico("MPU6050 ERROR", "Verifica SDA=21 SCL=22");
    while (true) { delay(1000); }
  }
}

// Muestra un error crítico en OLED y sugiere reiniciar
void mostrarErrorCritico(const char* linea1, const char* linea2) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(5, 15, "!! ERROR !!");
  u8g2.drawStr(5, 30, linea1);
  u8g2.drawStr(5, 43, linea2);
  u8g2.drawStr(5, 58, "Reinicia el equipo");
  u8g2.sendBuffer();
}

// =============================================================================
// SECCIÓN 9 — LECTURA DE BOTONES (debounce 150 ms vía MCP23017)
// =============================================================================

// Retorna true en el flanco descendente del botón (presionado), con debounce
bool leerBoton(int indiceDebounce, int pinMCP) {
  bool estadoActualBtn = mcp.digitalRead(pinMCP);  // LOW = presionado (pullup activo)

  if (estadoActualBtn == LOW && btn_estadoAnterior[indiceDebounce] == true) {
    if (millis() - btn_ultimoTiempoMs[indiceDebounce] > 150) {
      btn_ultimoTiempoMs[indiceDebounce]  = millis();
      btn_estadoAnterior[indiceDebounce]  = false;
      return true;
    }
  }

  if (estadoActualBtn == HIGH) {
    btn_estadoAnterior[indiceDebounce] = true;
  }

  return false;
}

// =============================================================================
// SECCIÓN 10 — SENSOR DE HUMEDAD YL-69
// =============================================================================

// Lee el YL-69 y retorna humedad en porcentaje (0–100%)
int getHumedadPorcentaje() {
  int valor = analogRead(SENSOR_HUMEDAD_PIN);
  int pct   = map(valor, HUMEDAD_SECO, HUMEDAD_MOJADO, 0, 100);
  if (pct > 100) pct = 100;
  if (pct < 0)   pct = 0;
  return pct;
}

// =============================================================================
// SECCIÓN 11 — DEAD RECKONING MPU6050
// =============================================================================

// Calibra el bias del acelerómetro promediando N lecturas en reposo.
// Llamar siempre con el sensor QUIETO y HORIZONTAL sobre una superficie plana.
void calibrarBiasMPU() {
  int16_t ax, ay, az, gx, gy, gz;
  long sumAX = 0, sumAY = 0;

  // Mostrar aviso en OLED mientras se calibra (dura ~500 ms)
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(5, 20, "Calibrando MPU6050");
  u8g2.drawStr(5, 35, "Mantenga el sensor");
  u8g2.drawStr(5, 48, "quieto y horizontal.");
  u8g2.sendBuffer();

  delay(200);  // Dar tiempo al sensor a estabilizarse tras el wake-up

  for (int i = 0; i < MPU_BIAS_MUESTRAS; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumAX += ax;
    sumAY += ay;
    delay(5);
  }

  // Convertir LSB → m/s² (escala ±2 g: 16384 LSB/g, g = 9.81 m/s²)
  dr_biasAX = (sumAX / (float)MPU_BIAS_MUESTRAS) / 16384.0f * 9.81f;
  dr_biasAY = (sumAY / (float)MPU_BIAS_MUESTRAS) / 16384.0f * 9.81f;
}

// Reinicia posición, velocidad y timestamp del dead reckoning
void resetearDeadReckoning() {
  dr_velX = 0; dr_velY = 0;
  dr_posX = 0; dr_posY = 0;
  dr_ultimoTiempoMs = millis();
}

// Integra aceleración → velocidad → posición en cada ciclo de MPU_INTERVALO_MS
void actualizarDeadReckoning() {
  if (millis() - dr_ultimoTiempoMs < MPU_INTERVALO_MS) return;

  float dt = (millis() - dr_ultimoTiempoMs) / 1000.0f;
  dr_ultimoTiempoMs = millis();

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = (ax / 16384.0f * 9.81f) - dr_biasAX;
  float accelY = (ay / 16384.0f * 9.81f) - dr_biasAY;

  // Filtro de zona muerta: ignorar micro-vibraciones bajo el umbral
  if (fabsf(accelX) < MPU_UMBRAL_RUIDO) accelX = 0;
  if (fabsf(accelY) < MPU_UMBRAL_RUIDO) accelY = 0;

  dr_velX += accelX * dt;
  dr_velY += accelY * dt;
  dr_posX += dr_velX * dt;
  dr_posY += dr_velY * dt;
}

// Retorna la distancia euclidiana acumulada desde el último resetearDeadReckoning()
float getDistanciaRecorrida() {
  return sqrtf(dr_posX * dr_posX + dr_posY * dr_posY);
}

// =============================================================================
// SECCIÓN 12 — HANDLERS DE ESTADOS FSM
// =============================================================================

// Navega el menú principal (Modo Campo / Modo Laboratorio) y transiciona
void handleMenuPrincipal() {
  if (leerBoton(IDX_BTN_UP,   MCP_BTN_UP))   menuPrincipalIndex = (menuPrincipalIndex + 1) % 2;
  if (leerBoton(IDX_BTN_DOWN, MCP_BTN_DOWN)) menuPrincipalIndex = (menuPrincipalIndex + 1) % 2;

  if (leerBoton(IDX_BTN_SELECT, MCP_BTN_SELECT)) {
    if (menuPrincipalIndex == 0) {
      estadoActual = CAMPO_INSTRUCCIONES;
    } else {
      labMenuIndex = 0;
      estadoActual = LAB_MENU;
    }
  }

  dibujarMenuPrincipal();
}

// Muestra instrucciones del modo campo y espera confirmación del usuario
void handleCampoInstrucciones() {
  if (leerBoton(IDX_BTN_SELECT, MCP_BTN_SELECT)) {
    // Reiniciar datos de sesión de campo
    campo_puntosCompletados = 0;
    campo_lecturasActuales  = 0;
    campo_sumaLecturas      = 0;
    for (int i = 0; i < CAMPO_NUM_PUNTOS; i++) campo_mediciones[i] = 0;

    mcp.digitalWrite(MCP_LED_ESTADO, LOW);

    // Calibrar MPU6050 en reposo antes de empezar
    calibrarBiasMPU();
    resetearDeadReckoning();

    estadoActual = CAMPO_ESPERANDO_DESPLAZAMIENTO;
    return;
  }

  if (leerBoton(IDX_BTN_BACK, MCP_BTN_BACK)) {
    estadoActual = MENU_PRINCIPAL;
    return;
  }

  dibujarCampoInstrucciones();
}

// Integra el MPU6050 y espera hasta detectar ~1 m de desplazamiento
void handleCampoEsperandoDesplazamiento() {
  actualizarDeadReckoning();
  float distancia = getDistanciaRecorrida();

  if (leerBoton(IDX_BTN_BACK, MCP_BTN_BACK)) {
    mcp.digitalWrite(MCP_LED_ESTADO, LOW);
    estadoActual = MENU_PRINCIPAL;
    return;
  }

  if (distancia >= CAMPO_DISTANCIA_META) {
    // Desplazamiento suficiente → apagar LED e iniciar medición
    campo_lecturasActuales = 0;
    campo_sumaLecturas     = 0;
    mcp.digitalWrite(MCP_LED_ESTADO, LOW);
    estadoActual = CAMPO_MIDIENDO;
    return;
  }

  dibujarCampoEsperando(distancia);
}

// Acumula 25 lecturas del YL-69 no-bloqueantes; transiciona al completar
void handleCampoMidiendo() {
  dibujarCampoMidiendo(campo_lecturasActuales);

  if (campo_lecturasActuales < CAMPO_LECTURAS_PUNTO) {
    campo_sumaLecturas += getHumedadPorcentaje();
    campo_lecturasActuales++;
    delay(20);  // 20 ms entre lecturas para estabilidad del ADC
  } else {
    float promedioPunto = campo_sumaLecturas / (float)CAMPO_LECTURAS_PUNTO;
    campo_mediciones[campo_puntosCompletados] = promedioPunto;
    campo_puntosCompletados++;

    // LED encendido: indica al usuario que puede desplazarse al siguiente punto
    mcp.digitalWrite(MCP_LED_ESTADO, HIGH);
    estadoActual = CAMPO_PUNTO_COMPLETO;
  }
}

// Muestra el resultado del punto recién medido y gestiona continuación o fin
void handleCampoPuntoCompleto() {
  float promedioUltimo = campo_mediciones[campo_puntosCompletados - 1];

  if (leerBoton(IDX_BTN_BACK, MCP_BTN_BACK)) {
    mcp.digitalWrite(MCP_LED_ESTADO, LOW);
    estadoActual = MENU_PRINCIPAL;
    return;
  }

  if (campo_puntosCompletados >= CAMPO_NUM_PUNTOS) {
    // Todos los puntos completados → mostrar resultados al confirmar
    if (leerBoton(IDX_BTN_SELECT, MCP_BTN_SELECT)) {
      mcp.digitalWrite(MCP_LED_ESTADO, LOW);
      estadoActual = CAMPO_RESULTADO;
    }
  } else {
    // Quedan puntos → preparar siguiente desplazamiento al confirmar
    if (leerBoton(IDX_BTN_SELECT, MCP_BTN_SELECT)) {
      resetearDeadReckoning();
      // LED permanece encendido durante el desplazamiento al siguiente punto
      estadoActual = CAMPO_ESPERANDO_DESPLAZAMIENTO;
    }
  }

  dibujarCampoPuntoCompleto(campo_puntosCompletados, promedioUltimo);
}

// Calcula y muestra el promedio final de las 10 mediciones de campo
void handleCampoResultado() {
  float suma = 0;
  for (int i = 0; i < campo_puntosCompletados; i++) suma += campo_mediciones[i];
  float promedioFinal = (campo_puntosCompletados > 0) ? suma / campo_puntosCompletados : 0;

  if (leerBoton(IDX_BTN_SELECT, MCP_BTN_SELECT)) {
    estadoActual = CAMPO_INSTRUCCIONES;
    return;
  }
  if (leerBoton(IDX_BTN_BACK, MCP_BTN_BACK)) {
    estadoActual = MENU_PRINCIPAL;
    return;
  }

  dibujarCampoResultado(campo_puntosCompletados, promedioFinal);
}

// Navega el submenú de laboratorio (Humedad / Infiltración)
void handleLabMenu() {
  if (leerBoton(IDX_BTN_UP,   MCP_BTN_UP))   labMenuIndex = (labMenuIndex + 1) % 2;
  if (leerBoton(IDX_BTN_DOWN, MCP_BTN_DOWN)) labMenuIndex = (labMenuIndex + 1) % 2;

  if (leerBoton(IDX_BTN_SELECT, MCP_BTN_SELECT)) {
    if (labMenuIndex == 0) {
      lab_humedadPorcentaje = 0;
      estadoActual = LAB_HUMEDAD_MIDIENDO;
    } else {
      infil_tiempoInicio         = millis();
      infil_mmAcumulados         = 0;
      infil_mmPorMinuto          = 0;
      infil_estadoRainAnterior   = true;  // Asumir inicio en seco
      estadoActual = LAB_INFILTRACION_ACTIVA;
    }
    return;
  }

  if (leerBoton(IDX_BTN_BACK, MCP_BTN_BACK)) {
    estadoActual = MENU_PRINCIPAL;
    return;
  }

  dibujarLabMenu();
}

// Promedia 20 lecturas del YL-69 y muestra humedad en tiempo real
void handleLabHumedadMidiendo() {
  // Leer promedio fresco en cada iteración del loop
  long suma = 0;
  for (int i = 0; i < LAB_HUMEDAD_NUM_LECTURAS; i++) {
    suma += getHumedadPorcentaje();
    delay(10);
  }
  lab_humedadPorcentaje = suma / (float)LAB_HUMEDAD_NUM_LECTURAS;

  if (leerBoton(IDX_BTN_SELECT, MCP_BTN_SELECT)) {
    // Reiniciar: limpiar valor y continuar midiendo en la próxima iteración
    lab_humedadPorcentaje = 0;
  }
  if (leerBoton(IDX_BTN_BACK, MCP_BTN_BACK)) {
    estadoActual = LAB_MENU;
    return;
  }

  dibujarLabHumedad(lab_humedadPorcentaje);
}

// Acumula agua infiltrada vía flancos del sensor de lluvia durante 5 minutos
void handleLabInfiltracionActiva() {
  unsigned long transcurrido = millis() - infil_tiempoInicio;

  // Detectar flanco descendente en pin digital (HIGH→LOW = primera gota detectada)
  bool estadoRainActual = (digitalRead(RAINDROPS_DIGITAL_PIN) == HIGH);
  if (!estadoRainActual && infil_estadoRainAnterior) {
    infil_mmAcumulados += MM_POR_DETECCION_LLUVIA;
  }
  infil_estadoRainAnterior = estadoRainActual;

  if (leerBoton(IDX_BTN_BACK, MCP_BTN_BACK)) {
    estadoActual = LAB_MENU;
    return;
  }

  if (transcurrido >= LAB_INFILTRACION_DURACION_MS) {
    float minutosTotal = transcurrido / 60000.0f;
    infil_mmPorMinuto  = (minutosTotal > 0) ? infil_mmAcumulados / minutosTotal : 0;
    estadoActual = LAB_INFILTRACION_RESULTADO;
    return;
  }

  dibujarInfiltracionActiva(transcurrido, infil_mmAcumulados);
}

// Muestra resultados finales de infiltración y permite reiniciar o volver
void handleLabInfiltracionResultado() {
  unsigned long duracionTotal = millis() - infil_tiempoInicio;

  if (leerBoton(IDX_BTN_SELECT, MCP_BTN_SELECT)) {
    infil_tiempoInicio       = millis();
    infil_mmAcumulados       = 0;
    infil_mmPorMinuto        = 0;
    infil_estadoRainAnterior = true;
    estadoActual = LAB_INFILTRACION_ACTIVA;
    return;
  }
  if (leerBoton(IDX_BTN_BACK, MCP_BTN_BACK)) {
    estadoActual = LAB_MENU;
    return;
  }

  dibujarInfiltracionResultado(infil_mmAcumulados, infil_mmPorMinuto, duracionTotal);
}

// =============================================================================
// SECCIÓN 13 — FUNCIONES DE DIBUJO OLED
// =============================================================================

// Dibuja una barra de progreso con marco exterior y relleno proporcional
void dibujarBarraProgreso(int porcentaje, int x, int y, int w, int h) {
  int fill = map(constrain(porcentaje, 0, 100), 0, 100, 0, w - 4);
  u8g2.drawFrame(x, y, w, h);
  if (fill > 0) u8g2.drawBox(x + 2, y + 2, fill, h - 4);
}

// Dibuja el menú principal con título SEMILLA y cursor de selección invertido
void dibujarMenuPrincipal() {
  const char* items[] = { "Modo Campo", "Modo Laboratorio" };

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawStr(25, 12, "SEMILLA");
  u8g2.drawHLine(0, 14, 128);
  u8g2.setFont(u8g2_font_6x10_tf);

  for (int i = 0; i < 2; i++) {
    int y = 28 + i * 18;
    if (i == menuPrincipalIndex) {
      u8g2.drawBox(0, y - 11, 128, 13);
      u8g2.setDrawColor(0);
    } else {
      u8g2.setDrawColor(1);
    }
    u8g2.setCursor(4, y);
    u8g2.print("> ");
    u8g2.print(items[i]);
  }

  u8g2.setDrawColor(1);
  u8g2.sendBuffer();
}

// Dibuja las instrucciones del modo campo antes de iniciar
void dibujarCampoInstrucciones() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(10, 10, "MODO CAMPO");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 24, "Camine ~1m entre ptos.");
  u8g2.drawStr(0, 34, "Se tomaran 10 puntos,");
  u8g2.drawStr(0, 44, "25 lecturas c/u.");
  u8g2.drawStr(0, 54, "Coloque sensor al inicio.");

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 63, "3=Iniciar      4=Salir");
  u8g2.sendBuffer();
}

// Dibuja el estado de espera de desplazamiento con barra de progreso de distancia
void dibujarCampoEsperando(float distancia) {
  int pct = (int)constrain(distancia / CAMPO_DISTANCIA_META * 100.0f, 0, 100);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Desplazandose...");

  u8g2.setCursor(0, 24);
  u8g2.print("Dist: ");
  u8g2.print(distancia, 2);
  u8g2.print(" / ");
  u8g2.print(CAMPO_DISTANCIA_META, 1);
  u8g2.print(" m");

  dibujarBarraProgreso(pct, 0, 28, 128, 10);

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.setCursor(0, 47);
  u8g2.print("Punto ");
  u8g2.print(campo_puntosCompletados + 1);
  u8g2.print(" de ");
  u8g2.print(CAMPO_NUM_PUNTOS);
  u8g2.print("  (LED ON)");

  u8g2.drawStr(0, 58, "4=Cancelar");
  u8g2.sendBuffer();
}

// Dibuja el progreso de las 25 lecturas activas del punto actual
void dibujarCampoMidiendo(int lecturas) {
  int pct = (lecturas * 100) / CAMPO_LECTURAS_PUNTO;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Midiendo sensor...");

  u8g2.setCursor(0, 24);
  u8g2.print("Lectura ");
  u8g2.print(lecturas);
  u8g2.print(" / ");
  u8g2.print(CAMPO_LECTURAS_PUNTO);

  dibujarBarraProgreso(pct, 0, 28, 128, 10);

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.setCursor(0, 47);
  u8g2.print("Punto ");
  u8g2.print(campo_puntosCompletados + 1);
  u8g2.print(" de ");
  u8g2.print(CAMPO_NUM_PUNTOS);
  u8g2.sendBuffer();
}

// Dibuja el resumen del punto completado con su promedio de humedad
void dibujarCampoPuntoCompleto(int puntos, float promedioUltimo) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Punto completado");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setCursor(0, 26);
  u8g2.print("Punto #");
  u8g2.print(puntos);
  u8g2.print(": ");
  u8g2.print(promedioUltimo, 1);
  u8g2.print("%");

  u8g2.setCursor(0, 40);
  u8g2.print("Total: ");
  u8g2.print(puntos);
  u8g2.print(" / ");
  u8g2.print(CAMPO_NUM_PUNTOS);

  u8g2.setFont(u8g2_font_5x8_tr);
  if (puntos >= CAMPO_NUM_PUNTOS) {
    u8g2.drawStr(0, 54, "3=Ver resultados");
    u8g2.drawStr(0, 63, "4=Menu principal");
  } else {
    u8g2.drawStr(0, 54, "LED ON: desplacese 1m");
    u8g2.drawStr(0, 63, "3=Continuar  4=Salir");
  }
  u8g2.sendBuffer();
}

// Dibuja la pantalla de resultados finales del modo campo
void dibujarCampoResultado(int puntos, float promedioFinal) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(5, 10, "RESULTADO CAMPO");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setCursor(0, 26);
  u8g2.print("Puntos medidos: ");
  u8g2.print(puntos);

  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.setCursor(15, 44);
  u8g2.print("Prom: ");
  u8g2.print(promedioFinal, 1);
  u8g2.print("%");

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 56, "Medicion finalizada.");
  u8g2.drawStr(0, 64, "3=Reiniciar  4=Menu");
  u8g2.sendBuffer();
}

// Dibuja el submenú de laboratorio con cursor de selección
void dibujarLabMenu() {
  const char* items[] = { "Medir Humedad", "Medir Infiltracion" };

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(10, 10, "LABORATORIO");
  u8g2.drawHLine(0, 12, 128);

  for (int i = 0; i < 2; i++) {
    int y = 28 + i * 18;
    if (i == labMenuIndex) {
      u8g2.drawBox(0, y - 11, 128, 13);
      u8g2.setDrawColor(0);
    } else {
      u8g2.setDrawColor(1);
    }
    u8g2.setCursor(4, y);
    u8g2.print("> ");
    u8g2.print(items[i]);
  }

  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 62, "4=Volver al menu");
  u8g2.sendBuffer();
}

// Dibuja la pantalla de medición de humedad en laboratorio con barra y etiqueta
void dibujarLabHumedad(float porcentaje) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(5, 10, "HUMEDAD - LAB");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_10x20_tf);
  u8g2.setCursor(22, 38);
  u8g2.print((int)porcentaje);
  u8g2.print(" %");

  dibujarBarraProgreso((int)porcentaje, 0, 42, 128, 8);

  u8g2.setFont(u8g2_font_5x8_tr);
  const char* etiqueta = (porcentaje < 30) ? "SECO" : (porcentaje < 70) ? "NORMAL" : "HUMEDO";
  u8g2.drawStr(50, 56, etiqueta);
  u8g2.drawStr(0, 64, "3=Reiniciar  4=Volver");
  u8g2.sendBuffer();
}

// Dibuja el progreso de la prueba de infiltración con temporizador y mm acumulados
void dibujarInfiltracionActiva(unsigned long transcurridoMs, float mm) {
  unsigned long seg = transcurridoMs / 1000;
  unsigned long min = seg / 60;
  seg = seg % 60;
  int pct = (int)constrain(transcurridoMs * 100UL / LAB_INFILTRACION_DURACION_MS, 0, 100);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(5, 10, "INFILTRACION");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setCursor(0, 25);
  u8g2.print("Tiempo: ");
  if (min < 10) u8g2.print("0");
  u8g2.print(min);
  u8g2.print(":");
  if (seg < 10) u8g2.print("0");
  u8g2.print(seg);
  u8g2.print(" / 05:00");

  dibujarBarraProgreso(pct, 0, 29, 128, 8);

  u8g2.setCursor(0, 46);
  u8g2.print("Agua: ");
  u8g2.print(mm, 2);
  u8g2.print(" mm");

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 58, "4=Cancelar prueba");
  u8g2.sendBuffer();
}

// Dibuja los resultados finales de la prueba de infiltración
void dibujarInfiltracionResultado(float mmTotal, float mmPorMin, unsigned long duracionMs) {
  unsigned long min = duracionMs / 60000;
  unsigned long seg = (duracionMs % 60000) / 1000;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "RESULTADO INFILTR.");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setCursor(0, 25);
  u8g2.print("Duracion: ");
  if (min < 10) u8g2.print("0");
  u8g2.print(min);
  u8g2.print(":");
  if (seg < 10) u8g2.print("0");
  u8g2.print(seg);

  u8g2.setCursor(0, 38);
  u8g2.print("Total:  ");
  u8g2.print(mmTotal, 2);
  u8g2.print(" mm");

  u8g2.setCursor(0, 51);
  u8g2.print("Tasa:   ");
  u8g2.print(mmPorMin, 2);
  u8g2.print(" mm/min");

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 63, "3=Reiniciar  4=Volver");
  u8g2.sendBuffer();
}
