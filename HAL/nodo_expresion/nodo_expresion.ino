/**
 * =========================================================================
 * PROYECTO:     GARRA-OS  -  Agente Robotico Autonomo de Combate ("Chappie")
 * ARCHIVO:      nodo_expresion.ino
 * NODO:         EXPRESION  (ESP32 DevKit V1, 30 pines)
 *
 * OBJETIVO DEL ARCHIVO:
 *   Dar "cara" y personalidad al robot controlando un Ventilador Holografico
 *   comercial. El nodo se conecta por WiFi a la red propia del ventilador y le
 *   inyecta comandos TCP propietarios (knock + payload) para encender/apagar y
 *   proyectar distintas imagenes (caras y animaciones). Recibe las ordenes del
 *   Cerebro por cable UART y las traduce a una maquina de estados de animacion
 *   (IDLE, BAILE, SEGUIR y CUARENTENA de emergencia).
 *
 * INTEGRANTES:
 *   - Alcala Ramos Luz Estefania      (23240079)
 *   - Bahena Mora Emilio Salvador     (23240009)
 *   - Casas Bastidas Jose Ivan        (23240883)
 *   - Fischer Gonzalez Patrick        (23240045)
 *
 * MATERIA:      Sistemas Programables
 * DOCENTE:      Ma. Veronica Tapia Ibarra
 * INSTITUCION:  Instituto Tecnologico de Leon
 * =========================================================================
 */

#include <WiFi.h>      // Conexion WiFi (nos unimos a la red del ventilador).
#include <esp_wifi.h>  // Permite clonar una MAC concreta (el ventilador la exige).

// =========================================================================
// BUFFER DE RECEPCION UART Y PARAMETROS DE RED
// =========================================================================
const int BUFFER_SIZE = 64;        // Tamano del buffer donde armamos la linea recibida.
char uart_buffer[BUFFER_SIZE];     // Almacen temporal de caracteres del UART.
int buffer_index = 0;              // Posicion actual de escritura en el buffer.

const char* ssid     = "3D_42CM_67F854"; // Red WiFi que emite el ventilador holografico.
const char* password = "";                // Red abierta (sin contrasena).
const char* host     = "192.168.4.1";     // IP del ventilador (actua como punto de acceso).
const uint16_t port  = 20320;             // Puerto TCP propietario del ventilador.

#define RX_PIN 16   // Pin de recepcion del puerto serie 2 (UART desde el Cerebro).
#define TX_PIN 17   // Pin de transmision del puerto serie 2.

unsigned long lastWifiCheckTime = 0;          // Marca de tiempo del ultimo chequeo de WiFi.
const unsigned long WIFI_CHECK_INTERVAL = 5000; // Revisamos el enlace cada 5 segundos.
bool ventiladorEncendido = true;              // Estado logico de encendido del ventilador.

// =========================================================================
// MAQUINA DE ESTADOS FINITOS (FSM) DE ANIMACION
// =========================================================================
enum EstadoFSM { MODO_IDLE, MODO_BAILE, MODO_SEGUIR, MODO_CUARENTENA };
EstadoFSM estado_actual = MODO_IDLE;          // Estado inicial: reposo.

unsigned long cronometro_estado = 0;          // Cronometro reutilizable por estado.

// --- PARAMETROS DE LA RUTINA "SEGUIR" ---
const unsigned long DURACION_SEGUIR = 20000;  // 20 s en total (10 s IRONCLAW + 10 s RUGIDO).
int seguirPaso = -1;                          // Fase actual de la rutina (-1 = sin iniciar).
const unsigned long TIEMPO_ALTO_TOTAL = 30000;// Duracion de la cuarentena de emergencia.

// =========================================================================
// FUNCIONES TCP DE BAJO NIVEL (PROTOCOLO DEL VENTILADOR)
// =========================================================================
// Alterna (ON/OFF) la energia del ventilador holografico.
bool alternarEnergiaHolograma() {
  WiFiClient client;                                  // Cliente TCP.
  if (!client.connect(host, port)) {                  // Intentamos conectar al ventilador.
      Serial.println("[ERROR TCP] No se pudo conectar al ventilador para TOGGLE.");
      return false;
  }
  client.setNoDelay(true);                            // Desactiva el buffer Nagle (envio inmediato).

  // "KNOCK": handshake inicial con las MAC de origen y destino del protocolo.
  uint8_t knock[24];
  memcpy(knock, "C0EEB7C9BAA3", 12); memcpy(knock + 12, "C0EEBDF9E5B7", 12);
  client.write(knock, 24); client.flush();

  // Vaciamos cualquier respuesta del ventilador (no la necesitamos).
  unsigned long t = millis();
  while(client.available() == 0 && millis() - t < 1000) delay(1);
  while(client.available()) client.read();

  // Payload de "poder": los bytes 'c','c','a' son el codigo de encender/apagar.
  uint8_t powerPayload[28];
  memcpy(powerPayload, "C0EEB7C9BAA3", 12);
  powerPayload[12] = 0x00; powerPayload[13] = 'c'; powerPayload[14] = 'c'; powerPayload[15] = 'a';
  memcpy(powerPayload + 16, "C0EEBDF9E5B7", 12);

  client.write(powerPayload, 28); client.flush();     // Inyectamos el comando.
  delay(200); client.stop();                          // Cerramos la conexion.
  Serial.println("[TCP] Comando de Poder inyectado.");
  return true;
}

// Proyecta una imagen/animacion concreta segun su indice (cara o efecto).
bool proyectarHolograma(uint8_t indiceImagen) {
  WiFiClient client;
  if (!client.connect(host, port)) {
      Serial.println("[ERROR TCP] No se pudo conectar al ventilador para HOLOGRAMA.");
      return false;
  }
  client.setNoDelay(true);

  // Paso 1: KNOCK (handshake, igual que arriba).
  uint8_t knock[24];
  memcpy(knock, "C0EEB7C9BAA3", 12); memcpy(knock + 12, "C0EEBDF9E5B7", 12);
  client.write(knock, 24); client.flush();
  unsigned long t = millis();
  while(client.available() == 0 && millis() - t < 1000) delay(1);
  while(client.available()) client.read();

  // Paso 2: SELECT. Codigo 'c','d','B' + indice de la imagen a seleccionar.
  uint8_t selectPayload[29];
  memcpy(selectPayload, "C0EEB7C9BAA3", 12);
  selectPayload[12] = 0x00; selectPayload[13] = 'c'; selectPayload[14] = 'd'; selectPayload[15] = 'B';
  selectPayload[16] = indiceImagen;                   // Aqui va el numero de imagen (ej. 0x05 = FELIZ).
  memcpy(selectPayload + 17, "C0EEBDF9E5B7", 12);
  client.write(selectPayload, 29); client.flush();
  t = millis();
  while(client.available() == 0 && millis() - t < 1000) delay(1);
  while(client.available()) client.read();

  // Paso 3: PLAY. Codigo 'c','g','b' + numero de secuencia para forzar la reproduccion.
  uint8_t playPayload[32];
  memcpy(playPayload, "C0EEB7C9BAA3", 12);
  playPayload[12] = 0x00; playPayload[13] = 'c'; playPayload[14] = 'g'; playPayload[15] = 'b';
  uint16_t seq = millis() & 0xFFFF;                   // Secuencia variable para que el aparato no la ignore.
  playPayload[16] = seq & 0xFF; playPayload[17] = (seq >> 8) & 0xFF;
  playPayload[18] = 0x00; playPayload[19] = 0x00;
  memcpy(playPayload + 20, "C0EEBDF9E5B7", 12);
  client.write(playPayload, 32); client.flush();
  delay(200); client.stop();
  Serial.printf("[TCP EXITO] Holograma 0x%02X ejecutado en red fisica.\n", indiceImagen);
  return true;
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);                              // Consola de depuracion.
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); // Puerto serie hacia el Cerebro.
  delay(1000);

  Serial.println("\n=======================================");
  Serial.println("  GARRA-OS: NODO EXPRESION (V_DIETA)");
  Serial.println("=======================================");

  WiFi.mode(WIFI_STA);                               // Modo estacion.
  // El ventilador solo acepta ordenes de una MAC concreta: la clonamos.
  uint8_t phoneMAC[] = {0xC0, 0xEE, 0xBD, 0xF9, 0xE5, 0xB7};
  esp_wifi_set_mac(WIFI_IF_STA, &phoneMAC[0]);

  Serial.print("[WIFI] Conectando a Ventilador...");
  WiFi.begin(ssid, password);                        // Nos unimos a la red del ventilador.
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.println("\n[WIFI] !Conectado al Ventilador Holografico!");

  // Al arrancar, dejamos la cara FELIZ (0x05) como estado de reposo valido.
  Serial.println("[BOOT] Inyectando cara inicial para ciclo IDLE (0x05 FELIZ)...");
  proyectarHolograma(0x05);
  cronometro_estado = millis();
}

// =========================================================================
// LOOP
// =========================================================================
void loop() {
  unsigned long currentMillis = millis();

  // Vigilancia del enlace WiFi: si se cae, intentamos reconectar.
  if (currentMillis - lastWifiCheckTime >= WIFI_CHECK_INTERVAL) {
    lastWifiCheckTime = currentMillis;
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] Conexion perdida. Reconectando...");
        WiFi.reconnect();
    }
  }

  // =========================================================
  // MOTOR DE ESTADOS (FSM)
  // =========================================================
  switch (estado_actual) {
    case MODO_IDLE:
      // Reposo: el ventilador cicla libremente la cara feliz; nada que hacer.
      break;

    case MODO_BAILE:
      // El holograma de baile ya se disparo una sola vez; solo esperamos otra orden.
      break;

    case MODO_CUARENTENA:
      // Tras una emergencia, esperamos 30 s antes de volver a la normalidad.
      if (currentMillis - cronometro_estado >= TIEMPO_ALTO_TOTAL) {
          Serial.println("\n[FSM] Cuarentena de 30s finalizada.");
          if (!ventiladorEncendido) {                // Si lo habiamos apagado, lo reencendemos.
              Serial.println("[FSM] Encendiendo ventilador de forma automatica...");
              if (alternarEnergiaHolograma()) ventiladorEncendido = true;
          }
          estado_actual = MODO_IDLE;                 // Regresamos a reposo.
          Serial.println("[FSM] Retornando a ciclo IDLE libre...");
          proyectarHolograma(0x05);                  // Reactivamos la cara FELIZ.
      }
      break;

    case MODO_SEGUIR: {
      // Rutina de 20 s en dos fases (IRONCLAW -> RUGIDO).
      unsigned long tiempoTranscurrido = currentMillis - cronometro_estado;
      if (tiempoTranscurrido >= DURACION_SEGUIR) {   // Si ya terminaron los 20 s...
          Serial.println("\n[FSM] Rutina SEGUIR de 20s finalizada. Retornando a IDLE.");
          estado_actual = MODO_IDLE;
          proyectarHolograma(0x05);                  // Volvemos a FELIZ.
      } else {
          // Fase 0 = primeros 10 s; Fase 1 = segundos 10 s.
          int pasoCalculado = (tiempoTranscurrido < 10000) ? 0 : 1;
          if (pasoCalculado != seguirPaso) {         // Solo inyectamos al cambiar de fase.
              seguirPaso = pasoCalculado;
              uint8_t hexHolo = (seguirPaso == 0) ? 0x06 : 0x07; // 0x06=IRONCLAW, 0x07=RUGIDO.
              Serial.printf("[RUTINA SEGUIR] Fase %d/2. Inyectando 0x%02X...\n", seguirPaso + 1, hexHolo);
              proyectarHolograma(hexHolo);
          }
      }
      break;
    }
  }

  // =========================================================
  // PARSEO DEL UART (Ordenes que llegan del Cerebro)
  // =========================================================
  while (Serial2.available() > 0) {
    char c = Serial2.read();                          // Leemos un caracter.
    if (buffer_index >= BUFFER_SIZE - 1) buffer_index = 0; // Anti-desbordamiento del buffer.

    if (c == '\n') {                                  // Si recibimos fin de linea...
      uart_buffer[buffer_index] = '\0';               // Cerramos la cadena.
      buffer_index = 0;                               // Reiniciamos el indice.

      // Esperamos el formato "tipo,destino,accion" (ej. "2,EXP,BAILAR").
      int tipo_recibido; char destino_recibido[5]; char accion_recibida[16];
      if (sscanf(uart_buffer, "%d,%4[^,],%15[^\r\n]", &tipo_recibido, destino_recibido, accion_recibida) == 3) {
          // Solo atendemos lo dirigido a Expresion ("EXP") o a todos ("ALL").
          if (strcmp(destino_recibido, "EXP") == 0 || strcmp(destino_recibido, "ALL") == 0) {

              Serial.println("\n-----------------------------------------");
              Serial.printf("[UART IN] Recibido: %s\n", accion_recibida);
              Serial2.print("3,CER,ACK_EXP\n");        // Devolvemos ACK al Cerebro.
              Serial.println("[HANDSHAKE] ACK devuelto al Cerebro.");

              String comandoStr = String(accion_recibida); // Convertimos a String para comparar.

              // --- TABLA DE DECISIONES SEGUN EL COMANDO ---
              if (comandoStr == "ALTOTOTAL") {          // Emergencia: entrar en cuarentena.
                  Serial.println("[CRITICO] Iniciando Cuarentena de 30s.");
                  estado_actual = MODO_CUARENTENA;
                  cronometro_estado = currentMillis;
                  if (ventiladorEncendido) {            // Apagado de emergencia del ventilador.
                      Serial.println("[FSM] Ejecutando apagado de emergencia (TOGGLE).");
                      if (alternarEnergiaHolograma()) ventiladorEncendido = false;
                  }
              }
              else if (estado_actual == MODO_CUARENTENA || !ventiladorEncendido) {
                  // En cuarentena/apagado solo aceptamos "ENCENDER"; lo demas se ignora.
                  if (comandoStr == "ENCENDER" && estado_actual != MODO_CUARENTENA) {
                      Serial.println("[FSM] Encendiendo ventilador...");
                      if (alternarEnergiaHolograma()) {
                          ventiladorEncendido = true;
                          estado_actual = MODO_IDLE;
                          proyectarHolograma(0x05);     // Cara FELIZ al encender.
                      }
                  } else {
                      Serial.println("[SEGURIDAD] Hardware en cuarentena o apagado. Ignorando comando.");
                  }
              }
              else if (comandoStr == "BAILAR") {        // Disparo unico del holograma de baile.
                  if (estado_actual != MODO_BAILE) {
                      Serial.println("[FSM] ORDEN UNICA: Inyectando holograma de BAILE (0x00)");
                      estado_actual = MODO_BAILE;
                      proyectarHolograma(0x00);         // Indice 0x00 = BAILE.
                  } else {
                      Serial.println("[FSM] Ya estamos en MODO_BAILE. Ignorando repeticion.");
                  }
              }
              else if (comandoStr == "SEGUIR") {        // Inicia la rutina de 20 s en dos fases.
                  if (estado_actual != MODO_SEGUIR) {
                      Serial.println("[FSM] Iniciando Macro SEGUIR (20s - Ironclaw a Rugido)...");
                      estado_actual = MODO_SEGUIR;
                      cronometro_estado = currentMillis;
                      seguirPaso = -1;
                  }
              }
              else if (comandoStr == "REPOSO" || comandoStr == "IDLE") { // Volver a reposo.
                  if (estado_actual != MODO_IDLE) {
                      Serial.println("[FSM] Retornando a ciclo IDLE Libre...");
                      estado_actual = MODO_IDLE;
                      proyectarHolograma(0x05);         // FELIZ.
                  } else {
                      Serial.println("[FSM] Ya estamos en MODO_IDLE.");
                  }
              }
              else if (comandoStr == "APAGAR") {        // Apagado remoto del ventilador.
                  Serial.println("[FSM] Apagando ventilador de forma remota.");
                  if (alternarEnergiaHolograma()) ventiladorEncendido = false;
              }
              else if (comandoStr == "ALERTA") {        // Cara de alerta (ENOJADO 0x02).
                  Serial.println("[FSM] Inyectando holograma de ALERTA (0x02)");
                  proyectarHolograma(0x02);
              }
              else {                                    // Cualquier otra cosa: no reconocida.
                  Serial.println("[ERROR] Comando no reconocido.");
              }
              Serial.println("-----------------------------------------");
          }
      }
    } else {
      uart_buffer[buffer_index++] = c;                 // Aun no es fin de linea: acumulamos el caracter.
    }
  }
}
