/**
 * =========================================================================
 * PROYECTO:     GARRA-OS  -  Agente Robotico Autonomo de Combate ("Chappie")
 * ARCHIVO:      nodo_cerebro.ino
 * NODO:         CEREBRO  (ESP32-S3-WROOM-1)
 *
 * OBJETIVO DEL ARCHIVO:
 *   Actuar como el nodo maestro del robot. Es el unico nodo con acceso a
 *   Internet: descarga los comandos del operador desde la nube (Supabase),
 *   los reparte a los nodos esclavos (Motores por radio ESP-NOW y Expresion
 *   por cable UART) y devuelve la telemetria de estado a la nube. Ademas
 *   ejecuta el reflejo de emergencia de "perdida de suelo" mediante el
 *   sensor optico TCRT5000 conectado por interrupcion.
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

// --- LIBRERIAS DEL SISTEMA ---
#include <WiFi.h>        // Manejo de la conexion WiFi en modo estacion (STA).
#include <HTTPClient.h>  // Cliente HTTP para hablar con la API REST de Supabase.
#include <esp_now.h>     // Protocolo ESP-NOW: radio punto a punto sin router.
#include <ArduinoJson.h> // Serializa y deserializa los cuerpos JSON de Supabase.

// =========================================================================
// ESTRUCTURA GLOBAL GARRA-OS (Paquete de comunicacion por radio)
// =========================================================================
// Todos los nodos comparten EXACTAMENTE esta misma estructura. ESP-NOW envia
// bloques de bytes, por lo que ambos extremos deben coincidir byte por byte.
typedef struct msg_garra {
    uint8_t tipo;         // Categoria del mensaje: 1=BOOT, 2=ACTION, 3=PING/ACK, 4=EMERGENCY.
    char destino[4];      // Nodo destino: "MOT" (motores), "EXP" (expresion) o "ALL" (todos).
    char accion[15];      // Texto del comando (max. 14 caracteres + terminador nulo).
} msg_garra;

// =========================================================================
// CONFIGURACION GLOBAL (Credenciales y constantes de red)
// =========================================================================
const char* WIFI_SSID = "honor";        // Nombre de la red WiFi (hotspot del celular).
const char* WIFI_PASS = "TamalVerde";   // Contrasena de la red WiFi.

// URL base del proyecto en Supabase (capa de servidor / nube).
const String SUPABASE_URL = "https://wybhfefasaduuqapzgaz.supabase.co";
// Llave publica "anon": esta disenada para exponerse en el cliente; la
// seguridad real la imponen las politicas RLS definidas en el script SQL.
const String SUPABASE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Ind5YmhmZWZhc2FkdXVxYXB6Z2F6Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NzkxNDk0ODksImV4cCI6MjA5NDcyNTQ4OX0.qEZ-5tQ3jy0QxjoGMh8MlQ0Sq5_3kIXYn4u34-Fbfw8";

#define RX_PIN 18   // Pin de RECEPCION del puerto serie 1 (UART hacia/desde Expresion).
#define TX_PIN 17   // Pin de TRANSMISION del puerto serie 1.

// --- CONFIGURACION DEL SENSOR DE CAIDA (FASE 1) ---
#define PIN_SENSOR_MH 4   // Pin digital donde entra la senal del modulo TCRT5000.
// Nivel logico que representa "PELIGRO" (no se detecta piso debajo del robot).
// Si el sensor entrega la senal invertida, basta cambiar este 1 por 0.
#define ESTADO_PELIGRO 1

// Direccion MAC del nodo de Motores. Usamos broadcast (FF:FF:FF:FF:FF:FF)
// para no tener que conocer la MAC exacta y maximizar la compatibilidad.
uint8_t slaveMAC1[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// =========================================================================
// VARIABLES DE ESTADO Y EMERGENCIA
// =========================================================================
uint8_t wifi_channel = 0;                 // Canal WiFi obtenido al conectarse (ESP-NOW debe usar el mismo).
String current_command   = "IDLE";        // Ultimo comando ya procesado (para detectar cambios).
String motor_state       = "DESCONOCIDO"; // Estado reportado por el nodo de Motores.
String expression_state  = "DESCONOCIDO"; // Estado reportado por el nodo de Expresion.

unsigned long last_poll_time  = 0;        // Marca de tiempo del ultimo sondeo a Supabase.
unsigned long wait_start_time = 0;        // Marca de tiempo de inicio de la ventana de espera de ACKs.
const unsigned long POLLING_INTERVAL = 999; // Cada cuanto (ms) preguntamos a la nube por comandos.
const unsigned long WAIT_WINDOW      = 500; // Ventana (ms) para recibir confirmaciones de los esclavos.

// Maquina de estados simple del bucle principal: o sondeamos, o esperamos ACKs.
enum LoopState { STATE_POLLING, STATE_WAITING_DEBUG };
LoopState currentState = STATE_POLLING;   // Arrancamos sondeando comandos.

// --- INTERRUPCION DEL SENSOR (FASE 1) ---
// "volatile" obliga al compilador a leer la variable de la RAM cada vez,
// porque puede cambiar en cualquier momento dentro de la interrupcion.
volatile bool banderaEmergencia = false;

// Rutina de Interrupcion (ISR). IRAM_ATTR la coloca en RAM para ejecucion rapida.
// Debe ser MINIMA: solo levanta la bandera; el trabajo pesado se hace en loop().
void IRAM_ATTR isrFrenoCaida() {
    banderaEmergencia = true; // Avisamos al loop que el sensor cambio de estado.
}

// =========================================================================
// FUNCIONES DE SUPABASE
// =========================================================================
// Sube un registro de telemetria a la tabla 'telemetria_debug' (flujo ESP32 -> Web).
void postTelemetry(int error_code, String mensaje, String estado_motores = "", String estado_expresion = "") {
  // Si no hay WiFi no tiene caso intentar el POST; evitamos bloqueos.
  if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[DEBUG-SUPABASE] Error: WiFi desconectado, omitiendo telemetria.");
      return;
  }
  Serial.printf("[DEBUG-SUPABASE] Posteando: Error=%d, Msg=%s\n", error_code, mensaje.c_str());

  HTTPClient http;                                              // Objeto cliente HTTP.
  http.begin(SUPABASE_URL + "/rest/v1/telemetria_debug");       // Endpoint REST de la tabla.
  http.addHeader("apikey", SUPABASE_KEY);                       // Cabecera obligatoria de Supabase.
  http.addHeader("Authorization", "Bearer " + SUPABASE_KEY);    // Token de autorizacion (rol anon).
  http.addHeader("Content-Type", "application/json");           // Avisamos que enviamos JSON.

  StaticJsonDocument<256> doc;            // Documento JSON en memoria estatica (256 bytes).
  doc["estado_maestro"] = "ONLINE";       // Marcamos que el cerebro esta vivo.
  doc["codigo_error"]   = error_code;     // 0 = sin error; >0 = codigo de falla.
  doc["mensaje"]        = mensaje;         // Texto descriptivo del evento.
  // Solo agregamos los estados de los esclavos si nos los pasaron.
  if (estado_motores   != "") doc["estado_motores"]   = estado_motores;
  if (estado_expresion != "") doc["estado_expresion"] = estado_expresion;

  String payload; serializeJson(doc, payload); // Convertimos el JSON a texto plano.
  http.POST(payload);                          // Disparamos la peticion POST.
  http.end();                                  // Liberamos la conexion.
}

// Consulta el comando vigente en la tabla 'control_comandos' (flujo Web -> ESP32).
String fetchCommand() {
  if (WiFi.status() != WL_CONNECTED) return "ERROR"; // Sin WiFi devolvemos "ERROR".
  HTTPClient http;
  // Pedimos SOLO la fila id=1 y SOLO la columna 'comando' para que la respuesta sea minima.
  http.begin(SUPABASE_URL + "/rest/v1/control_comandos?id=eq.1&select=comando");
  http.addHeader("apikey", SUPABASE_KEY);
  http.addHeader("Authorization", "Bearer " + SUPABASE_KEY);
  String commandStr = "ERROR";              // Valor por defecto si algo falla.
  if (http.GET() == HTTP_CODE_OK) {         // Si el servidor responde 200 OK...
    StaticJsonDocument<256> doc;
    deserializeJson(doc, http.getString()); // Parseamos el arreglo JSON recibido.
    if (doc.size() > 0) commandStr = doc[0]["comando"].as<String>(); // Tomamos el primer registro.
  }
  http.end();
  return commandStr;                        // Regresamos el comando leido.
}

// Apaga el sistema de forma controlada ante un error fatal e irrecuperable.
void haltSystem(int error_code, String reason) {
  Serial.printf("\n[FATAL ERROR %d] %s\n", error_code, reason.c_str());
  postTelemetry(error_code, reason);        // Dejamos rastro del fallo en la nube.
  while (true) { yield(); }                 // Bucle infinito; cede CPU para no resetear el watchdog.
}

// =========================================================================
// CALLBACK ESP-NOW (Se ejecuta cuando llega un paquete por radio)
// =========================================================================
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  // Validamos que el paquete tenga el tamano exacto de nuestra estructura.
  if (len == sizeof(msg_garra)) {
      msg_garra *msg = (msg_garra *)incomingData; // Reinterpretamos los bytes como msg_garra.
      // Nos interesan los PING/ACK (tipo 3) dirigidos al Cerebro ("CER").
      if (msg->tipo == 3 && strcmp(msg->destino, "CER") == 0) {
          // Si el nodo de Motores confirma con "ACK_MOT", lo damos por activo.
          if (strcmp(msg->accion, "ACK_MOT") == 0) {
              motor_state = "OK";
              Serial.println("[DEBUG-RADIO] Recepcion: ACK_MOT recibido desde Motores.");
          }
      }
  }
}

// =========================================================================
// SETUP DE PRODUCCION (Secuencia de arranque o "POST")
// =========================================================================
void setup() {
  Serial.begin(115200);                                  // Consola de depuracion por USB.
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);     // Puerto serie hacia el nodo Expresion.
  Serial1.setTimeout(20);                                // Timeout corto de lectura (20 ms).
  delay(1000);                                           // Pausa de estabilizacion al encender.
  Serial.println("\n--- Iniciando GARRA-OS (Nodo Maestro) ---");

  // --- 0. CONFIGURACION DEL REFLEJO DE EMERGENCIA (SENSOR DE CAIDA) ---
  pinMode(PIN_SENSOR_MH, INPUT_PULLUP);                  // Entrada con resistencia interna a VCC.
  Serial.println("[DEBUG-BOOT] Configurando Interrupcion de Sensor MH...");
  // Disparamos la ISR ante CUALQUIER cambio (CHANGE) y luego filtramos en el loop.
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_MH), isrFrenoCaida, CHANGE);

  // --- 1. CONEXION WIFI ---
  Serial.print("[DEBUG-BOOT] Conectando WiFi...");
  WiFi.mode(WIFI_STA);                                   // Modo estacion (nos unimos a una red).
  WiFi.begin(WIFI_SSID, WIFI_PASS);                      // Iniciamos la conexion.
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); } // Esperamos enlace.
  wifi_channel = WiFi.channel();                         // Guardamos el canal: ESP-NOW lo necesita igual.
  Serial.printf(" OK (Canal: %d)\n", wifi_channel);

  // --- 2. HANDSHAKE CON EL NODO EXPRESION (por UART) ---
  Serial.println("[DEBUG-BOOT] Iniciando Handshake con Nodo Expresion...");
  bool pingOk = false;                                   // Bandera de saludo exitoso.
  unsigned long startAttempt = millis();                 // Cronometro para el timeout.
  while (!pingOk) {
    Serial1.print("1,EXP,BOOT\n");                       // Enviamos un mensaje BOOT por cable.
    delay(500);
    if (Serial1.available()) {                           // Si Expresion contesto algo...
        String resp = Serial1.readStringUntil('\n');     // Leemos la respuesta completa.
        Serial.printf("[DEBUG-UART] RX Expresion: %s\n", resp.c_str());
        if (resp.indexOf("3,CER,ACK_EXP") != -1) pingOk = true; // Buscamos su confirmacion.
    }
    // Si tras 10 segundos no contesta, abortamos el arranque.
    if (millis() - startAttempt > 10000) haltSystem(2, "Timeout Expresion");
  }
  Serial.println("[DEBUG-BOOT] Handshake Expresion OK.");

  // --- 3. INICIALIZACION DE ESP-NOW Y ENLACE CON MOTORES ---
  Serial.println("[DEBUG-BOOT] Inicializando ESP-NOW y enlazando Nodo Motores...");
  if (esp_now_init() != ESP_OK) haltSystem(3, "Fallo ESP-NOW"); // Si no arranca, error fatal.
  esp_now_register_recv_cb(OnDataRecv);                  // Registramos el callback de recepcion.
  esp_now_peer_info_t peerInfo = {};                     // Estructura del "peer" (companero de radio).
  memcpy(peerInfo.peer_addr, slaveMAC1, 6);              // Copiamos la MAC destino (broadcast).
  peerInfo.channel = wifi_channel;                       // Mismo canal que el WiFi.
  esp_now_add_peer(&peerInfo);                           // Registramos al nodo de Motores como peer.

  msg_garra boot_msg = {1, "MOT", "BOOT"};               // Armamos el paquete de arranque.
  motor_state = "ESPERANDO";                             // Aun no sabemos si responde.
  esp_now_send(slaveMAC1, (uint8_t *)&boot_msg, sizeof(msg_garra)); // Enviamos BOOT a Motores.
  delay(1000);                                           // Damos 1 segundo para que conteste el ACK.
  if (motor_state != "OK") haltSystem(3, "Timeout Motores"); // Si no llego ACK_MOT, error fatal.
  Serial.println("[DEBUG-BOOT] Nodo Motores OK.");

  // --- 4. LECTURA INICIAL DEL COMANDO EN LA NUBE ---
  current_command = fetchCommand();                      // Sincronizamos con el estado actual de Supabase.
  Serial.printf("[DEBUG-BOOT] Comando inicial Supabase: %s\n", current_command.c_str());
  Serial.println("\n[POST COMPLETADO] Red GARRA-OS Estable.");
}

// =========================================================================
// LOOP CENTRAL
// =========================================================================
void loop() {
  // --- RUTINA CRITICA DE EMERGENCIA (maxima prioridad) ---
  if (banderaEmergencia) {
      banderaEmergencia = false;          // Bajamos la bandera de inmediato.

      // FILTRO ANTI-REBOTE OPTICO: esperamos 10 ms y reconfirmamos la lectura,
      // para no disparar el freno por un rebote o una falsa alarma momentanea.
      delay(10);
      int estadoActualSensor = digitalRead(PIN_SENSOR_MH);

      if (estadoActualSensor == ESTADO_PELIGRO) { // Si de verdad no hay piso...
          Serial.println("\n[CRITICO] SENSOR CONFIRMADO: !Perdida de suelo detectada! Ejecutando ALTOTOTAL...");

          // Mandamos el paro total por radio a Motores.
          msg_garra msg_em = {4, "ALL", "ALTOTOTAL"};
          esp_now_send(slaveMAC1, (uint8_t *)&msg_em, sizeof(msg_em));
          Serial.println("[DEBUG-RADIO] Comando ALTOTOTAL disparado a Motores.");

          // Mandamos el paro total por cable a Expresion.
          Serial1.print("4,ALL,ALTOTOTAL\n");
          Serial.println("[DEBUG-UART] Comando ALTOTOTAL disparado a Expresion.");

          // Registramos el evento de emergencia en la nube (codigo 99).
          postTelemetry(99, "EMERGENCIA_ALTOTOTAL_SENSOR_CAIDA", "TIMEOUT", "TIMEOUT");

          Serial.println("[CRITICO] Protocolo de emergencia lanzado. Bloqueando lecturas por 1 segundo...");
          delay(1000); // Bloqueo largo: evita inundar la red mientras alguien levanta el chasis.
      } else {
          Serial.println("[DEBUG-SENSOR] Falsa alarma o rebote mecanico evadido. El chasis sigue en el piso.");
      }
  }

  unsigned long currentMillis = millis();  // Tiempo actual para los temporizadores no bloqueantes.

  // --- MAQUINA DE ESTADOS DEL BUCLE PRINCIPAL ---
  switch (currentState) {
    case STATE_POLLING: // ESTADO 1: Sondeamos la nube buscando comandos nuevos.
      if (currentMillis - last_poll_time >= POLLING_INTERVAL) { // Solo cada ~1 segundo.
        last_poll_time = currentMillis;
        String fetched_cmd = fetchCommand();               // Leemos el comando de Supabase.

        // Si el comando es valido y DISTINTO al que ya teniamos, lo procesamos.
        if (fetched_cmd != "ERROR" && fetched_cmd != current_command) {
          Serial.printf("\n[DEBUG-FSM] Nuevo comando detectado: %s -> %s\n", current_command.c_str(), fetched_cmd.c_str());
          current_command = fetched_cmd;                   // Actualizamos el comando vigente.

          msg_garra msg = {2, "ALL", ""};                  // Mensaje de accion (tipo 2) para todos.
          strncpy(msg.accion, current_command.c_str(), 14);// Copiamos el texto del comando.

          motor_state      = "TIMEOUT";                    // Asumimos timeout hasta recibir ACK.
          expression_state = "TIMEOUT";

          Serial.println("[DEBUG-FSM] Enviando comando a nodos esclavos...");
          esp_now_send(slaveMAC1, (uint8_t *)&msg, sizeof(msg)); // A Motores por radio.
          Serial1.printf("%d,EXP,%s\n", msg.tipo, msg.accion);   // A Expresion por cable.

          wait_start_time = millis();                      // Arrancamos la ventana de espera.
          currentState = STATE_WAITING_DEBUG;              // Cambiamos a esperar confirmaciones.
        }
      }
      break;

    case STATE_WAITING_DEBUG: // ESTADO 2: Esperamos los ACKs de los esclavos.
      // Leemos todo lo que haya llegado por el cable serie desde Expresion.
      while (Serial1.available() > 0) {
        char c = Serial1.read();
        static char buf[64]; static int idx = 0;           // Buffer estatico de armado de linea.
        if (c == '\n') {                                    // Si es fin de linea...
          buf[idx] = '\0'; idx = 0;                         // Cerramos la cadena y reiniciamos indice.
          Serial.printf("[DEBUG-UART] RX Expresion: %s\n", buf);
          if (String(buf).indexOf("3,CER,ACK_EXP") != -1) expression_state = "OK"; // ACK valido.
        } else if (idx < 63) buf[idx++] = c;                // Vamos acumulando caracteres.
      }

      // Cuando se cumple la ventana de espera, reportamos y volvemos a sondear.
      if (millis() - wait_start_time >= WAIT_WINDOW) {
        Serial.printf("[DEBUG-FSM] Ventana de espera finalizada. Estado MOT: %s, ESTADO EXP: %s\n", motor_state.c_str(), expression_state.c_str());
        postTelemetry(0, "Comando ejecutado", motor_state, expression_state); // Subimos resultado.
        currentState = STATE_POLLING;                       // Regresamos a sondear.
      }
      break;
  }
}
