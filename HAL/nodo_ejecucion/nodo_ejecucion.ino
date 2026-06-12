/**
 * =========================================================================
 * PROYECTO:     GARRA-OS  -  Agente Robotico Autonomo de Combate ("Chappie")
 * ARCHIVO:      nodo_ejecucion.ino
 * NODO:         EJECUCION  (ESP32 DevKit V1, 38 pines)
 *
 * OBJETIVO DEL ARCHIVO:
 *   Controlar la planta motriz del robot: dos motores 550 a traves de sus
 *   ESC QuicRun 880. Funciona en arquitectura HIBRIDA: tiene PRIORIDAD el
 *   control manual del piloto (lee el receptor RC R8EF por PWM) y, cuando el
 *   piloto suelta los sticks, acepta ordenes remotas por radio ESP-NOW desde
 *   el Cerebro (rutina de BAILE pregrabada, auto-prueba de media vuelta y el
 *   cortafuegos de emergencia ALTOTOTAL).
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
#include <WiFi.h>        // Necesaria para inicializar la radio (aunque no nos conectamos a Internet).
#include <esp_now.h>     // Protocolo ESP-NOW para recibir ordenes del Cerebro.
#include <ESP32Servo.h>  // Genera la senal PWM de 50 Hz que entienden los ESC.
#include <esp_wifi.h>    // Permite fijar manualmente el canal de radio.

// Estructura compartida por todos los nodos (debe ser identica byte a byte).
typedef struct msg_garra {
    uint8_t tipo;         // 1=BOOT, 2=ACTION, 3=ACK, 4=EMERGENCY.
    char destino[4];      // "MOT", "EXP" o "ALL".
    char accion[15];      // Texto del comando.
} msg_garra;

uint8_t mac_cerebro[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // MAC del Cerebro (broadcast).
char accion_global[15] = "IDLE";                              // Ultima accion recibida.

// --- ASIGNACION DE PINES ---
#define PIN_CH1 34       // Entrada: canal 1 del receptor RC (PWM del stick).
#define PIN_CH2 35       // Entrada: canal 2 del receptor RC (PWM del stick).
#define PIN_ESC_CH1 25   // Salida: senal hacia el ESC del motor 1.
#define PIN_ESC_CH2 26   // Salida: senal hacia el ESC del motor 2.

// --- CONSTANTES DE CONTROL DE MOTORES ---
const uint16_t PWM_NEUTRAL    = 1500; // Microsegundos = motor detenido (punto medio del ESC).
const uint16_t DEADBAND       = 45;   // Zona muerta: ignora micro-movimientos del stick (ruido).
const uint16_t PWM_GIRO_SUAVE = 1620; // Velocidad suave para la auto-prueba de giro.
const unsigned long TIEMPO_MACRO = 800; // Duracion (ms) de la macro de media vuelta.

// =========================================================================
// MACRO "BAILE": coreografia pregrabada con la herramienta Grabadora.
// =========================================================================
const unsigned long DURACION_BAILE = 17000; // El baile completo dura 17 segundos.
const int POTENCIA_BAILE_PORCENTAJE = 35;   // Atenuamos al 35% la energia original (mas seguro en demo).

// Cada fila {chA, chB} es una "foto" de los dos sticks tomada cada 100 ms
// durante la grabacion. El robot reproducira estas fotos en secuencia.
const uint16_t MACRO_BAILE[][2] = {
  {1500, 1500}, {1473, 1520}, {1468, 1531}, {1466, 1549}, {1308, 1691},
  {1186, 1844}, {1056, 1942}, {1067, 1932}, {1279, 1703}, {1480, 1508},
  {1486, 1507}, {1483, 1509}, {1481, 1505}, {1482, 1507}, {1491, 1520},
  {1575, 1640}, {1743, 1819}, {1864, 1901}, {1941, 1947}, {1946, 1948},
  {1767, 1730}, {1583, 1564}, {1481, 1502}, {1469, 1505}, {1465, 1499},
  {1355, 1293}, {1054, 1081}, {1000, 1028}, {1000, 1019}, {1029, 1077},
  {1174, 1352}, {1489, 1508}, {1489, 1505}, {1487, 1500}, {1492, 1495},
  {1567, 1423}, {1739, 1238}, {1852, 1122}, {1980, 1000}, {1994, 1000},
  {1753, 1229}, {1497, 1501}, {1467, 1504}, {1473, 1517}, {1475, 1519},
  {1493, 1518}, {1533, 1560}, {1636, 1684}, {1789, 1847}, {1873, 1904},
  {1894, 1937}, {1914, 1915}, {1795, 1695}, {1481, 1500}, {1467, 1506},
  {1470, 1511}, {1480, 1522}, {1480, 1527}, {1455, 1552}, {1311, 1739},
  {1122, 1881}, {1037, 1995}, {1000, 2000}, {1129, 1867}, {1427, 1567},
  {1483, 1508}, {1486, 1509}, {1485, 1510}, {1484, 1511}, {1676, 1320},
  {1855, 1103}, {1976, 1000}, {1992, 1000}, {1848, 1122}, {1495, 1515},
  {1478, 1515}, {1484, 1508}, {1473, 1502}, {1444, 1476}, {1357, 1310},
  {1049, 1057}, {1000, 1039}, {1086, 1160}, {1377, 1432}, {1540, 1576},
  {1599, 1590}, {1538, 1496}, {1468, 1505}, {1476, 1509}, {1493, 1517},
  {1508, 1528}, {1581, 1635}, {1884, 1938}, {2000, 2000}, {2000, 2000},
  {1954, 1837}, {1537, 1500}, {1470, 1505}, {1494, 1518}, {1516, 1525},
  {1558, 1548}, {1583, 1513}, {1583, 1510}, {1583, 1510}, {2000, 1000},
  {1852, 1132}, {1590, 1496}, {1469, 1508}, {1481, 1504}, {1479, 1504},
  {1461, 1515}, {1308, 1671}, {1120, 1896}, {1060, 1959}, {1099, 1892},
  {1312, 1578}, {1486, 1507}, {1491, 1509}, {1488, 1513}, {1475, 1515},
  {1460, 1511}, {1370, 1388}, {1161, 1137}, {1000, 1000}, {1000, 1000},
  {1011, 1057}, {1128, 1227}, {1490, 1516}, {1492, 1513}, {1468, 1512},
  {1464, 1512}, {1465, 1511}, {1478, 1498}, {1540, 1407}, {1835, 1150},
  {1957, 1016}, {1980, 1000}, {1846, 1143}, {1613, 1389}, {1491, 1510},
  {1473, 1511}, {1467, 1506}, {1474, 1501}, {1487, 1508}, {1496, 1507},
  {1503, 1519}, {1605, 1633}, {1752, 1786}, {1862, 1888}, {1937, 1968},
  {1999, 2000}, {1973, 1922}, {1756, 1734}, {1588, 1576}, {1498, 1516},
  {1479, 1502}, {1475, 1502}, {1479, 1504}, {1472, 1514}, {1464, 1515},
  {1460, 1514}, {1459, 1513}, {1447, 1436}, {1269, 1230}, {1000, 1000},
  {1000, 1000}, {1000, 1019}, {1189, 1348}, {1460, 1503}, {1481, 1508},
  {1485, 1511}, {1466, 1521}, {1173, 1782}, {1000, 2000}, {1000, 2000},
  {1199, 1772}, {1500, 1500}
};

// Numero total de fotogramas: tamano total del arreglo / tamano de una fila.
const int ELEMENTOS_BAILE = sizeof(MACRO_BAILE) / sizeof(MACRO_BAILE[0]);

// Estados posibles del nodo de Motores.
enum EstadoFSM { MODO_MANUAL, MACRO_MEDIA_VUELTA, RUTINA_BAILE };
EstadoFSM estado_actual = MODO_MANUAL;          // Arrancamos en control manual.

unsigned long macro_start_time     = 0;         // Inicio de la macro de media vuelta.
unsigned long last_auto_test_time  = 0;         // Ultima vez que hubo actividad (para el auto-test).
unsigned long inicio_rutina_baile  = 0;         // Inicio de la rutina de baile.

volatile bool en_emergencia   = false;          // Bandera de paro de emergencia.
unsigned long tiempo_emergencia = 0;            // Marca de tiempo del inicio de la cuarentena.

Servo esc_ch1, esc_ch2;                         // Objetos que generan el PWM de cada ESC.

// =========================================================================
// CALLBACK ESP-NOW (Recepcion de ordenes del Cerebro)
// =========================================================================
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    if (len != sizeof(msg_garra)) return;              // Descartamos paquetes de tamano incorrecto.
    msg_garra *msg = (msg_garra *)incomingData;        // Reinterpretamos los bytes recibidos.

    // Solo atendemos mensajes dirigidos a Motores ("MOT") o a todos ("ALL").
    if (strcmp(msg->destino, "MOT") == 0 || strcmp(msg->destino, "ALL") == 0) {
        strncpy(accion_global, msg->accion, 14);       // Guardamos la accion recibida.

        // 1. CORTAFUEGOS: ALTOTOTAL tiene prioridad maxima sobre todo lo demas.
        if (strcmp(accion_global, "ALTOTOTAL") == 0 && !en_emergencia) {
            en_emergencia = true;                      // Activamos el modo emergencia.
            tiempo_emergencia = millis();              // Arrancamos el cronometro de cuarentena.
            Serial.println("\n[CRITICO] ALTOTOTAL RECIBIDO. Activando cortafuegos.");
        }
        // 2. DISPARADOR: orden de BAILAR (solo si no estamos en emergencia).
        else if (strcmp(accion_global, "BAILAR") == 0 && !en_emergencia) {
            if (estado_actual != RUTINA_BAILE) {       // Evitamos reiniciar un baile en curso.
                estado_actual = RUTINA_BAILE;
                inicio_rutina_baile = millis();
                Serial.println("\n[FSM] INICIANDO RUTINA MACRO: BAILE (Duracion: 17s)");
            }
        }

        // Respondemos un ACK inmediato al Cerebro para confirmar recepcion.
        msg_garra ack = {3, "CER", "ACK_MOT"};
        esp_now_send(mac_cerebro, (uint8_t *)&ack, sizeof(ack));
    }
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);                              // Consola de depuracion.

  WiFi.mode(WIFI_STA);                               // Activamos la radio en modo estacion.
  // Buscamos en que canal esta la red "honor" para que ESP-NOW coincida con el Cerebro.
  int32_t canal = 1;
  int n = WiFi.scanNetworks();
  for(int i=0; i<n; i++) if(WiFi.SSID(i) == "honor") canal = WiFi.channel(i);
  esp_wifi_set_channel(canal, WIFI_SECOND_CHAN_NONE);// Fijamos ese canal manualmente.

  if (esp_now_init() == ESP_OK) {                    // Inicializamos ESP-NOW.
      esp_now_register_recv_cb(OnDataRecv);          // Registramos el callback de recepcion.
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, mac_cerebro, 6);        // Registramos al Cerebro como peer.
      peer.channel = canal;
      esp_now_add_peer(&peer);
  }

  // --- CONFIGURACION DE LOS ESC (PWM de 50 Hz, pulso de 1000-2000 us) ---
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); // Reservamos temporizadores hardware.
  esc_ch1.setPeriodHertz(50); esc_ch2.setPeriodHertz(50); // Frecuencia estandar de servo/ESC.
  esc_ch1.attach(PIN_ESC_CH1, 1000, 2000);                // Asociamos el pin y el rango de pulso.
  esc_ch2.attach(PIN_ESC_CH2, 1000, 2000);
  pinMode(PIN_CH1, INPUT); pinMode(PIN_CH2, INPUT);       // Pines del receptor RC como entradas.

  Serial.println("[SISTEMA] Nodo Motores Operativo.");
}

// =========================================================================
// LOOP
// =========================================================================
void loop() {
  unsigned long currentMillis = millis();            // Tiempo actual (temporizadores no bloqueantes).

  // =========================================================
  // CORTAFUEGOS DE JERARQUIA ABSOLUTA (ALTOTOTAL)
  // =========================================================
  if (en_emergencia) {
      if (currentMillis - tiempo_emergencia < 30000) {   // Durante 30 s de cuarentena...
          esc_ch1.writeMicroseconds(PWM_NEUTRAL);        // ...forzamos motores a NEUTRAL (parados).
          esc_ch2.writeMicroseconds(PWM_NEUTRAL);
          return;                                        // Ignoramos cualquier otra logica.
      } else {
          en_emergencia = false;                         // Termina la cuarentena.
          last_auto_test_time = currentMillis;
          Serial.println("[SISTEMA] Cuarentena finalizada. Retornando control a MODO_MANUAL.");
      }
  }

  // =========================================================
  // LECTURA DEL CONTROL MANUAL (tiene prioridad sobre lo automatico)
  // =========================================================
  // pulseIn mide el ancho del pulso PWM que envia el receptor RC (timeout 25 ms).
  unsigned long raw_ch1 = pulseIn(PIN_CH1, HIGH, 25000);
  unsigned long raw_ch2 = pulseIn(PIN_CH2, HIGH, 25000);

  // Validamos rango: si la lectura es basura (radio apagada), usamos NEUTRAL.
  uint16_t ch1 = (raw_ch1 > 800 && raw_ch1 < 2200) ? (uint16_t)raw_ch1 : PWM_NEUTRAL;
  uint16_t ch2 = (raw_ch2 > 800 && raw_ch2 < 2200) ? (uint16_t)raw_ch2 : PWM_NEUTRAL;

  // El operador esta "activo" si mueve algun stick mas alla de la zona muerta.
  bool operador_activo = (abs(ch1 - PWM_NEUTRAL) > DEADBAND) || (abs(ch2 - PWM_NEUTRAL) > DEADBAND);

  if (operador_activo) {                               // Si el piloto toma el control...
      if (estado_actual == RUTINA_BAILE) {
          Serial.println("[OVERRIDE] Operador activo. Abortando rutina BAILE.");
      }
      estado_actual = MODO_MANUAL;                     // ...cancelamos cualquier rutina automatica.
      last_auto_test_time = currentMillis;
  }

  // =========================================================
  // MAQUINA DE ESTADOS FINITOS (FSM)
  // =========================================================
  switch (estado_actual) {
    case MODO_MANUAL: // El robot obedece directamente al piloto.
      esc_ch1.writeMicroseconds(ch1);                  // Enviamos el pulso del stick al ESC 1.
      esc_ch2.writeMicroseconds(ch2);                  // Enviamos el pulso del stick al ESC 2.
      // Si lleva 20 s inactivo, lanza una pequena auto-prueba de giro (senal de vida).
      if (!operador_activo && (currentMillis - last_auto_test_time >= 20000)) {
        estado_actual = MACRO_MEDIA_VUELTA;
        macro_start_time = currentMillis;
      }
      break;

    case MACRO_MEDIA_VUELTA: // Auto-prueba: gira suavemente durante TIEMPO_MACRO.
      if (currentMillis - macro_start_time < TIEMPO_MACRO) {
        esc_ch1.writeMicroseconds(PWM_GIRO_SUAVE);     // Un motor avanza...
        esc_ch2.writeMicroseconds(1500);               // ...el otro queda quieto -> giro sobre su eje.
      } else {
        estado_actual = MODO_MANUAL;                   // Termina y regresa a manual.
        last_auto_test_time = currentMillis;
      }
      break;

    case RUTINA_BAILE: { // Reproduce la coreografia pregrabada MACRO_BAILE.
      // Tomamos un tiempo fresco y blindamos contra "underflow" (resta negativa).
      unsigned long tiempo_real = millis();
      if (tiempo_real < inicio_rutina_baile) tiempo_real = inicio_rutina_baile;

      unsigned long tiempo_transcurrido = tiempo_real - inicio_rutina_baile;

      if (tiempo_transcurrido < DURACION_BAILE) {      // Mientras dure el baile...
          last_auto_test_time = tiempo_real;           // Reseteamos el contador de inactividad.

          // Calculamos que fotograma corresponde a este milisegundo exacto.
          int frame_actual = (tiempo_transcurrido * ELEMENTOS_BAILE) / DURACION_BAILE;
          if (frame_actual >= ELEMENTOS_BAILE) frame_actual = ELEMENTOS_BAILE - 1; // Limite de seguridad.

          // Extraemos los valores crudos grabados para ese fotograma.
          int pwm1_crudo = MACRO_BAILE[frame_actual][0];
          int pwm2_crudo = MACRO_BAILE[frame_actual][1];

          // Atenuamos la potencia con matematica de enteros (centrada en 1500).
          uint16_t pwm1_suave = 1500 + ((pwm1_crudo - 1500) * POTENCIA_BAILE_PORCENTAJE / 100);
          uint16_t pwm2_suave = 1500 + ((pwm2_crudo - 1500) * POTENCIA_BAILE_PORCENTAJE / 100);

          esc_ch1.writeMicroseconds(pwm1_suave);       // Aplicamos al ESC 1.
          esc_ch2.writeMicroseconds(pwm2_suave);       // Aplicamos al ESC 2.
      }
      else {                                           // Cuando termina el baile...
          Serial.println("[FSM] Rutina BAILE finalizada (17s). Retornando a MODO_MANUAL.");
          estado_actual = MODO_MANUAL;
          last_auto_test_time = millis();
      }
      break;
    }
  }
}
