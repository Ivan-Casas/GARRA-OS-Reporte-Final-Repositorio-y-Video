/**
 * =========================================================================
 * PROYECTO:     GARRA-OS  -  Agente Robotico Autonomo de Combate ("Chappie")
 * ARCHIVO:      grabadora_macros.ino   (HERRAMIENTA DE DESARROLLO - TEMPORAL)
 * NODO:         se carga en el mismo ESP32 de EJECUCION (DevKit V1)
 *
 * OBJETIVO DEL ARCHIVO:
 *   Firmware AUXILIAR que NO forma parte del producto final. Sirve para
 *   "grabar" una coreografia: el piloto mueve los sticks del control RC y este
 *   programa imprime por el monitor serie, cada 100 ms, los valores de los dos
 *   canales en formato { ch1, ch2 }. Esa lista se copia y pega en el arreglo
 *   MACRO_BAILE del archivo nodo_ejecucion.ino. Por eso se documenta y se
 *   conserva en la carpeta de herramientas: explica COMO se genero el baile.
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

#include <WiFi.h>        // Inicializa la radio (necesaria para ESP-NOW).
#include <esp_now.h>     // Recepcion del paro de emergencia mientras se graba.
#include <ESP32Servo.h>  // Mantiene los ESC en neutral cuando no se graba.
#include <esp_wifi.h>    // Fija el canal de radio.

// Estructura compartida (identica a la de los demas nodos).
typedef struct msg_garra {
    uint8_t tipo;
    char destino[4];
    char accion[15];
} msg_garra;

uint8_t mac_cerebro[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // MAC del Cerebro (broadcast).
char accion_global[15] = "IDLE";

#define PIN_CH1 34       // Entrada canal 1 del receptor RC.
#define PIN_CH2 35       // Entrada canal 2 del receptor RC.
#define PIN_ESC_CH1 25   // Salida hacia ESC 1.
#define PIN_ESC_CH2 26   // Salida hacia ESC 2.

const uint16_t PWM_NEUTRAL = 1500; // Pulso de motor detenido.

volatile bool en_emergencia = false; // Bandera de paro de emergencia.
unsigned long tiempo_emergencia = 0;

Servo esc_ch1, esc_ch2;              // Objetos PWM de los ESC.

// =========================================================================
// VARIABLES DE LA GRABADORA (lectura de pulsos por interrupcion)
// =========================================================================
volatile unsigned long ch1_start = 0; // Momento (us) en que empezo el pulso del canal 1.
volatile uint16_t ch1_val = 1500;     // Ancho medido del pulso del canal 1.

volatile unsigned long ch2_start = 0; // Momento (us) en que empezo el pulso del canal 2.
volatile uint16_t ch2_val = 1500;     // Ancho medido del pulso del canal 2.

bool grabando = false;                // True mientras se esta capturando la coreografia.
unsigned long last_sample_time = 0;   // Marca de tiempo de la ultima muestra impresa.

// =========================================================================
// ISR: LECTURA DE PULSOS NO BLOQUEANTE
// =========================================================================
// A diferencia de pulseIn (que bloquea), medimos el pulso por interrupciones:
// en el flanco de subida guardamos el tiempo de inicio; en el de bajada
// calculamos la duracion. Asi la lectura no congela el bucle principal.
void IRAM_ATTR isr_ch1() {
    if (digitalRead(PIN_CH1) == HIGH) ch1_start = micros();      // Flanco de subida.
    else ch1_val = (uint16_t)(micros() - ch1_start);            // Flanco de bajada -> duracion.
}

void IRAM_ATTR isr_ch2() {
    if (digitalRead(PIN_CH2) == HIGH) ch2_start = micros();
    else ch2_val = (uint16_t)(micros() - ch2_start);
}

// =========================================================================
// CALLBACK ESP-NOW (Seguridad: detener grabacion ante emergencia)
// =========================================================================
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    if (len != sizeof(msg_garra)) return;
    msg_garra *msg = (msg_garra *)incomingData;

    if (strcmp(msg->destino, "MOT") == 0 || strcmp(msg->destino, "ALL") == 0) {
        strncpy(accion_global, msg->accion, 14);

        if (strcmp(accion_global, "ALTOTOTAL") == 0 && !en_emergencia) {
            en_emergencia = true;
            grabando = false;                // Si hay emergencia, abortamos la grabacion.
            tiempo_emergencia = millis();
            Serial.println("\n[CRITICO] ALTOTOTAL RECIBIDO. Grabacion detenida.");
        }

        msg_garra ack = {3, "CER", "ACK_MOT"}; // Confirmamos al Cerebro.
        esp_now_send(mac_cerebro, (uint8_t *)&ack, sizeof(ack));
    }
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    int32_t canal = 1;
    int n = WiFi.scanNetworks();
    for(int i=0; i<n; i++) if(WiFi.SSID(i) == "honor") canal = WiFi.channel(i);
    esp_wifi_set_channel(canal, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(OnDataRecv);
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, mac_cerebro, 6);
        peer.channel = canal;
        esp_now_add_peer(&peer);
    }

    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    esc_ch1.setPeriodHertz(50); esc_ch2.setPeriodHertz(50);
    esc_ch1.attach(PIN_ESC_CH1, 1000, 2000);
    esc_ch2.attach(PIN_ESC_CH2, 1000, 2000);

    pinMode(PIN_CH1, INPUT);
    pinMode(PIN_CH2, INPUT);

    // Conectamos las interrupciones a ambos canales (ante cualquier cambio de nivel).
    attachInterrupt(digitalPinToInterrupt(PIN_CH1), isr_ch1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_CH2), isr_ch2, CHANGE);

    Serial.println("\n[GRABADORA LISTA] Escribe 'REC' para iniciar, 'STOP' para detener.");
}

void loop() {
    unsigned long currentMillis = millis();

    // 1. CORTAFUEGOS (maxima prioridad): mantiene motores quietos en emergencia.
    if (en_emergencia) {
        if (currentMillis - tiempo_emergencia < 30000) {
            esc_ch1.writeMicroseconds(PWM_NEUTRAL);
            esc_ch2.writeMicroseconds(PWM_NEUTRAL);
            return;
        } else {
            en_emergencia = false;
            Serial.println("[SISTEMA] Cuarentena finalizada.");
        }
    }

    // 2. GATILLO POR TECLADO: comandos 'REC' y 'STOP' desde el monitor serie.
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();                                 // Quitamos espacios y saltos sobrantes.

        if (input == "REC" && !en_emergencia) {
            grabando = true;
            last_sample_time = currentMillis;
            Serial.println("// --- INICIO GRABACION ---");
        }
        else if (input == "STOP") {
            grabando = false;
            Serial.println("// --- FIN GRABACION ---");
        }
    }

    // 3. GENERADOR DE SALIDA A 10 Hz (una muestra cada 100 ms).
    if (grabando) {
        if (currentMillis - last_sample_time >= 100) {
            last_sample_time += 100;                  // Avance fijo de 100 ms (cadencia estable).

            // Acotamos los valores al rango valido de los ESC.
            uint16_t c1 = constrain(ch1_val, 1000, 2000);
            uint16_t c2 = constrain(ch2_val, 1000, 2000);

            // Imprimimos en el formato exacto del arreglo MACRO_BAILE: "  {c1, c2},".
            Serial.printf("  {%u, %u},\n", c1, c2);
        }
    } else {
        // En reposo mantenemos los motores en neutral por seguridad.
        esc_ch1.writeMicroseconds(PWM_NEUTRAL);
        esc_ch2.writeMicroseconds(PWM_NEUTRAL);
    }
}
