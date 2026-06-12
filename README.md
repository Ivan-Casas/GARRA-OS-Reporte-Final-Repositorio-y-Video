# GARRA-OS · Agente Robótico Autónomo de Mascota Chappie

**Materia:** Sistemas Programables · **Docente:** Ma. Verónica Tapia Ibarra
**Institución:** Instituto Tecnológico de León · Equipo **Iron Claws ITL**

| Integrante | No. de control |
|---|---|
| Alcalá Ramos Luz Estefanía | 23240079 |
| Bahena Mora Emilio Salvador | 23240009 |
| Casas Bastidas José Iván | 23240883 |
| Fischer González Patrick | 23240045 |

---

## 1. ¿Qué es GARRA-OS?

GARRA-OS es el sistema operativo distribuido que transfroma a **Chappie**, un robot de
combate en un robot mascota. En lugar de un solo microcontrolador, el robot está gobernado por **tres nodos
ESP32** que se reparten el trabajo y se comunican entre sí, más una **nube** (Supabase) y
una **interfaz web** desde la que el operador manda órdenes y observa el estado del robot.

La filosofía de diseño es **"cero lógica en el control directo"**: cada nodo expone su
hardware mediante una capa de abstracción (HAL) y se comunica con los demás únicamente con
mensajes cortos, de modo que un fallo en un subsistema no arrastra a los otros.

## 2. Arquitectura del sistema (3 capas)

```
   [ INTERFAZ WEB ]  --(UPDATE)-->  [ SUPABASE (nube) ]  <--(GET polling)--  [ NODO CEREBRO ]
        (dashboard)  <--(Realtime)--   tabla telemetria_debug                     (ESP32-S3)
                                                                                  /            \
                                                                       (ESP-NOW / radio)   (UART / cable)
                                                                              |                  |
                                                                     [ NODO EJECUCION ]   [ NODO EXPRESION ]
                                                                       2x ESC + Motores    Ventilador Holográfico
                                                                       Receptor RC R8EF     (caras / animaciones)
```

- **Nodo Cerebro (ESP32-S3):** único con Internet. Sondea los comandos en Supabase, los
  reparte a los esclavos y sube la telemetría. Además vigila el sensor de caída TCRT5000
  para disparar el paro de emergencia `ALTOTOTAL`.
- **Nodo Ejecución (ESP32 DevKit V1):** controla los dos motores 550 vía ESC. Da prioridad
  al control manual (receptor RC) y, en automático, ejecuta la rutina de baile pregrabada.
- **Nodo Expresión (ESP32 DevKit V1):** controla un ventilador holográfico por TCP/WiFi para
  proyectar las "caras" y animaciones del robot.

## 3. Estructura del repositorio

```
GARRA-OS/
├── README.md
├── HAL/                         → Capa de control de hardware (firmware de los nodos)
│   ├── nodo_cerebro/            → nodo_cerebro.ino   (ESP32-S3, maestro/nube)
│   ├── nodo_ejecucion/          → nodo_ejecucion.ino (motores + RC)
│   ├── nodo_expresion/          → nodo_expresion.ino (ventilador holográfico)
│   └── herramientas/
│       └── grabadora_macros/    → grabadora_macros.ino (genera la coreografía del baile)
├── Servidor/                    → Capa de servidor / nube
│   └── schema_supabase.sql      → Script completo de tablas, RLS y Realtime
├── Interfaz/                    → Capa de interfaz
│   └── index.html               → Dashboard de mando y telemetría
└── docs/                        → Evidencias
    ├── diagrama_conexion_chappie.jpeg
    ├── capturas_supabase/
    └── fotos_prototipo/
```

> Cada subcarpeta de `HAL/` lleva el mismo nombre que su `.ino` porque así lo exige el IDE de
> Arduino para abrir el sketch.

## 4. Materiales y equipos

| Categoría | Componente |
|---|---|
| Cómputo | ESP32-S3-WROOM-1 (Cerebro), 2× ESP32 DevKit V1 (Ejecución y Expresión) |
| Potencia motriz | 2× Motor 550, 2× ESC QuicRun 880 Brushed |
| Control RC | Receptor R8EF (8 canales) |
| Energía | Batería LiPo 14.8 V (4S) LP103665, reductores DC-DC a 12 V y 5 V (LM2596), power bank (Cerebro) |
| Sensórica | Módulo óptico TCRT5000 (sensor de caída) |
| Expresión | Ventilador holográfico (proyección de imágenes) |
| Nube | Proyecto Supabase (PostgreSQL + REST + Realtime) |

## 5. Instalación y ejecución

### 5.1 Backend (Supabase) — hacerlo primero
1. Crear un proyecto en [supabase.com](https://supabase.com).
2. Abrir **SQL Editor** y pegar/ejecutar `Servidor/schema_supabase.sql`. Esto crea las
   tablas `control_comandos` y `telemetria_debug`, las políticas RLS y activa Realtime.
3. Copiar de **Project Settings → API** la `Project URL` y la `anon public key`.

### 5.2 Firmware (los tres nodos)
1. Instalar **Arduino IDE** y el paquete de placas **ESP32** (Boards Manager → "esp32" by
   Espressif).
2. Instalar las librerías desde **Library Manager**:
   - `ArduinoJson` (Cerebro)
   - `ESP32Servo` (Ejecución y Grabadora)
   - `WiFi`, `HTTPClient`, `esp_now`, `esp_wifi` ya vienen con el core de ESP32.
3. En `nodo_cerebro.ino` colocar tu `SUPABASE_URL` y `SUPABASE_KEY`, y el SSID/clave de tu
   WiFi. En `nodo_expresion.ino` ajustar el SSID del ventilador.
4. Seleccionar la placa correcta por nodo (ESP32-S3 Dev Module / ESP32 Dev Module) y subir
   cada sketch a su microcontrolador.

> **Nota:** este proyecto **no usa Python ni un servidor propio**, por lo que **no existe**
> `requirements.txt` ni `pip install`. El "servidor" es Supabase (BaaS) y la interfaz es un
> archivo estático. La dependencia del firmware se gestiona desde el Library Manager de
> Arduino, no con pip.

### 5.3 Interfaz web
- Abrir `Interfaz/index.html` directamente en el navegador (o servirlo con cualquier
  servidor estático). Carga el SDK de Supabase desde CDN; solo necesita conexión a Internet.
- Si cambiaste de proyecto Supabase, actualizar `window.GARRA.URL` y `window.GARRA.KEY`.

## 6. Flujo de uso (demo)
1. Encender el robot y esperar el `[POST COMPLETADO]` del Cerebro en el monitor serie.
2. Abrir el dashboard: el indicador debe ponerse en verde ("Conectado · Realtime").
3. Pulsar **BAILAR / SEGUIR / ALERTA / REPOSO**. El comando viaja a la nube, el Cerebro lo
   reparte y el robot reacciona; la telemetría regresa al dashboard.
4. **Prueba de emergencia:** levantar el robot del piso → el TCRT5000 dispara `ALTOTOTAL` →
   motores a neutral y ventilador en cuarentena 30 s.

## 7. Documentación
Todos los archivos llevan encabezado con **Objetivo, Integrantes y Proyecto**, y están
comentados línea por línea. El reporte técnico completo está en
`Documentación Técnica/Reporte_Final_GARRA-OS.pdf`.
