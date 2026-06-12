-- =========================================================================
-- PROYECTO:     GARRA-OS  -  Agente Robotico Autonomo de Combate ("Chappie")
-- ARCHIVO:      schema_supabase.sql   (CAPA SERVIDOR / NUBE)
--
-- OBJETIVO DEL ARCHIVO:
--   Definir por completo el backend del sistema en Supabase (PostgreSQL).
--   Crea las dos tablas que sostienen la comunicacion bidireccional con el
--   robot, sus politicas de seguridad (RLS) y la configuracion de Realtime
--   para que el dashboard web reciba la telemetria al instante. Se ejecuta
--   una sola vez desde el SQL Editor de Supabase.
--
-- ARQUITECTURA: IoT 3-Tier (Frontend Web -> Supabase REST/Realtime <- ESP32-S3)
--
-- INTEGRANTES:
--   - Alcala Ramos Luz Estefania      (23240079)
--   - Bahena Mora Emilio Salvador     (23240009)
--   - Casas Bastidas Jose Ivan        (23240883)
--   - Fischer Gonzalez Patrick        (23240045)
--
-- MATERIA:      Sistemas Programables
-- DOCENTE:      Ma. Veronica Tapia Ibarra
-- INSTITUCION:  Instituto Tecnologico de Leon
-- =========================================================================


-- 1. FUNCION DE UTILIDAD: Auto-actualizacion del Timestamp
-- Esta funcion es clave: permite que el ESP32 sepa si un comando es nuevo
-- comparando el 'updated_at' contra su ultima lectura.
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = now();   -- Sella la fila con la hora exacta del cambio.
    RETURN NEW;
END;
$$ language 'plpgsql';


-- =========================================================================
-- TABLA 1: control_comandos   (Flujo: Web -> ESP32)
-- Diseno de "Fila Unica": una sola fila (id=1) guarda el comando vigente.
-- Asi el ESP32 hace un GET muy ligero por HTTP en cada sondeo (polling).
-- =========================================================================
CREATE TABLE IF NOT EXISTS public.control_comandos (
    id INT PRIMARY KEY,                          -- Siempre sera 1 (clave de la fila maestra).
    comando VARCHAR(50) NOT NULL DEFAULT 'IDLE', -- Comando actual: BAILAR, SEGUIR, ALERTA, REPOSO...
    parametros JSONB DEFAULT '{}'::jsonb,        -- Datos extra opcionales en formato JSON.
    updated_at TIMESTAMPTZ DEFAULT now()         -- Fecha/hora del ultimo cambio.
);

-- Insertamos la fila maestra que el ESP32 leera siempre (id=1).
-- ON CONFLICT DO NOTHING evita duplicarla si el script se corre dos veces.
INSERT INTO public.control_comandos (id, comando, parametros)
VALUES (1, 'IDLE', '{}')
ON CONFLICT (id) DO NOTHING;

-- Disparador que mantiene 'updated_at' al dia automaticamente en cada UPDATE.
CREATE TRIGGER update_control_comandos_modtime
    BEFORE UPDATE ON public.control_comandos
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();


-- =========================================================================
-- TABLA 2: telemetria_debug   (Flujo: ESP32 -> Web)
-- Diseno de Bitacora (solo se agregan filas) para guardar el historial de
-- estados y disparar eventos Realtime que el dashboard escucha en vivo.
-- =========================================================================
CREATE TABLE IF NOT EXISTS public.telemetria_debug (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),    -- Identificador unico por registro.
    estado_maestro VARCHAR(50) DEFAULT 'OFFLINE',     -- Estado del nodo Cerebro.
    estado_motores VARCHAR(50) DEFAULT 'STOP',        -- Estado reportado del nodo Motores.
    estado_expresion VARCHAR(50) DEFAULT 'NEUTRAL',   -- Estado reportado del nodo Expresion.
    codigo_error INT DEFAULT 0,                        -- 0 = OK; >0 = codigo de falla.
    mensaje TEXT DEFAULT '',                           -- Descripcion legible del evento.
    updated_at TIMESTAMPTZ DEFAULT now()               -- Momento del registro.
);


-- =========================================================================
-- 3. SEGURIDAD Y PERMISOS (RLS - Row Level Security)
-- Entorno de laboratorio/prototipo: acceso total usando la llave anonima.
-- =========================================================================

-- Habilitamos RLS (Supabase lo exige antes de definir politicas).
ALTER TABLE public.control_comandos ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.telemetria_debug ENABLE ROW LEVEL SECURITY;

-- Politica para control_comandos: el rol anonimo puede leer y escribir.
CREATE POLICY "Permitir acceso total anonimo en control_comandos"
ON public.control_comandos
FOR ALL
TO anon
USING (true)
WITH CHECK (true);

-- Politica para telemetria_debug: el rol anonimo puede leer y escribir.
CREATE POLICY "Permitir acceso total anonimo en telemetria_debug"
ON public.telemetria_debug
FOR ALL
TO anon
USING (true)
WITH CHECK (true);


-- =========================================================================
-- 4. CONFIGURACION DE SUPABASE REALTIME
-- Habilita la propagacion de eventos (INSERT/UPDATE/DELETE) de la telemetria
-- para que el Frontend reciba cada cambio sin recargar la pagina.
-- =========================================================================
ALTER PUBLICATION supabase_realtime ADD TABLE public.telemetria_debug;

-- =========================================================================
-- FIN DEL SCRIPT
-- =========================================================================
