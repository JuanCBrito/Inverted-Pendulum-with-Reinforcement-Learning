#!/usr/bin/env python3
"""
lqr_control_with_serial.py
Versión integrada: LQR + funciones de lectura/escritura serial extraídas del entorno pygame.
- Lee telemetría de Arduino: 4 floats por línea
- Calcula control (LQR / swing-up) y envía comando con formato: 0"voltaje"
- Usa time.time() en Python para dt
"""

import serial
import time
import math
import sys

# ---------------------------
# CONFIGURACIÓN (ajusta según tu setup)
SERIAL_PORT = "COM3"       # ejemplo Windows; en Linux /dev/ttyACM0 o /dev/ttyUSB0
BAUDRATE = 115200
TIMEOUT = 0.5              # segundos para lectura serial
REQUEST_PREFIX = b'R\n'    # comando que pide telemetría al Arduino (si aplica)
VBUS = 10.0                # voltaje de alimentación para conversión PWM->V
ANGLE_IN_DEGREES = True    # True si Arduino manda ángulo/vel en grados
SERIAL_ORDER = "x,theta,v,w"

# Si tu Arduino no requiere que pidamos telemetría con 'R', pon False
WRITE_REQUEST_BEFORE_READ = True

# ---------------------------
# Constantes físicas (copiadas del .ino)
Mt = 0.1901
Cfriction = 0.63
Bfriction = 0.00007892
Jp = 0.00517333
mp = 0.097
lp = 0.2
g = 9.81
mplp = mp * lp
INERTIA_EQ = Jp + mplp * lp
MGL = mplp * g
desiredEnergy = 2 * MGL
armatureResistance = 4.3
pulleyradius = 0.012
kt = 0.219848705

# Parámetros de encoders (no necesarios en Python si Arduino ya envía datos)
MOTOR_ENCODER_PPR = 2400
PENDULUM_ENCODER_PPR = 2400
SHAFT_R = 0.00573

# Control/limites
MAX_STALL_U = 140
POSITION_LIMIT = 4500
THETA_THRESHOLD = math.pi / 12.0

# Setpoint
cartpositionsetpoint = 0.0
#anglepositionsetpoint = math.pi
anglepositionsetpoint = 0.0

# Ganancias LQR (ajusta aquí si quieres)
# LQR_K = [2200.0, 108.0, -12.0, -9.5]
LQR_K = [-3000, -800.0, 0.04, 0.01]
kfeedbacklinearization = 1.0
filter_alpha = 0.5

# ---------------------------
# Helpers
def signof(x):
    if x > 0: return 1.0
    if x < 0: return -1.0
    return 0.0

def saturate(v, maxValue):
    return (maxValue if v > 0 else -maxValue) if abs(v) > maxValue else v

def avoidStall(u):
    if abs(u) < MAX_STALL_U:
        return (2 + MAX_STALL_U) if u > 0 else (-2 - MAX_STALL_U)
    return u

def normalizeAngle(angle):
    angle = math.fmod(angle, 2.0 * math.pi)
    if angle < 0.0:
        angle += 2.0 * math.pi
    return angle


# ---------------------------
# Serial helpers (extraídos y adaptados del entorno pygame)
def init_serial(port=SERIAL_PORT, baud=BAUDRATE, timeout=TIMEOUT):
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(1.0)  # espera a que el Arduino reinicie si aplica
        return ser
    except Exception as e:
        print("Error abriendo puerto serial:", e)
        raise

def parse_4_floats_from_line(line):
    """Intenta convertir una línea 'a,b,c,d' en 4 floats. Levanta ValueError si falla."""
    parts = line.strip().split(',')
    if len(parts) < 4:
        raise ValueError("No hay 4 valores en la línea")
    return list(map(float, parts[:4]))

def get_state(ser, tries=10, send_request=WRITE_REQUEST_BEFORE_READ):
    """
    Lee la telemetría del Arduino y devuelve un dict con:
      { "angle_rad": ..., "w_rad_s": ..., "x": ..., "v": ... }
    Soporta SERIAL_ORDER configurable:
      - "x,theta,v,w"  -> map: raw[0]=x, raw[1]=theta_deg(or rad), raw[2]=v, raw[3]=w
      - "x,v,theta,w"  -> map: raw[0]=x, raw[1]=v, raw[2]=theta_deg(or rad), raw[3]=w
    Retries varias veces si no llegan datos o línea inválida.
    """
    for attempt in range(tries):
        try:
            if send_request:
                # pedirle al Arduino que mande una línea
                try:
                    ser.write(REQUEST_PREFIX)
                except:
                    pass
            raw = ser.readline().decode('ascii', errors='ignore').strip()
            if not raw:
                # No vino nada
                time.sleep(0.001)
                continue
            # intentar parsear 4 floats
            values = parse_4_floats_from_line(raw)
            # mapear según orden
            if SERIAL_ORDER == "x,theta,v,w":
                x_raw = values[0]
                theta_raw = values[1]
                v_raw = values[2]
                w_raw = values[3]
            elif SERIAL_ORDER == "x,v,theta,w":
                x_raw = values[0]
                v_raw = values[1]
                theta_raw = values[2]
                w_raw = values[3]
            else:
                # Si el usuario puso otra cosa, intentar heurística mínima:
                # asumimos que si values[1] tiene magnitud > 3 entonces es un ángulo en grados.
                if abs(values[1]) > 3:
                    x_raw = values[0]; theta_raw = values[1]; v_raw = values[2]; w_raw = values[3]
                else:
                    x_raw = values[0]; v_raw = values[1]; theta_raw = values[2]; w_raw = values[3]

            if ANGLE_IN_DEGREES:
                angle_rad = math.radians(theta_raw)
                w_rad_s = math.radians(w_raw)
            else:
                angle_rad = theta_raw
                w_rad_s = w_raw

            return {"angle_rad": angle_rad, "w_rad_s": w_rad_s, "x": x_raw, "v": v_raw}

        except ValueError:
            # línea no con formato correcto, intentar otra vez
            time.sleep(0.001)
            continue
        except Exception as e:
            # otros errores (por ejemplo decode), continuar
            # print("get_state error:", e)
            time.sleep(0.001)
            continue

    # Si no se pudo leer nada válido, devolver ceros (para seguridad)
    return {"angle_rad": 0.0, "w_rad_s": 0.0, "x": 0.0, "v": 0.0}

# ---------------------------
# Comandos para Arduino
def format_command_voltage(voltage):
    """Formatea: 0"voltaje"\n (dos decimales)."""
    return '0"{:.2f}"\n'.format(voltage)
def _format_voltage_value_no_quotes(voltage):
    """
    Formatea el valor siguiendo las reglas solicitadas:
      3     -> "03"       (positivo: entero con padding a 2 dígitos)
      2.5   -> "02.5"
      0     -> "00"
      -2.4  -> "-2.4"     (negativo: conserva el signo, sin padding en la parte entera)
      12.0  -> "12"
    Devuelve la parte numérica (sin el prefijo '0').
    """
    abs_v = abs(voltage)
    integer = int(abs_v)
    frac = abs_v - integer

    # Construimos la parte decimal si existe (hasta 3 decimales, sin ceros finales)
    if frac < 1e-9:
        frac_str = ""
    else:
        # formateamos la fracción con 3 decimales y recortamos ceros finales
        frac_str = ("{:.3f}".format(frac))[1:]  # genera ".500", ".250", etc.
        frac_str = frac_str.rstrip('0').rstrip('.')  # ".5" , ".25"
    
    if voltage >= 0:
        # positivo: entero con padding a 2 dígitos
        int_str = f"{integer:02d}"
        return int_str + frac_str
    else:
        # negativo: signo + entero (sin padding) + fracción
        return "-" + str(integer) + frac_str
def send_voltage_command(ser, voltage):
    """
    Envía por serial: prefijo '0' + valor formateado + '\\n'
    Ejemplos:
      send_voltage_command(ser, 3.0)   -> envía b"003\n"
      send_voltage_command(ser, 2.5)   -> envía b"002.5\n"
      send_voltage_command(ser, -2.4)  -> envía b"0-2.4\n"
      send_voltage_command(ser, 0.0)   -> envía b"000\n"
    """
    value_str = _format_voltage_value_no_quotes(voltage)
    cmd = f"0{value_str}\n".encode('ascii')
    ser.write(cmd)
    try:
        ser.flush()
    except:
        pass

def send_pwm_command(ser, pwm):
    """Envía PWM (entero 0..255) con la misma estructura '0"pwm"' por compatibilidad."""
    pwm_i = int(round(saturate(pwm, 255.0)))
    cmd = '0"{}"\n'.format(pwm_i).encode('ascii')
    ser.write(cmd)
    try:
        ser.flush()
    except:
        pass

def center(ser):
    """Comando rápido para centrar (usa '10' como en tu entorno)."""
    try:
        ser.write(b"00\n")
    except:
        pass

def reset_to_down(ser, timeout=15.0):
    """
    Envía 'R' continuamente y espera hasta que el péndulo esté abajo (~180°) durante 2s.
    Timeout en segundos para no bloquear indefinidamente.
    """
    start_global = time.time()
    start_down_time = None
    while True:
        try:
            ser.write(b'R\n')
        except:
            pass
        state = get_state(ser, tries=5, send_request=False)
        # si ANGLE_IN_DEGREES True, angle en rad -> convertir a deg para comparar
        angle_deg = math.degrees(state["angle_rad"])
        # Normalizar a [-180,180)
        ang_norm = ((angle_deg + 180) % 360) - 180
        # Consideramos "abajo" alrededor de ±180 o 0? En tu entorno usabas ~180
        if abs(abs(ang_norm) - 180) < 4:  # dentro de ~4 grados de 180/-180
            if start_down_time is None:
                start_down_time = time.time()
            elif time.time() - start_down_time >= 2.0:
                return True
        else:
            start_down_time = None

        if time.time() - start_global > timeout:
            return False
        time.sleep(0.01)


# Lógica de control (idéntica a la anterior)
def compute_control(state):
    theta = state["angle_rad"]
    w_filtered = state["w_rad_s"]
    x = state["x"]
    v_filtered = state["v"]

    if abs(anglepositionsetpoint - theta) < THETA_THRESHOLD and abs(x) < POSITION_LIMIT:
        print("here", anglepositionsetpoint, "theta", theta)
        state_vec = [theta, w_filtered, x, v_filtered]
        u = (LQR_K[0] * (anglepositionsetpoint - state_vec[0])
             - LQR_K[1] * state_vec[1]
             + LQR_K[2] * (cartpositionsetpoint - state_vec[2])
             - LQR_K[3] * state_vec[3])
        u = saturate(avoidStall(u), 255.0)
        volt = (abs(u) / 255.0) * VBUS * signof(u)
        return volt
    elif abs(x) < POSITION_LIMIT:
            #print("balanceo", anglepositionsetpoint, "theta", theta)
            thetaaux = theta
            currentEnergy = 0.5 * INERTIA_EQ * w_filtered * w_filtered + MGL * (1 - math.cos(thetaaux))
            # condición de ángulo cerca de arriba (theta > 2pi-0.25 o theta < 0.25)
            if ((theta > (2.0 * math.pi - 0.25)) or (theta < 0.25)) and (currentEnergy < 0.85):
                carAcceleration = 300.0 * kfeedbacklinearization * (abs(currentEnergy - desiredEnergy)) * w_filtered
                u =  saturate(avoidStall(carAcceleration), 255.0)
            else:
                carAcceleration = 0.0
                u = 0.0

            volt = (abs(u) / 255.0) * VBUS * signof(u)
            return volt
    else:
        print("Fuera de posición, no se envía control.")
        return 0.0

# ---------------------------
# Main loop de ejemplo
def main_loop(run_time=60.0, send_pwm=False):
    ser = init_serial()
    print("Puerto serial abierto:", SERIAL_PORT)
    # opcional: centrar y reset
    center(ser)
    print("Centrado enviado. Intentando reset hasta abajo...")
    ok = reset_to_down(ser)
    print("Reset terminado (abajo)?", ok)

    last_time = None
    t0 = time.time()
    try:
        while True:
            now = time.time()
            if last_time is None:
                dt = 0.01
            else:
                dt = now - last_time
                if dt <= 0:
                    dt = 1e-3
            last_time = now

            state = get_state(ser, tries=8, send_request=True)
            voltage = compute_control(state)

            if send_pwm:
                # convertir volt->pwm si queremos enviar pwm
                pwm = (abs(voltage) / VBUS) * 255.0 * signof(voltage)
                send_pwm_command(ser, pwm)
            else:
                send_voltage_command(ser, voltage)

            # impresión informativa
            print("t={:.3f} theta={:.2f}° w={:.2f}°/s x={:.4f} v={:.4f} -> V={:.2f}V".format(
                now - t0,
                (theta_rad := state["angle_rad"]) * (180.0 / math.pi),
                (w_rad_s := state["w_rad_s"]) * (180.0 / math.pi),
                state["x"],
                state["v"],
                voltage
            ))

            # pequeña espera para no saturar el puerto; cadencia aproximada
            # si tu control requiere timing estricto, ajusta el sleep y dt
            time.sleep(0.02)

            if run_time is not None and (now - t0) > run_time:
                break

    except KeyboardInterrupt:
        print("Interrumpido por usuario.")
    finally:
        try:
            ser.close()
        except:
            pass
    
        try:
            ser = init_serial()
            center(ser)
            ser.close()
            print("Centrado al finalizar.")
        except Exception as e:
            print("No se pudo centrar al finalizar:", e)

# ---------------------------
if __name__ == "__main__":
    # Ejemplo: ejecutar por 5s y enviar voltaje (no PWM)
    main_loop(run_time=10.0, send_pwm=False)
