from controller import Robot
import csv
import math

# Configuración básica
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 1. Inicializar Motores
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# 2. Inicializar Sensores de Distancia
ps = []
sensors_names = ['ps0', 'ps7', 'ps5', 'ps2']
for name in sensors_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ps.append(sensor)

# Sensores adicionales para mapeo completo del entorno
ps_extra = []
for name in ['ps1', 'ps3', 'ps4', 'ps6']:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ps_extra.append(sensor)

# 3. Inicializar Encoders
left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# --- Variables de Odometría y Kalman ---
RADIO_RUEDA = 0.0205
DISTANCIA_EJES = 0.052          # L [m]
robot.step(timestep)
pos_izq_anterior = left_encoder.getValue()
pos_der_anterior = right_encoder.getValue()

d_k = 0.0
P_k = 100.0
Q = 0.2
R = 30.0

# --- Constantes de Navegación ---
VELOCIDAD_BASE = 3.0
TIEMPO_MAXIMO = 10.0

# Odometría 2D
x_robot     = 0.0
y_robot     = 0.0
theta_robot = 0.0

# Métricas
colisiones_inminentes = 0
datos = []
tiempo = 0.0

print("Iniciando exploración... Los datos se guardarán al finalizar.")

# Bucle principal
while robot.step(timestep) != -1:
    tiempo += timestep / 1000.0

    # A. Leer Sensores
    lectura_ps0 = ps[0].getValue()
    lectura_ps7 = ps[1].getValue()
    lectura_izq = ps[2].getValue()  # ps5
    lectura_der = ps[3].getValue()  # ps2

    lectura_ps1 = ps_extra[0].getValue()
    lectura_ps3 = ps_extra[1].getValue()
    lectura_ps4 = ps_extra[2].getValue()
    lectura_ps6 = ps_extra[3].getValue()

    # Medición cruda (promedio frontal para el filtro)
    z_k = (lectura_ps0 + lectura_ps7) / 2.0

    # B. Leer Encoders y Calcular Avance Lineal
    pos_izq_actual = left_encoder.getValue()
    pos_der_actual = right_encoder.getValue()

    delta_theta_izq = pos_izq_actual - pos_izq_anterior
    delta_theta_der = pos_der_actual - pos_der_anterior

    avance_lineal = (RADIO_RUEDA * delta_theta_izq + RADIO_RUEDA * delta_theta_der) / 2.0

    pos_izq_anterior = pos_izq_actual
    pos_der_anterior = pos_der_actual

    # C. FILTRO DE KALMAN (Se mantiene para evaluar métricas de estabilidad)
    d_k_prediccion = d_k - avance_lineal
    P_k_prediccion = P_k + Q
    K_k = P_k_prediccion / (P_k_prediccion + R)
    d_k = d_k_prediccion + K_k * (z_k - d_k_prediccion)
    P_k = (1.0 - K_k) * P_k_prediccion

    # D. ODOMETRÍA 2D
    delta_sl = RADIO_RUEDA * delta_theta_izq
    delta_sr = RADIO_RUEDA * delta_theta_der
    delta_s  = (delta_sr + delta_sl) / 2.0
    delta_phi = (delta_sr - delta_sl) / DISTANCIA_EJES

    x_robot     += delta_s * math.cos(theta_robot + delta_phi / 2.0)
    y_robot     += delta_s * math.sin(theta_robot + delta_phi / 2.0)
    theta_robot += delta_phi

    # Normalizar theta a (−π, π)
    while theta_robot >  math.pi: theta_robot -= 2 * math.pi
    while theta_robot < -math.pi: theta_robot += 2 * math.pi

    # E. LÓGICA DE NAVEGACIÓN (Avanzar recto y evasión reactiva)
    
    # Agrupamos las lecturas para saber de qué lado está la amenaza
    frente_izq = max(lectura_ps7, lectura_ps6, lectura_izq)
    frente_der = max(lectura_ps0, lectura_ps1, lectura_der)

    UMBRAL_DETECCION = 150.0

    if max(lectura_ps0, lectura_ps7) > 300:
        colisiones_inminentes += 1

    if frente_izq > UMBRAL_DETECCION or frente_der > UMBRAL_DETECCION:
        # Si detecta algo, gira hacia el lado que esté más despejado
        if frente_izq > frente_der:
            # Obstáculo a la izquierda -> girar a la derecha
            left_motor.setVelocity(VELOCIDAD_BASE * 0.8)
            right_motor.setVelocity(-VELOCIDAD_BASE * 0.8)
            estado = "EVADIENDO_DER"
        else:
            # Obstáculo a la derecha -> girar a la izquierda
            left_motor.setVelocity(-VELOCIDAD_BASE * 0.8)
            right_motor.setVelocity(VELOCIDAD_BASE * 0.8)
            estado = "EVADIENDO_IZQ"
    else:
        # Camino libre -> Avanzar en línea recta a máxima velocidad
        left_motor.setVelocity(VELOCIDAD_BASE)
        right_motor.setVelocity(VELOCIDAD_BASE)
        estado = "AVANZANDO_RECTO"

    # F. GUARDAR DATOS EN MEMORIA
    datos.append([
        round(tiempo, 3),
        round(x_robot, 4),
        round(y_robot, 4),
        round(theta_robot, 4),
        lectura_ps0, lectura_ps7, lectura_izq, lectura_der,
        lectura_ps1, lectura_ps3, lectura_ps4, lectura_ps6,
        round(z_k, 2), round(d_k, 2),
        estado
    ])

    # G. CONDICIÓN DE SALIDA
    if tiempo >= TIEMPO_MAXIMO:
        print(f"\nTiempo máximo de {TIEMPO_MAXIMO}s alcanzado.")
        print("Deteniendo motores y guardando datos...")
        
        # Detener el robot
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        
        # Romper el ciclo while para pasar a la escritura del CSV
        break

# ======================================================
# EXPORTAR CSV AL FINALIZAR
# ======================================================
with open('registro_sensores.csv', 'w', newline='') as archivo:
    writer = csv.writer(archivo)
    writer.writerow([
        'time', 'x', 'y', 'theta_rad',
        'ps0', 'ps7', 'ps5', 'ps2', 'ps1', 'ps3', 'ps4', 'ps6',
        'z_k_raw', 'd_k_kalman', 'estado'
    ])
    writer.writerows(datos)

print(f"\nArchivo 'registro_sensores.csv' guardado correctamente.")
print(f"Colisiones inminentes registradas: {colisiones_inminentes}")