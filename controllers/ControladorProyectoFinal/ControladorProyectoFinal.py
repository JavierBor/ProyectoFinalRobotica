from controller import Robot
import math
import heapq

# --- FUNCIONES DEL ALGORITMO A* ---
def heuristica(a, b):
    # Distancia euclidiana entre dos puntos
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def a_star(grid, start, goal):
    # Movimientos permitidos: arriba, abajo, izquierda, derecha
    vecinos = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristica(start, goal)}
    oheap = []
    
    heapq.heappush(oheap, (fscore[start], start))
    
    while oheap:
        current = heapq.heappop(oheap)[1]
        
        if current == goal:
            ruta = []
            while current in came_from:
                ruta.append(current)
                current = came_from[current]
            ruta.append(start)
            return ruta[::-1] # Retorna la ruta desde el inicio hasta la meta
            
        close_set.add(current)
        
        for i, j in vecinos:
            vecino = (current[0] + i, current[1] + j)
            
            # Verificar límites de la matriz
            if 0 <= vecino[0] < len(grid) and 0 <= vecino[1] < len(grid[0]):
                if grid[vecino[0]][vecino[1]] == 1: # Es un obstáculo
                    continue
            else:
                continue # Fuera de la matriz
                
            tentative_g_score = gscore[current] + heuristica(current, vecino)
            
            if vecino in close_set and tentative_g_score >= gscore.get(vecino, 0):
                continue
                
            if tentative_g_score < gscore.get(vecino, 0) or vecino not in [i[1] for i in oheap]:
                came_from[vecino] = current
                gscore[vecino] = tentative_g_score
                fscore[vecino] = tentative_g_score + heuristica(vecino, goal)
                heapq.heappush(oheap, (fscore[vecino], vecino))
                
    return [] # Retorna lista vacía si no hay ruta posible

# --- INICIALIZACIÓN DEL ROBOT ---
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

WHEEL_RADIUS = 0.0205
TRACK_WIDTH = 0.0528
MAX_SPEED = 6.28

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

prox_sensors = [robot.getDevice(name) for name in ['ps0', 'ps7']]
for sensor in prox_sensors:
    sensor.enable(TIME_STEP)

# --- DEFINICIÓN DEL ENTORNO (GRILLA DE OCUPACIÓN) ---
# 0 = Libre, 1 = Obstáculo
MAPA_GRID = [
    [ 0,  0,  1,  0,  0 ],
    [ 0,  0,  0,  0,  0 ],
    [ 0,  0,  1,  0,  1 ],
    [ 1,  1,  0,  0,  1 ],
    [ 0,  0,  0,  0,  0 ]
]

# --- PLANIFICACIÓN DE LA RUTA ---
CELDA_INICIO = (0, 0) # (fila, columna)
CELDA_META = (4, 4)   # (fila, columna)
TAMANO_CELDA_MTS = 0.2 # Cada celda representa 0.1 metros (10 cm) en Webots

print("Calculando ruta con A*...")
ruta_celdas = a_star(MAPA_GRID, CELDA_INICIO, CELDA_META)

if not ruta_celdas:
    print("ERROR: No se encontró una ruta viable a la meta.")
    ruta_planificada = []
else:
    print(f"Ruta encontrada (en celdas): {ruta_celdas}")
    
    ruta_planificada = []
    TOTAL_FILAS = len(MAPA_GRID)
    
    # Convertir las celdas a coordenadas (x, y) invirtiendo el eje Y
    for (fila, col) in ruta_celdas:
        x = col * TAMANO_CELDA_MTS
        y = (TOTAL_FILAS - 1 - fila) * TAMANO_CELDA_MTS  # <--- INVERSIÓN AQUÍ
        ruta_planificada.append((x, y))
    print(f"Ruta en metros (x, y): {ruta_planificada}")

# --- VARIABLES DE ESTADO Y ODOMETRÍA ---
x_k = CELDA_INICIO[1] * TAMANO_CELDA_MTS  
y_k = (TOTAL_FILAS - 1 - CELDA_INICIO[0]) * TAMANO_CELDA_MTS  
phi_k = 0.0
last_enc_l = 0.0
last_enc_r = 0.0

current_waypoint_index = 0
DISTANCE_TOLERANCE = 0.08  # Se subió a 0.08 para dar un margen de arribo realista

# --- BUCLE PRINCIPAL ---
while robot.step(TIME_STEP) != -1:
    if not ruta_planificada:
        break 
        
    # 1. PERCEPCIÓN Y ODOMETRÍA (Basado en ecuaciones de guía)
    enc_l = left_encoder.getValue()
    enc_r = right_encoder.getValue()
    
    diff_l = enc_l - last_enc_l
    diff_r = enc_r - last_enc_r
    
    ds_l = WHEEL_RADIUS * diff_l
    ds_r = WHEEL_RADIUS * diff_r
    
    ds = (ds_r + ds_l) / 2.0
    dphi = (ds_r - ds_l) / TRACK_WIDTH
    
    x_k += ds * math.cos(phi_k + dphi / 2.0)
    y_k += ds * math.sin(phi_k + dphi / 2.0)
    phi_k += dphi
    
    last_enc_l = enc_l
    last_enc_r = enc_r
    
    # 2. DETECCIÓN REACTIVA DE OBSTÁCULOS
    obstaculo_detectado = False
    for sensor in prox_sensors:
        # Ajustado a 400 para evitar falsos positivos con paredes laterales de la grilla
        if sensor.getValue() > 400.0:
            obstaculo_detectado = True
            break
            
    # 3. CONTROL DE NAVEGACIÓN (Prioridades Corregidas según Rúbrica)
    v_l = 0.0
    v_r = 0.0
    
    # CORRECCIÓN 1: Validar si la meta fue alcanzada de forma absoluta primero
    if current_waypoint_index >= len(ruta_planificada):
        print("¡Meta final alcanzada con éxito!")
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        break # Rompe el ciclo y detiene por completo al robot
        
    # CORRECCIÓN 2: Si no ha llegado, actuar ante obstáculos imprevistos en ruta
    elif obstaculo_detectado:
        v_l = -0.5 * MAX_SPEED
        v_r = 0.5 * MAX_SPEED
        print("¡Obstáculo imprevisto! Evitando de forma reactiva.")
        
    # CORRECCIÓN 3: Navegación estándar hacia puntos intermedios
    else:
        target_x, target_y = ruta_planificada[current_waypoint_index]
        
        dx = target_x - x_k
        dy = target_y - y_k
        distancia_objetivo = math.sqrt(dx**2 + dy**2)
        angulo_objetivo = math.atan2(dy, dx)
        
        error_angulo = angulo_objetivo - phi_k
        
        while error_angulo > math.pi: error_angulo -= 2 * math.pi
        while error_angulo < -math.pi: error_angulo += 2 * math.pi
        
        if distancia_objetivo < DISTANCE_TOLERANCE:
            print(f"Punto intermedio alcanzado: {ruta_planificada[current_waypoint_index]}")
            current_waypoint_index += 1
        else:
            K_w = 3.0
            K_v = 2.0
            
            omega = K_w * error_angulo
            velocidad = K_v * distancia_objetivo if abs(error_angulo) < 0.5 else 0.0
            
            # Fórmulas cinemáticas estandarizadas de asignación diferencial
            v_l = (2 * velocidad - omega * TRACK_WIDTH) / (2 * WHEEL_RADIUS)
            v_r = (2 * velocidad + omega * TRACK_WIDTH) / (2 * WHEEL_RADIUS)
            
            v_r = max(min(v_r, MAX_SPEED), -MAX_SPEED)
            v_l = max(min(v_l, MAX_SPEED), -MAX_SPEED)
            
    left_motor.setVelocity(v_l)
    right_motor.setVelocity(v_r)