# Proyecto final robótica

## Integrantes
- Diego Álvarez
- Javier Bórquez
- Ariel Carrasco Suárez
- Benjamín Peredo
- Diego Valenzuela

## Línea seleccionada
La línea seleccionada para este proyecto corresponde a planificación de rutas, utilizando el algoritmo A* sobre una grilla de ocupación.

## 1. Objetivo del proyecto
El objetivo del proyecto fue diseñar, implementar y evaluar un sistema de navegación autónoma para un robot móvil diferencial en Webots. Para ello, se integraron conceptos de control cinemático diferencial, lectura de sensores, odometría mediante encoders, planificación global de rutas y evitación reactiva de obstáculos.

El sistema desarrollado permite que el robot calcule una ruta desde una posición inicial hasta una meta usando el algoritmo A*, además permite que convierta dicha ruta en puntos intermedios y luego siga esos puntos mediante un control de velocidad de las ruedas.

## 2. Descripción del Robot
El robot utilizado fue un E-puck, disponible dentro de Webots. Este robot corresponde a un robot móvil diferencial, ya que se desplaza mediante dos ruedas motrices independientes: una rueda izquierda y una rueda derecha.
Estas son las características del robot:
- Robot: E-puck.
- Tipo de movimiento: diferencial.
- Motor izquierdo: left wheel motor.
- Motor derecho: right wheel motor.
- Encoder izquierdo: left wheel sensor.
- Encoder derecho: right wheel sensor.
- Sensores de proximidad utilizados: ps0 y ps7.
- Controlador: ControladorProyectoFinal.py.

## 3. Descripción del entorno de Webots
El mundo de Webots fue diseñado con obstáculos, representados mediante barriles.
El entorno se trabajó como un espacio conocido, por lo que fue representado en el controlador mediante una grilla de ocupación. En esta grilla:
- 0 representa una celda libre.
- 1 representa una celda ocupada por un obstáculo.

La grilla utilizada fue la siguiente:
MAPA_GRID = [ 
    [0, 0, 1, 0, 0], 
    [0, 0, 0, 0, 0], 
    [0, 0, 1, 0, 1], 
    [1, 1, 0, 0, 1], 
    [0, 0, 0, 0, 0] 
]

Cada celda fue asociada a una distancia de 0.2 m, permitiendo convertir las posiciones discretas de la ruta planificada en coordenadas reales dentro del entorno.

## 4. Sensores y actuadores que se usaron
### 4.1 Motores diferenciales
El robot utiliza dos motores principales:
- Motor de la rueda izquierda.
- Motor de la rueda derecha.

El controlador asigna velocidades distintas a cada rueda para lograr diferentes tipos de movimiento:
- Si ambas ruedas giran a la misma velocidad, el robot avanza recto.
- Si una rueda gira más rápido que la otra, el robot describe una curva.
- Si una rueda gira hacia adelante y la otra hacia atrás, el robot gira sobre su propio eje.
  
Esto permite implementar el modelo de movimiento diferencial trabajado en los laboratorios.

### 4.2 Encoders de las ruedas
Se utilizaron los sensores de posición de las ruedas para estimar el movimiento del robot mediante odometría.
Los encoders permiten calcular cuánto ha girado cada rueda. A partir de esa información, el controlador estima:
- La distancia recorrida por la rueda izquierda.
- La distancia recorrida por la rueda derecha.
- El desplazamiento promedio del robot.
- El cambio de orientación.
- La nueva posición aproximada del robot.

Las ecuaciones utilizadas están en ControladorProyectoFinal.py y fueron estas:
- Δsr = r · Δθr
- Δsl = r · Δθl
  
- Δs = (Δsr + Δsl) / 2
- Δφ = (Δsr - Δsl) / L

- xk = xk-1 + Δs · cos(φk-1 + Δφ/2)
- yk = yk-1 + Δs · sin(φk-1 + Δφ/2)
- φk = φk-1 + Δφ

Donde:
- r es el radio de la rueda.
- L es la distancia entre ruedas.
- Δθr y Δθl son los cambios angulares de las ruedas.
- x, y y φ representan la pose estimada del robot.

### 4.3 Sensores de proximidad
Se utilizaron los sensores ps0 y ps7 del E-puck para detectar obstáculos cercanos.
Estos sensores permiten implementar una estrategia de navegación local. Si alguno de los sensores supera un umbral definido, el robot interpreta que existe un obstáculo cercano y ejecuta una maniobra reactiva de evasión.
La reacción implementada consiste en hacer girar al robot sobre su eje, asignando velocidad negativa a una rueda y positiva a la otra. De esta manera, el robot intenta evitar una posible colisión.

## 5. Funcionamiento del controlador
El controlador fue implementado en Python y se divide en las siguientes etapas:

### 5.1 Inicialización
Primero se inicializa el robot, el tiempo de simulación, los motores, los encoders y los sensores de proximidad.
También se definen los parámetros físicos principales:
- WHEEL_RADIUS = 0.0205
- TRACK_WIDTH = 0.0528
- MAX_SPEED = 6.28
Estos valores se utilizan para convertir las velocidades deseadas del robot en velocidades angulares de las ruedas.

### 5.2 Definición del mapa
El entorno se representa mediante una matriz 2D. Las celdas libres se indican con 0 y los obstáculos con 1.
Esta representación permite aplicar un algoritmo de planificación de rutas antes de iniciar el movimiento físico del robot.

### 5.3 Planificación de ruta con A*
El algoritmo A* calcula una ruta desde la celda inicial hasta la celda meta.
En este proyecto se definió:
- CELDA_INICIO = (0, 0)
- CELDA_META = (4, 4)

El algoritmo explora la grilla considerando movimientos hacia arriba, abajo, izquierda y derecha. Para guiar la búsqueda se utiliza una heurística basada en distancia euclidiana.
La ruta calculada se transforma después en coordenadas reales (x, y) para que el robot pueda seguirla dentro de Webots.

### 5.4 Seguimiento de los puntos intermedios
Una vez obtenida la ruta, el robot no intenta llegar directamente a la meta final, sino que sigue una secuencia de puntos intermedios o waypoints.
Para cada waypoint, el controlador calcula:
- Diferencia en X.
- Diferencia en Y.
- Distancia al objetivo.
- Ángulo deseado hacia el objetivo.
- Error angular entre la orientación actual y la orientación deseada.

Si el robot está suficientemente cerca del waypoint, se considera alcanzado y se avanza al siguiente.
La tolerancia usada fue:
- DISTANCE_TOLERANCE = 0.08

### 5.5 Control de movimiento diferencial
El controlador usa una estrategia proporcional simple:
- omega = K_w * error_angulo
- velocidad = K_v * distancia_objetivo

Luego se transforman la velocidad lineal y angular en velocidades de rueda:
- v_l = (2 * velocidad - omega * TRACK_WIDTH) / (2 * WHEEL_RADIUS)
- v_r = (2 * velocidad + omega * TRACK_WIDTH) / (2 * WHEEL_RADIUS)

Finalmente, las velocidades se limitan al valor máximo permitido por el robot.

### 5.6 Evitación reactiva de obstáculos
Además de seguir la ruta planificada, el robot revisa continuamente los sensores de proximidad.
Si se detecta un obstáculo cercano, el robot interrumpe temporalmente el seguimiento de la ruta y ejecuta un giro reactivo:
- v_l = -0.5 * MAX_SPEED
- v_r = 0.5 * MAX_SPEED

Esto permite reducir el riesgo de colisión ante obstáculos cercanos o no considerados perfectamente en la planificación inicial.

## 6. Estrategia de navegación
La estrategia desarrollada combina navegación global y navegación local.

### 6.1 Navegación global
La navegación global se realiza mediante planificación de rutas con A*. El mapa del entorno se representa como una grilla de ocupación conocida previamente. El algoritmo genera una ruta desde la posición inicial hasta la meta, evitando las celdas marcadas como obstáculos.
Esta ruta representa el camino ideal que debería seguir el robot.

### 6.2 Navegación local
La navegación local se realiza mediante sensores de proximidad. Si el robot detecta un obstáculo cercano, ejecuta una acción reactiva para evitarlo.
Por lo tanto, el sistema no depende únicamente de la planificación inicial, sino que también puede reaccionar ante obstáculos detectados durante la ejecución.

### 6.3 Seguimiento de trayectoria
La ruta generada por A* se convierte en una lista de waypoints. El robot sigue estos puntos usando su posición estimada por odometría y un control proporcional sobre la distancia y el error angular.

## 7. Relación con los Laboratorios 1 y 2

### Laboratorio 1
Del Laboratorio 1 se reutilizó el modelo de control cinemático diferencial. En particular, se aplicó el uso de velocidades independientes en las ruedas izquierda y derecha para lograr:
- Avance recto.
- Giros.
- Trayectorias curvas.
También se aplicaron las ecuaciones que relacionan velocidad lineal, velocidad angular y velocidades de las ruedas.

### Laboratorio 2
Del Laboratorio 2 se integró el uso de sensores y encoders. Los encoders permitieron estimar la posición y orientación del robot mediante odometría, mientras que los sensores de proximidad permitieron detectar obstáculos cercanos y activar una respuesta reactiva.

Por lo tanto el proyecto final integra control de movimiento, percepción del entorno, estimación de movimiento y toma de decisiones de navegación.

## 8. Diagrama de flujo de la solución
Inicio -> Inicializar robot, motores, encoders y sensores -> Definir grilla de ocupación -> Ejecutar algoritmo A* -> ¿Existe ruta? | -> Si es no -> Detener simulación | -> Sí -> Convertir ruta a coordenadas reales -> Iniciar ciclo de navegación -> Leer encoders -> Actualizar posición por odometría -> Leer sensores de proximidad -> ¿Hay obstáculo cercano? | -> Sí -> Girar de forma reactiva | -> Si es no -> Calcular distancia y error angular al waypoint -> ¿Waypoint alcanzado? | -> Sí -> Avanzar al siguiente waypoint | -> Si es no -> Calcular velocidades de rueda -> Mover robot -> ¿Meta alcanzada? | -> Sí -> Detener robot | -> Si es no -> Repetir ciclo

## 9. Escenarios de prueba
### 9.1 Escenario simple
El primer escenario corresponde a un entorno con obstáculos distribuidos en la arena, representados mediante barriles. El robot debía desplazarse desde la posición inicial hasta una meta ubicada en un extremo del mapa, siguiendo una ruta calculada por A*.

La ruta planificada fue:
[
    [0.0, 0.8],
    [0.2, 0.8],
    [0.2, 0.6],
    [0.4, 0.6],
    [0.6, 0.6],
    [0.6, 0.4],
    [0.6, 0.2],
    [0.6, 0.0],
    [0.8, 0.0]
]

### 9.2 Escenario complejo
El segundo escenario corresponde también a un entorno con obstáculos distribuidos en la arena, representados mediante barriles. La idea fue poner más barriles para hacer el trayecto más complicado. Al igual que el otro escenario, el robot debía desplazarse desde la posición inicial hasta una meta ubicada en un extremo del mapa, siguiendo una ruta calculada por A*.

La ruta planificada fue:
[
    [0.0, 0.8],
    [0.2, 0.8],
    [0.4, 0.8],
    [0.6, 0.8],
    [0.6, 0.6],
    [0.6, 0.4],
    [0.4, 0.4],
    [0.2, 0.4],
    [0.2, 0.2],
    [0.2, 0.0],
    [0.4, 0.0],
    [0.6, 0.0],
    [0.8, 0.0]
]

## 10. Resultados obtenidos
### 10.1 Resultados cuantitativos del escenario simple
A partir de la ejecución del robot en Webots, se obtuvieron las siguientes métricas:

| Métrica                                                 | Resultado          |
|---------------------------------------------------------|--------------------|
| Tiempo total de navegación                              | 13,47 s            |
| Longitud de la ruta planificada                         | 1,60 m             |
| Longitud aproximada de la trayectoria ejecutada         | 1,29 m             |
| Diferencia entre ruta planificada y trayectoria real    | 0,31 m             |
| Error final de posición respecto a la meta              | 0,077 m            |
| Error angular promedio absoluto                         | 0,422 rad          |
| Error angular máximo absoluto                           | 1,368 rad          |
| Puntos intermedios alcanzados                           | 9/9                |
| Estado final                                            | Meta alcanzada     |

El robot logró alcanzar todos los puntos intermedios definidos por la ruta planificada. La posición final estimada fue aproximadamente:
- x = 0.723 m
- y = -0.009 m

La meta estaba ubicada en:
- x = 0.8 m
- y = 0.0 m

El error final fue aproximadamente 0.077 m, valor menor que la tolerancia configurada de 0.08 m. Por esta razón, la ejecución se considera exitosa.

### 10.2 Comparación entre ruta planificada y trayectoria real (escenario simple)
En el gráfico de trayectoria se observa que la ruta planificada por A* tiene forma segmentada, con movimientos rectos y cambios de dirección de 90°. En cambio, la trayectoria real estimada por odometría es más curva y suave.
Esto ocurre porque A* trabaja en una grilla discreta, mientras que el robot diferencial se mueve en un espacio continuo. El robot no puede seguir las esquinas como en la grilla, porque debe girar y corregir su orientación progresivamente.
La trayectoria real fue más corta que la planificada. Esto se debe principalmente a que el robot recortó algunas esquinas durante el seguimiento de los waypoints. Aunque la trayectoria no coincidió exactamente con la ruta ideal, el robot logró acercarse correctamente a la meta.

## 10.3 Gráficos y evidencias
El gráfico de trayectoria del escenario simple muestra:
- Ruta planificada por A* en rojo.
- Trayectoria real estimada por odometría en azul.
- Punto de inicio en verde.
- Meta en negro.

El gráfico de error muestra:
- Evolución del reloj angular en naranjo
  
<img width="1488" height="590" alt="resultados" src="https://github.com/user-attachments/assets/fbb8fc64-c021-4484-aea8-4aa51bcd2faf" />

### 10.4 Análisis del error angular (escenario simple)
El gráfico de error angular muestra que el error se mantiene bajo o relativamente estable durante los tramos rectos, pero aumenta en los cambios de dirección.
Los mayores picos de error angular aparecen cuando el robot cambia de un waypoint a otro y debe modificar su orientación. El error angular máximo fue de aproximadamente 1.368 rad, lo que indica que en algunos momentos el robot tuvo que realizar correcciones importantes.
Aun así, el controlador logró reducir el error después de cada cambio de dirección y permitió que el robot continuara avanzando hacia la meta.

### 10.5 Resultados cuantitativos del escenario complicado
A partir de la ejecución del robot en Webots en el escenario complicado, se obtuvieron las siguientes métricas:

| Métrica                                                 | Resultado          |
|---------------------------------------------------------|--------------------|
| Tiempo total de navegación                              | 28,67 s            |
| Longitud de la ruta planificada                         | 2,40 m             |
| Longitud aproximada de la trayectoria ejecutada         | 2,27 m             |
| Diferencia entre ruta planificada y trayectoria real    | 0,13 m             |
| Error final de posición respecto a la meta              | 0,079 m            |
| Error angular promedio absoluto                         | 0,433 rad          |
| Error angular máximo absoluto                           | 1,369 rad          |
| Puntos intermedios alcanzados                           | 13/13              |
| Estado final                                            | Meta alcanzada     |

La ruta planificada por A* estuvo compuesta por 13 puntos intermedios. El robot inició en la posición:
- x = 0.0 m
- y = 0.8 m

La meta estaba ubicada en:
- x = 0.8 m
- y = 0.0 m

La posición final estimada por odometría fue aproximadamente:
- x = 0.730 m
- y = 0.036 m

El error final fue aproximadamente 0.079 m, valor menor que la tolerancia configurada de 0.08 m. Por esta razón, la ejecución se considera exitosa, ya que el robot logró llegar a la zona de la meta dentro del margen permitido.

### 10.6 Comparación entre ruta planificada y trayectoria real (escenario complicado)
En el gráfico de trayectoria se observa que la ruta planificada por A* tiene una forma más extensa y con más cambios de dirección que en el escenario simple. Esto se debe a que el escenario complicado posee una distribución de obstáculos que obliga al robot a seguir un camino más largo antes de llegar a la meta.
La ruta planificada aparece en rojo, mientras que la trayectoria real estimada por odometría aparece en azul. En general, el robot siguió correctamente la forma global de la ruta, avanzando primero por la parte superior del mapa, luego descendiendo por la zona derecha, desplazándose hacia la izquierda en una zona intermedia y finalmente bajando para acercarse a la meta.
La trayectoria real no coincide exactamente con la ruta planificada. Esto ocurre porque A* entrega una ruta discreta sobre una grilla, formada por tramos rectos y cambios de dirección marcados. En cambio, el robot diferencial se mueve en un espacio continuo, por lo que realiza curvas y ajustes progresivos durante el seguimiento de los waypoints.
A pesar de estas diferencias, la trayectoria ejecutada se mantuvo cercana a la ruta planificada. La diferencia entre la longitud de la ruta ideal y la trayectoria real fue de aproximadamente 0,13 m, lo que indica que el robot siguió la ruta de manera bastante consistente en este escenario.

## 10.7 Gráficos y evidencias
El gráfico de trayectoria del escenario complicado muestra:
- Ruta planificada por A* en rojo.
- Trayectoria real estimada por odometría en azul.
- Punto de inicio en verde.
- Meta en negro.

El gráfico de error del escenario complicado muestra:
- Evolución del error angular en naranjo.
- Mayores variaciones del error durante los cambios de dirección.
- Correcciones de orientación realizadas por el robot mientras sigue los waypoints.
 
<img width="1489" height="590" alt="resultados dificiles" src="https://github.com/user-attachments/assets/97f743c4-dbc4-4a5e-9b62-3807250de8f6" />


### 10.8 Análisis del error angular (escenario complicado)
El gráfico de error angular muestra que el robot tuvo que realizar varias correcciones durante el recorrido. Esto era esperable, ya que el escenario complicado incluye más puntos intermedios y más cambios de dirección que el escenario simple.
Durante los primeros segundos, el error angular se mantuvo cercano a cero mientras el robot avanzaba en un tramo recto. Luego aparecen variaciones más notorias cuando el robot comienza a cambiar de dirección para seguir los nuevos waypoints.
El error angular máximo fue de aproximadamente 1.369 rad, valor similar al obtenido en el escenario simple. Esto indica que, en algunos momentos, el robot tuvo que corregir fuertemente su orientación. Sin embargo, el controlador logró estabilizar el movimiento y continuar avanzando hacia la meta.
El error angular promedio absoluto fue de aproximadamente 0.433 rad. Esto muestra que el robot mantuvo una desviación angular moderada durante el recorrido, pero sin perder la ruta ni dejar de alcanzar los puntos intermedios.

## 12. Limitaciones
### 12.1 Error acumulado de odometría
La posición del robot se estimó mediante encoders. Aunque este método permite calcular el desplazamiento aproximado, puede acumular error con el tiempo, especialmente durante giros o trayectorias curvas.

### 12.2 Diferencia entre grilla y mundo real
El algoritmo A* trabaja sobre una grilla discreta, pero el robot se mueve en un entorno continuo. Esto produce diferencias entre la ruta ideal y la trayectoria realmente ejecutada.

### 12.3 Evitación reactiva simple
La evasión de obstáculos se implementó con una regla simple basada en sensores de proximidad. Si se detecta un obstáculo, el robot gira sobre su eje. Esta solución permite evitar ciertos riesgos de colisión, pero no recalcula automáticamente una nueva ruta global.

### 12.4 Dependencia del mapa conocido
El sistema depende de una grilla de ocupación definida previamente. Si el escenario cambia, es necesario actualizar manualmente el mapa en el código.

## 13. Posibles mejoras
Como mejoras futuras se proponen:
- Implementar replanter la ruta cuando aparezca un obstáculo inesperado.
- Usar más sensores de proximidad del E-puck para detectar obstáculos en más direcciones.
- Incorporar filtrado simple o filtro de Kalman para estabilizar la odometría.
- Ajustar las ganancias del controlador para reducir el error angular.
- Reducir el recorte de esquinas mediante un seguimiento más preciso de waypoints.
- Ejecutar más pruebas por escenario para calcular porcentaje de éxito.

## 14. Instrucciones para ejecutar la simulación
1. Abrir Webots
2. Cargar el mundo del proyecto: proyectofinal.wbt
3. Verificar que el robot E-puck tenga asignado el controlador: ControladorProyectoFinal
4. Ejecutar la simulación.
5. Observar la consola para verificar:
    - Ruta calculada por A*.
    - Puntos intermedios alcanzados.
    - Detección de obstáculos.
    - Mensaje de meta alcanzada.
  
## 15. Conclusiones
El proyecto logró implementar un sistema de navegación autónoma en Webots utilizando un robot diferencial E-puck. La solución integró planificación global mediante A*, seguimiento de puntos intermedios, estimación de movimiento con encoders y evasión reactiva mediante sensores de proximidad.
En el escenario evaluado, el robot logró alcanzar la meta dentro de la tolerancia definida. La ruta planificada tuvo una longitud de 1.60 m, mientras que la trayectoria real estimada fue de aproximadamente 1.29 m. El error final de posición fue de 0.077 m, lo que indica que el robot llegó correctamente a la zona objetivo.
Los resultados muestran que la combinación de A*, odometría y control diferencial permite resolver el problema de navegación autónoma en un entorno conocido. Sin embargo, también se observaron limitaciones relacionadas con el error acumulado de odometría y la falta de replanteamiento automático ante obstáculos no considerados en el mapa inicial.
Como trabajo futuro, se propone mejorar la precisión del seguimiento de trayectoria, incorporar filtrado de mediciones, probar un escenario más complejo, etc.
