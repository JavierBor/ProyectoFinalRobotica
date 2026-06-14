# Proyecto final robótica

## Integrantes
- Diego Álvarez
- Javier Bórquez
- Ariel Carrasco Suárez
- Benjamín Peredo
- Diego Valenzuela

**Linea seleccionada:** Linea B (SLAM o mapeo autónomo simplificado)

## 1. Objetivo del proyecto
El objetivo principal es desarrollar un sistema de navegación autónoma para un robot diferencial en el simulador Webots, capaz de recorrer entornos con obstáculos y generar un mapa del espacio explorado (Línea B: SLAM simplificado). Para lograrlo, el sistema integra el modelo de control cinemático para la evasión de obstáculos, el uso de encoders para la estimación de la trayectoria (odometría 2D), y el procesamiento de sensores de distancia para actualizar una grilla de ocupación. Finalmente, se busca evaluar experimentalmente la calidad del mapa generado y el desempeño del robot en escenarios de distinta complejidad.

## 2. Descripción del Robot, Sensores y Actuadores

Para el desarrollo de este proyecto se utiliza el robot móvil **e-puck**, un micro-robot educativo de configuración diferencial, ampliamente documentado y optimizado para simulaciones de navegación y mapeo en el entorno Webots.

### 2.1. Actuadores (Sistema de Propulsión)
El movimiento del robot se basa en una configuración cinemática diferencial que consta de:
* **Motores de CC independientes:** Dos motores dedicados (`left wheel motor` y `right wheel motor`) que controlan las ruedas izquierda y derecha de forma independiente.
* **Modo de Operación:** Configurados en modo de control de velocidad (posición establecida en infinito `float('inf')`), lo que permite modificar dinámicamente la velocidad angular de cada rueda para ejecutar desplazamientos rectilíneos, curvas avanzadas y rotación sobre su propio eje.

### 2.2. Sensores
El e-puck cuenta con un conjunto integrado de sensores que permiten la percepción del entorno y la estimación de su propio estado:

#### A. Sensores de Rotación (Encoders)
* **Dispositivos:** Dos encoders de cuadratura (`left wheel sensor` y `right wheel sensor`) acoplados directamente a los ejes de los motores.
* **Función:** Miden continuamente la posición angular acumulada de cada rueda en radianes. Estos datos son la base fundamental para el cálculo de la odometría 2D, permitiendo estimar el desplazamiento lineal ($\Delta s$), la variación angular ($\Delta \phi$) y la pose global del robot $(x, y, \theta)$.

#### B. Sensores de Distancia (Infrarrojos)
Se utilizan los 8 sensores de proximidad infrarrojos integrados en la torreta del e-puck (`ps0` a `ps7`). Estos sensores entregan un valor crudo inversamente proporcional a la distancia del obstáculo. Para la implementación del algoritmo SLAM, las lecturas se transforman a métricas de distancia en metros [m] y se distribuyen espacialmente según sus ángulos fijos respecto al centro del robot (sentido antihorario):

| Sensor | Identificador en Código | Ángulo Relativo (rad) | Cobertura Espacial |
| :--- | :--- | :--- | :--- |
| **ps0** | `ps[0]` | -0.30 rad | Frontal-Derecho |
| **ps7** | `ps[1]` | 0.30 rad | Frontal-Izquierdo |
| **ps5** | `ps[2]` | 1.57 rad | Lateral Izquierdo |
| **ps2** | `ps[3]` | -1.57 rad | Lateral Derecho |
| **ps1** | `ps_extra[0]` | -1.20 rad | Diagonal Derecha |
| **ps3** | `ps_extra[1]` | -2.62 rad | Trasero-Derecho |
| **ps4** | `ps_extra[2]` | 2.62 rad | Trasero-Izquierdo |
| **ps6** | `ps_extra[3]` | 1.20 rad | Diagonal Izquierda |

Esta distribución perimetral completa de 360 grados permite al robot recolectar suficiente información del entorno en cada ciclo de simulación para alimentar los algoritmos de ray-casting y evasión reactiva de manera robusta.

## 3. Descripción de los Escenarios de Prueba

Para evaluar la robustez del sistema de evasión reactiva y la precisión de la construcción del mapa (SLAM), se diseñaron dos entornos en Webots con distintos niveles de complejidad geométrica y densidad de obstáculos. Ambos escenarios tienen un límite de tiempo definido para estandarizar la comparativa de la cobertura del mapa.

### 3.1. Escenario 1: Entorno Simple
* **Características:** Espacio predominantemente abierto con una baja densidad de obstáculos aislados (como muros rectos perimetrales y algunos objetos geométricos separados) y rutas de navegación directas.
* **Objetivo de la prueba:** Validar el funcionamiento base de la cinemática diferencial y la correcta configuración matemática del *ray-casting*. Al tener mucho espacio libre, permite verificar que el robot avanza en línea recta correctamente y que las rotaciones puntuales no generan una deriva significativa en la odometría, resultando en un mapa de ocupación nítido.

### 3.2. Escenario 2: Entorno Complejo
* **Características:** Entorno cerrado con alta densidad de obstáculos, pasillos estrechos, esquinas continuas y zonas de bloqueo (callejones ciegos).
* **Objetivo de la prueba:** Someter el controlador a condiciones de estrés. En este escenario, el robot se ve forzado a realizar maniobras evasivas y giros sobre su propio eje de manera constante. Esto permite evaluar dos cosas: primero, la confiabilidad de los umbrales de detección para no colisionar en espacios reducidos; y segundo, el impacto real de la acumulación de errores (deriva odométrica) sobre las celdas de la grilla de ocupación al momento de mapear giros repetitivos.

## 4. Explicación del Algoritmo Implementado

El sistema desarrollado se basa en una arquitectura de **SLAM Offline** (o diferido), separando la carga computacional en dos etapas principales: la recolección dinámica de datos y navegación autónoma en tiempo real (Webots), y el procesamiento matemático de la grilla de ocupación a posteriori (procesamiento de datos).

El algoritmo se divide en los siguientes módulos funcionales:

### 4.1. Estimación de Movimiento (Odometría 2D)
En cada *timestep* de la simulación, el controlador captura los pulsos de los *encoders* de ambas ruedas. El algoritmo calcula la diferencia de posición angular respecto al instante anterior para determinar cuánto avanzó la rueda izquierda ($\Delta s_l$) y la derecha ($\Delta s_r$). 
Utilizando las ecuaciones de cinemática diferencial del Laboratorio 1, se calcula el desplazamiento lineal central ($\Delta s$) y la variación de orientación ($\Delta \phi$). Finalmente, se actualiza iterativamente la pose global del robot $(x, y, \theta)$ mediante integración, normalizando el ángulo $\theta$ entre $-\pi$ y $\pi$. Adicionalmente, se ejecuta un Filtro de Kalman sobre las lecturas de los sensores frontales para analizar y registrar la estabilidad de las mediciones de distancia.

### 4.2. Estrategia de Exploración Reactiva
Para garantizar que el robot recolecte datos de toda la pista, se implementó una lógica de navegación autónoma basada en la evasión inteligente de obstáculos. 
* **Avance:** Mientras los sensores no detecten valores por encima de un umbral de seguridad ($150.0$), el robot avanza en línea recta a su velocidad base máxima.
* **Evasión:** Cuando detecta una amenaza, el algoritmo agrupa y evalúa los sensores del sector izquierdo (`ps7, ps6, ps5`) y derecho (`ps0, ps1, ps2`). Luego, aplica velocidades de signo opuesto a los motores para realizar una rotación sobre su propio eje hacia el sector que presente mayor espacio libre, retomando su camino una vez superado el obstáculo.

### 4.3. Recolección de Datos y Generación del Mapa (Ray-casting)
Durante toda la exploración, el código registra simultáneamente la pose calculada por la odometría y las lecturas crudas de los 8 sensores infrarrojos. Al finalizar el tiempo de simulación definido, esta información se exporta.
El mapa se construye externamente aplicando el algoritmo de *Ray-casting*: 
1. Se convierte el valor crudo del sensor a distancia métrica.
2. Se proyecta un rayo desde la coordenada $(x, y)$ del robot hacia el ángulo del sensor relativo a la orientación $\theta$.
3. Se discretiza el espacio en una matriz 2D, marcando las celdas atravesadas por el rayo como espacio **"libre"**, y la celda terminal de impacto (si detecta un objeto dentro del rango) como **"ocupada"**.

4. ## 5. Pseudocódigo de la Solución

El siguiente pseudocódigo describe el bucle principal de control y recolección de datos que se ejecuta en el simulador Webots:

```
INICIALIZAR conexión con el simulador Webots
INICIALIZAR motores (izquierdo y derecho) en modo velocidad a 0.0
INICIALIZAR 8 sensores infrarrojos de distancia (ps0 a ps7)
INICIALIZAR encoders de rueda (izquierdo y derecho)

DEFINIR CONSTANTES:
    VELOCIDAD_BASE = 3.0
    TIEMPO_MAXIMO = 120.0  // Segundos de exploración
    RADIO_RUEDA = 0.0205
    DISTANCIA_EJES = 0.052

INICIALIZAR VARIABLES DE ESTADO:
    x = 0.0, y = 0.0, theta = 0.0
    lista_datos = []
    tiempo_actual = 0.0

MIENTRAS (tiempo_actual < TIEMPO_MAXIMO) Y (Webots siga en ejecución) HACER:
    
    // 1. LECTURA DE SENSORES
    lecturas_IR = OBTENER_VALORES(sensores infrarrojos)
    frente_izq = MÁXIMO(ps7, ps6, ps5)
    frente_der = MÁXIMO(ps0, ps1, ps2)
    
    // 2. ESTIMACIÓN DE ODOMETRÍA (Cinemática Diferencial)
    delta_izq = encoder_izq_actual - encoder_izq_anterior
    delta_der = encoder_der_actual - encoder_der_anterior
    
    avance_lineal = (RADIO_RUEDA * delta_izq + RADIO_RUEDA * delta_der) / 2.0
    delta_angulo = (RADIO_RUEDA * delta_der - RADIO_RUEDA * delta_izq) / DISTANCIA_EJES
    
    x = x + avance_lineal * COSENO(theta + delta_angulo / 2.0)
    y = y + avance_lineal * SENO(theta + delta_angulo / 2.0)
    theta = NORMALIZAR_ANGULO(theta + delta_angulo)
    
    ACTUALIZAR valores anteriores de encoders
    
    // 3. NAVEGACIÓN Y EVASIÓN REACTIVA
    SI (frente_izq > UMBRAL_DETECCION) O (frente_der > UMBRAL_DETECCION) ENTONCES
        SI (frente_izq > frente_der) ENTONCES
            // Amenaza a la izquierda -> Rotar a la derecha
            velocidad_izq = VELOCIDAD_BASE * 0.8
            velocidad_der = -VELOCIDAD_BASE * 0.8
            estado = "EVADIENDO_DER"
        SINO
            // Amenaza a la derecha -> Rotar a la izquierda
            velocidad_izq = -VELOCIDAD_BASE * 0.8
            velocidad_der = VELOCIDAD_BASE * 0.8
            estado = "EVADIENDO_IZQ"
        FIN SI
    SINO
        // Espacio libre -> Avanzar en línea recta
        velocidad_izq = VELOCIDAD_BASE
        velocidad_der = VELOCIDAD_BASE
        estado = "AVANZANDO_RECTO"
    FIN SI
    
    APLICAR velocidades a los motores
    
    // 4. ALMACENAMIENTO DE DATOS PARA SLAM OFFLINE
    AGREGAR (tiempo_actual, x, y, theta, lecturas_IR, estado) a lista_datos
    
FIN MIENTRAS

// 5. FINALIZACIÓN Y EXPORTACIÓN
DETENER motores
EXPORTAR lista_datos a archivo "registro_sensores.csv"
