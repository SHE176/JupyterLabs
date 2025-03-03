### FUNCIONES PARA CÁLCULOS CON GPS


#!!!  gps.ZERO, gps.INCLINATION, gps.R_TER = gps.load_geodata(name)
#     gps.MED, gps.ZOOM                    = gps.load_reference(name)
'''

 FUNCIONES:
 
     
 load_geodata(name)
 ··············································································
 earth_radius(lat)
 distance(A, B)
 course(A, B)
 polar(A, B)
 locate(dis, ang, A = CORNER_LU)
 mapping()
 GPS2XY(point)
 XY2GPS(point)
 findPosition(ll1, ll2, xy1, xy2, xy = (0,0))
 ··············································································
 in_polygon(point, vert)
 ··············································································
 cartesian_distance(p1, p2)
 centerLine(pol)
 centralize(coor, line)
 apply_offset(pointXY, lineXY, OX, OY)
 ··············································································
 eq_rect(line)
 dist_rect(p, rect)
 dist_perpendicular(p, p1, p2)
 ··············································································
 
'''

# =============================================================================
# <<< MÓDULOS >>>
# =============================================================================

from math import radians, degrees, sqrt, pi
from math import sin, cos
from math import asin, atan2
from copy import deepcopy

import json

# =============================================================================
# <<< PARÁMETROS >>>
# =============================================================================

# Radios de la tierra [m]
R_AVG = 6371008.8           # Radio medio
R_POL = 6356752.3142        # Radio polar
R_EQU = 6378137.0           # Radio meridiano  

# Cálculos para 3xS
REF_A      = None           # Punto A del Terminal Management
REF_B      = None           # Punto B del Terminal Management        
REF_OFFSET = (None, None)   # Offset XY de la linea de BERTH para 3xS

# Parámetro que define el sentido del eje Y
#FLIP_Y = -1

# =============================================================================
# <<< GEODATA >>>
# =============================================================================

TERMINAL    = None
ZERO        = None          # Coordenadas Lat / Lon del punto ZERO
INCLINATION = None          # Inclinación del mapa de la terminal en grados
R_TER       = R_AVG         # Radio de la terminal según su Lat

# =============================================================================
# <<< REFERENCE >>>
# =============================================================================

MED  = None
ZOOM = None 

# =============================================================================
# <<< FUNCION DE CARGA >>>
# =============================================================================

#!!!       gps.ZERO, gps.INCLINATION, gps.R_TER = gps.load_geodata(name)
#          gps.MED, gps.ZOOM                    = gps.load_reference(name)


### Carga datos para hacer trnsformaciones de coordenadas
def load_geodata(name):

    global ZERO, INCLINATION, R_TER, TERMINAL
    
    path = 'geodata.json'
    
    with open(path) as file:
        dic = json.load(file)
    
    ZERO = dic[name]['ZERO']
    INCLINATION = dic[name]['INCLINATION']
    R_TER = dic[name]['R_TER']
    TERMINAL = name

# Carga datos de encuadre del mapa
def load_reference(name):

    global MED, ZOOM, TERMINAL
    
    path = 'geodata.json'
    
    with open(path) as file:
        dic = json.load(file)
    
    MED = dic[name]['MED']
    ZOOM = dic[name]['ZOOM']
    TERMINAL = name

# =============================================================================
# <<< FUNCIONES GPS >>>
# =============================================================================

# ································································ Radio tierra

# Calcula el radio de la tierra a una latitud dada, considerando un elipsoide
# y conocidos el radio polar menor (R_POL) y radio ecuatorial mayor (R_EQU)

### Radio de la tierra a latitud dada
def earth_radius(lat):
    
    # Definición Wiki
    
    num = (R_EQU**2 * cos(lat))**2 + (R_POL**2 * sin(lat))**2
    den = (R_EQU * cos(lat))**2 + (R_POL * sin(lat))**2
    
    R_MED = sqrt(num / den)
    
    return round(R_MED, 3)

# ·································································· Normalizar

### Normaliza [lat, lon] a rangos válidos
def normalize(coor):
    
    ## Normalize puntos a [-90, 90] lat y [-180, 180] lon.
    
    lat = coor[0]
    lon = coor[1]
    
    lat = (lat + 90) % 360 - 90
    if lat > 90:
        lat = 180 - lat
        lon += 180
    lon = (lon + 180) % 360 - 180
    return (round(lat, 6), round(lon, 6))

# ··································································· Haversine

# La distancia la calcula por el método Haversine. En comparativa con otros 
# métodos, es el que ha probado cometer menos error. 
# Se hace esto por la leve deformación sufrida por la curvatura terrestre.

### Distancia por el método Haversine
def distance(A, B):
    
    try:
        lat1 = radians(A[0])
        lon1 = radians(A[1])
        lat2 = radians(B[0])
        lon2 = radians(B[1])
        
        a = lat2 - lat1
        b = lon2 - lon1
        c = sin(a * 0.5) ** 2 + cos(lat1) * cos(lat2) * sin(b * 0.5) ** 2
        d = 2 * R_TER * asin(min(1, sqrt(c)))
        
        return d
    
    except:
        print('[lat, lon] bad definition')
        return None
    

# El curso es el ángulo inicial desde el punto de partida, en grados.
# Para saber el ángulo de llegada, basta con intercambiar los puntos.

### Rumbo inicial de A hasta B en rango [0, 360]
def course(A, B):

    try:
        lat1 = radians(A[0])
        lon1 = radians(A[1])
        lat2 = radians(B[0])
        lon2 = radians(B[1])
        
        dlon = lon2 - lon1
        
        num = sin(dlon) * cos(lat2)
        den = cos(lat1) * sin(lat2) - (sin(lat1)* cos(lat2) * cos(dlon))
        ang = atan2(num, den)
        
        # Lo queremos en grados entre [0, 360], no entre [-180, +180]
        ang = degrees(ang)
        dang = (ang + 360) % 360
        
        return dang
    
    except:
        print('[lat, lon] bad definition')
        return None

# Devuelve los datos en coordenadas polares del desplazamiento de A a B.
# En definitiva, aglutina las funciones anteriores.

### Definición polar del desplazamiento del punto A hasta el B
def polar(A, B):
    
    dis = distance(A, B)
    ang = course(A, B)
    
    return dis, ang
    

# Función inversa. Dado un punto inicial, una distancia y un ángulo de partida,
# en grados, o lo que es lo mismo, su definición polar, devuelve el 
# punto resultante

### Inverso - Funciona con Trig Espacial - OP
def locate(dis, ang, A = ZERO):
    
    try:
        lat1 = radians(A[0])
        lon1 = radians(A[1])
        
        ang = radians(ang)
        
        rel = dis / R_TER
        
        lat2 = asin ( sin(lat1) * cos(rel) + cos(lat1) * sin(rel) * cos(ang))
        lon2 = lon1 + atan2( (sin(ang) * sin(rel) * cos(lat1)),
                            cos(rel) - sin(lat1) * sin(lat2))
        
        return (degrees(lat2), degrees(lon2))
    
    except:
        print('[lat, lon] bad definition')
        return None

# ·································································· Mapeo a XY

# Esta función mapea punto [lat, lon] a XY
# realiza usando la función GPS_distance conociendo ZERO e INCLINATION

# Se considera el origen de coordenadas XY a la esquina superior izquierda y se
# tiene en cuenta la inclinación del mapa de la terminal con respecto a los ejes
# de latitud y longitud terrestres    

### Definir mapa con los puntos de esquina de la terminal y su inclinación
def mapping():
    
    # Se intoducen los datos mostrados en el Terminal Management
    # REF_A, REF_B, REF_OFFSET correspondientes a la definición de 3xS
    global REF_A, REF_B, REF_OFFSET
    global ZERO
    global INCLINATION, R_TER
    
    # Calcula el radio medio de la terminal
    R_TER = earth_radius(REF_A[0])
    
    # Inclinación de la terminal
    INCLINATION = course(REF_A, REF_B)
    
    # Origen de coordenadas
    ZERO_prev = locate(REF_OFFSET[0], INCLINATION + 180, REF_A)
    ZERO = locate(REF_OFFSET[1], INCLINATION - 90, ZERO_prev)
    #ZERO = locate(REF_OFFSET[1], INCLINATION + (90* FLIP_Y), ZERO_prev)
    
    return ZERO, INCLINATION, R_TER
    

### Coordenadas GPS a XY
def GPS2XY(point, red = None):
    
    if point is None:
        print('[lat, lon] bad definition')      
        return None
    
    # Inclinación propia del mapa de la terminal
    tinc = radians(INCLINATION)
    
    # Distancia desde el origen
    dist = distance(ZERO, point)
    cour = radians(course(ZERO, point))
    
    # Inclinación propia del mapa de la terminal
    tinc = radians(INCLINATION)
    
    # Dividir la distancia en XY, teniendo en cuenta la inclinación
    # Ahora si puede ser negativa alguna coordenada (fuera de terminal, claro)
    # Se permite precisamente para crear posteriormente la función in_terminal
    X = - dist * cos(pi - cour + tinc)
    Y = dist * sin(pi - cour + tinc)
    
    # Y = - FLIP_Y * dist * sin(pi - cour + tinc)
    
    # Para normalizar a valores [0, 1] se debe dividir por las dimensiones
    #X /= WIDTH
    #Y /= HEIGTH
    
    if X == -0.0: X = +0.0
    if Y == -0.0: Y = +0.0
    
    # Redondeo opcional
    if red is not None:
        X = round(X, red)
        Y = round(Y, red)
    
    return (X, Y) ### OPCIONAL

# Se usa el signo de XY por si es un punto de fuera de la terminal

### Coordenadas XY a GPS
def XY2GPS(point, red = None):
    
    if point is None:
        print('[X, Y] bad definition')      
        return None
    
    x, y = point
    
    # Distancia total
    dist = sqrt( pow(x, 2) + pow(y, 2))
    
    # Ángulo - incluye la inclinación de la terminal
    ang = degrees(atan2(x,  -y)) - (90 -INCLINATION)
    #ang = degrees(atan2(x,  FLIP_Y * y)) - (90 -INCLINATION)
    
    p = locate(dist, ang, ZERO)
    if red is not None:
        p = (round(p[0], red),round(p[1], red) )
    
    # Calcula punto
    return p

# ···························································· Encontrar origen

### Encuentra la posición del ZERO (o de cualquier otro punto)
def findPosition(ll1, ll2, xy1, xy2, xy = (0,0)):
    
    try:
        lat1 = radians(ll1[0])
        lon1 = radians(ll1[1])
        lat2 = radians(ll2[0])
        lon2 = radians(ll2[1])
        
        x1   = xy1[0]
        y1   = xy1[1]
        x2   = xy2[0]
        y2   = xy2[1]
        
        x   = xy[0]
        y   = xy[1]
        
    except:
        return
    
    # Interpolación lineal
    lat0 = (y - y1) / (y2 - y1) * (lat2 - lat1) + lat1
    lon0 = (x - x1) / (x2 - x1) * (lon2 - lon1) + lon1
    
    zero = (degrees(lat0), degrees(lon0))
    
    return normalize(zero)

# =============================================================================
# <<< FUNCIONES GEOMÉTRICAS Y DE CORRECCIÓN >>>
# =============================================================================
    
# ···························································· Algoritmo Radial

### ALGORITMO RADIAL - Ver si un punto pertenece a un polígono dado
def in_polygon(point, vert):
    
    vertex = deepcopy(vert)
    
    if point is None:
        print('Punto inválido')
        return
    
    if not vertex or len(vertex) < 3:
        print('No existe polígono')
        return
    
    # Variables
    suma = 0
    sides = len(vertex)
    
    # Cerramos el polígono con el primer punto
    vertex.append(vertex[0])
    
    # Recorremos cada lado del polígono
    for v in range(sides):
        
        # Vectores desde p a los vértices
        v1 = (vertex[v][0] - point[0], vertex[v][1] - point[1])
        v2 = (vertex[v + 1][0] - point[0], vertex[v + 1][1] - point[1])
        
        # Productos escalar y vectorial
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        vec = v1[0] * v2[1] - v1[1] * v2[0]
        
        # Ángulo con signo rotacional
        ang = degrees(atan2(vec , dot))
        
        # Pertenece al perímetro
        if abs(ang) == 180: return False
        
        # Acumular ángulo
        suma += ang
    
    # Si es 0, está afuera. Si es múltiplo de 360, dentro
    if round(suma, 6) == 0: return False
    else: return True
    
# ································································· centralizar

### Distancia Pitagórica entre dos puntos
def cartesian_distance(p1, p2):
    
    return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2))
    

### Calcula la linea central de un polígono (Cuadrilátero)
def centerLine(pol):
    
    # Sacar nervio del poligono donde proyectar
    px1 = (pol[0][0] + pol[3][0]) * 0.5
    py1 = (pol[0][1] + pol[3][1]) * 0.5
    
    px2 = (pol[1][0] + pol[2][0]) * 0.5
    py2 = (pol[1][1] + pol[2][1]) * 0.5
    
    p1 = (round(px1, 6), round(py1, 6))
    p2 = (round(px2, 6), round(py2, 6))
    
    return p1, p2


### Toda coordenada dentro del polígono dado, se aproxima a la centerLine
def centralize(coor, line):
    
    # Ver si viene definida la linea central
    # Si es necesario, definirla por el bloque

    p1, p2 = line
    
    # Vector unitario en direccion del radio
    v = (p2[0] - p1[0], p2[1] - p1[1])
    m = sqrt(pow(v[0], 2) + pow(v[1], 2))
    u = (v[0] / m, v[1]/ m)
    
    # Vector p1 al punto
    w = (coor[0] - p1[0], coor[1] - p1[1])
    
    # Producto escalar
    dot = w[0] * u[0] + w[1] * u[1]
    
    # Proyección
    proy = (dot * u[0], dot * u[1])
    
    # Punto final
    point = (p1[0] + proy[0], p1[1] + proy[1])
    
    return point

### Aplica un offset en coordenadas Cartesianas
def apply_offset(pointXY, lineXY, OX, OY):
    
    p1, p2 = lineXY
    
    # Calcularmos vectores directores
    m = cartesian_distance(p1, p2)
    u = ( (p2[0] - p1[0]) / m, (p2[1] - p1[1]) / m )
    w = ( -u[1], u[0])
    
    #offset en X
    pointXY = ( pointXY[0] + u[0] * OX , pointXY[1] + u[1] * OX )
    
    #offset en X
    pointXY = ( pointXY[0] + w[0] * OY , pointXY[1] + w[1] * OY )
    
    return pointXY

# ······························································ expandir linea

### Expande una linea para crear un cuadrilatero a una distancia dada
def expand_line(line, cant):

    # Primero sacar el ángulo de la linea
    ang = course(line[0], line[1])
    
    # Ahora hay que hacer el polígono
    p0 = locate(cant, ang - 90, line[0])
    p1 = locate(cant, ang - 90, line[1])
    p2 = locate(cant, ang + 90, line[1])
    p3 = locate(cant, ang + 90, line[0])
    
    return [p0, p1, p2, p3]