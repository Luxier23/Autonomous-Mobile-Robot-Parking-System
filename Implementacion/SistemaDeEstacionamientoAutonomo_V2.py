from machine import Pin, PWM
from time import sleep, sleep_us
from utime import ticks_ms, ticks_us
from math import sin, cos, atan2, tan, sqrt, pi

#### !!!!!!! EL COCHE SE ENCUNETRA VOLTEADO ESTE CÓDIGO, ES DECIR, COMO SI TUVIERA EL FRENTE EN LA PARTE TRASERA ####
#### ES POR ESO QUE HAY PARTES DONDE HAY UN '+ pi' o '- pi', ADEMAS DEL SIGNO NEGATIVO EN ALPHA = - Y v = -v     ####
#### ESO LE PERMITE AL COCHE SEGUIR EL SPLINE DE REVERSA, SI VAN A RECICLAR ESTE CODIGO, ASEGURENSE DE MODIFICAR ####
#### LO ANTES MENCIONADO PARA QUE EL COCHE SIGA LA RUTA DE FRENTE Y NO DE REVERSA ¡¡¡¡¡¡¡                        ####

######################################### ASIGNACION DE PINES ############################################

INICIO = Pin(13, Pin.IN)
# Pines de control motores
AIN1 = Pin(33, Pin.OUT)
AIN2 = Pin(32, Pin.OUT)
AIN1.value(0)
AIN2.value(0)
BIN1 = Pin(26,Pin.OUT)
BIN2 = Pin(27,Pin.OUT)
BIN1.value(0)
BIN2.value(0)

# Servo
SERVO = PWM(Pin(2),freq=50)
 
 # PWM velocidades motores
PWMA = PWM(Pin(25),freq=18000)
PWMB = PWM(Pin(14),freq=18000)

global QEM
QEM = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0]

# Contadores encoders
global counter_LEFT, counter_RIGHT, last_RIGHT, last_LEFT
counter_LEFT = 0
counter_RIGHT = 0
last_RIGHT = 0
last_LEFT = 0

##################################### DEFINICION DE FUNCIONES ############################################    

def Adaptative_Control(w_D,w,integral,dt):        
    e = w - w_D
    integral = -0.1*w*e*dt + integral
    integral = max(min(integral, 5), -5) 

    V = (1/2.5)*(-1.5*e + (w*integral))
    V = max(min(V, 12), -12)
    
    return integral, V

def Left_Motor(speed):
     if speed > 0:
          AIN1.value(1)
          AIN2.value(0)
          PWMA.duty_u16(int(speed*65535/100))
     elif speed < 0:
          AIN1.value(0)
          AIN2.value(1)
          PWMA.duty_u16(int((-speed)*65535/100))
     else:
          AIN1.value(0)
          AIN2.value(0)
          PWMA.duty_u16(0)
          
def Right_Motor(speed):
     if speed > 0:
          BIN1.value(1)
          BIN2.value(0)
          PWMB.duty_u16(int(speed*65535/100))
     elif speed < 0:
          BIN1.value(0)
          BIN2.value(1)
          PWMB.duty_u16(int((-speed)*65535/100))
     else:
          BIN1.value(0)
          BIN2.value(0)
          PWMB.duty_u16(0)
      
def handle_interrupt_LEFT(pin_LEFT):
    global counter_LEFT, last_LEFT, QEM
    
    new_A = INTERRUPT_LEFT.value()
    new_B = INTERRUPT_LEFT_A.value()
    
    index = (last_LEFT << 2) | (new_A << 1) | new_B
    
    index &= 0x0F 
    
    counter_LEFT += QEM[index]
    
    last_LEFT = index & 0x03
    
    
def handle_interrupt_RIGHT(pin_RIGHT):
    global counter_RIGHT, last_RIGHT, QEM
    
    new_A = INTERRUPT_RIGHT.value()
    new_B = INTERRUPT_RIGHT_A.value()
    
    index = (last_RIGHT << 2) | (new_A << 1) | new_B
    index &= 0x0F
    
    counter_RIGHT -= QEM[index]
    
    last_RIGHT = index & 0x03

def Direction_Angle(angle):
    global offset_angle
    angle = angle + offset_angle # Ajuste de offset de tu servo
    min_duty = 1638
    max_duty = 8200
    duty = int(min_duty +(angle/180)*(max_duty-min_duty))
    SERVO.duty_u16(duty)
    
def Evaluar_Splines(q, t_vec, time_val):
    n = len(t_vec)
    if time_val >= t_vec[n-1]:
        k = n - 2
        time_val = t_vec[n-1] 
    else:
        k = 0
        for i in range(n - 1):
            if t_vec[i] <= time_val and time_val <= t_vec[i+1]:
                k = i
                break
    dt_segment = t_vec[k+1] - t_vec[k]
    dq_segment = q[k+1] - q[k]
    
    if dt_segment == 0:
        slope = 0
    else:
        slope = dq_segment / dt_segment

    qk = q[k] + slope * (time_val - t_vec[k])
    qpk = slope 
    return qk, qpk

################### DETECCION DE CAJON #################
def Slot_detection():
    
    global D,L,R,N,integral_R,integral_L,dt,K_DIRECCION

    LAST_counter_LEFT = counter_LEFT
    LAST_counter_RIGHT = counter_RIGHT
    EXECUTION_START = ticks_ms()

    v = 0.12
    w = 0
    ALPHA_DEG = 0
    
    w_R = (v + w*L)/R
    w_L = (v - w*L)/R
    
    MAX_VELOCITY = 25
    w_R = max(min(w_R, MAX_VELOCITY), -MAX_VELOCITY)
    w_L = max(min(w_L, MAX_VELOCITY), -MAX_VELOCITY)
    
    while INICIO.value() == 1:
            START = ticks_us()                         
            t = (ticks_ms()-EXECUTION_START)/1000

            DELTA_counter_LEFT = counter_LEFT - LAST_counter_LEFT
            LAST_counter_LEFT = counter_LEFT            
            
            DELTA_counter_RIGHT = counter_RIGHT - LAST_counter_RIGHT
            LAST_counter_RIGHT = counter_RIGHT
            
            w_R_o = 2*pi*DELTA_counter_RIGHT/(N*dt)
            w_L_o = 2*pi*DELTA_counter_LEFT/(N*dt)
         
            #################### CONTROL ADAPTATIVO ###################
            
            integral_R, V_R = Adaptative_Control(w_R,w_R_o,integral_R,dt)
            integral_L, V_L = Adaptative_Control(w_L,w_L_o,integral_L,dt)
            
            Left_Motor(V_L*100/12)                             
            Right_Motor(V_R*100/12)                            
            Direction_Angle(ALPHA_DEG*K_DIRECCION)  
            
            END = ticks_us()
            LAPSE = END-START
            if LAPSE < 1000000*dt:                          
                sleep_us(int(1000000*dt - LAPSE)) # sleep_us requiere entero

    Left_Motor(0)                               
    Right_Motor(0)                          
    Direction_Angle(0)
    
### FINAL DE def Slot_detection

    
################### PRIMER AJUSTE ##################
def First_adjustment():

    global D,L,ALPHA,R,N,integral_R,integral_L,dt,Kx,Ky,Kv,Kw,K_ALPHA,K_DIRECCION
    
    S = 20

    shortest_distance = 0.35
    x_D = 0.6
    y_D = 0.56 #0.56

    xi = shortest_distance + 0.085 #Distancia del centro al sensor

    p = [xi, 0, pi/2]

    LAST_counter_LEFT = counter_LEFT
    LAST_counter_RIGHT = counter_RIGHT
    EXECUTION_START = ticks_ms()

    # WHILE
    while (ticks_ms()-EXECUTION_START)/1000 <= S:
            START = ticks_us()                        
            t = (ticks_ms()-EXECUTION_START)/1000
            
            # Control
            ux = Kx * (x_D - p[0])
            uy = Ky * (y_D - p[1])

            v = (ux * cos(p[2])) + (uy * sin(p[2]))
            w = ((-1/D) * ux * sin(p[2])) + ((1/D) * uy * cos(p[2]))
            
            # Velocidades con su respectiva ganancia
            v = v * Kv              # Ganancia V
            w = w * Kw				# Ganancia W
            
            # Saturacion
            MAX_VELOCITY = 0.5
            v = max(min(v, MAX_VELOCITY), -MAX_VELOCITY)

            # Calculo de la direccion
            ALPHA = atan2(w*D, v) * K_ALPHA                       
            
            # Cota de direccion
            BOUND_DEG = 40                             
            BOUND_RAD = BOUND_DEG*pi/180                
            ALPHA = max(min(ALPHA,BOUND_RAD),-BOUND_RAD) 
            ALPHA_DEG = ALPHA*180/pi

            w = tan(ALPHA)*v/D;
            
            # Calculo del diferencial
            w_R = (v + w*L)/R
            w_L = (v - w*L)/R
            w_R = (w_R+w_L)/2
            w_L = w_R
            
            # Saturacion
            MAX_VELOCITY = 25
            w_R = max(min(w_R, MAX_VELOCITY), -MAX_VELOCITY)
            w_L = max(min(w_L, MAX_VELOCITY), -MAX_VELOCITY)
            
            # Odometria
            DELTA_counter_LEFT = counter_LEFT - LAST_counter_LEFT
            LAST_counter_LEFT = counter_LEFT            
            
            DELTA_counter_RIGHT = counter_RIGHT - LAST_counter_RIGHT
            LAST_counter_RIGHT = counter_RIGHT
            
            w_R_o = 2*pi*DELTA_counter_RIGHT/(N*dt)
            w_L_o = 2*pi*DELTA_counter_LEFT/(N*dt)
         
            #################### CONTROL ADAPTATIVO ###################
            
            integral_R, V_R = Adaptative_Control(w_R,w_R_o,integral_R,dt)
            integral_L, V_L = Adaptative_Control(w_L,w_L_o,integral_L,dt)
            
            Left_Motor(V_L*100/12)                             
            Right_Motor(V_R*100/12)                            
            Direction_Angle(ALPHA_DEG*K_DIRECCION)  
            
            vs = (2*pi*R*(DELTA_counter_LEFT/N) + 2*pi*R*(DELTA_counter_RIGHT/N))/2
            DELTA_p = [vs*cos(p[2]), vs*sin(p[2]), vs*tan(ALPHA)/D] 
            
            p[0] = p[0] + DELTA_p[0]
            p[1] = p[1] + DELTA_p[1]
            
            p[2] = p[2] + DELTA_p[2] # Actualizar theta
            
            END = ticks_us()
            LAPSE = END-START
            if LAPSE < 1000000*dt:                          
                sleep_us(int(1000000*dt - LAPSE)) # sleep_us requiere entero
                
    Left_Motor(0)                               
    Right_Motor(0)                          
    Direction_Angle(0)
    
    return p
### FINAL DE def Fisrt_adjustment

################### SEGUNDO AJUSTE ################3
def Second_adjustment(xi,yi,p_i):
    
    global D,L,ALPHA,R,N,integral_R,integral_L,dt,Kx,Ky,Kv,Kw,K_ALPHA,K_DIRECCION
    
    S = 1000

    p = [xi, yi, p_i]

    if (p[2] < pi/2):
        ALPHA = 10
    elif p[2] > pi/2:
        ALPHA = -10
    else:
        ALPHA = 0

    LAST_counter_LEFT = counter_LEFT
    LAST_counter_RIGHT = counter_RIGHT
    EXECUTION_START = ticks_ms()

    print(pi)
    v = 0.1
    
    # WHILE
    while (ticks_ms()-EXECUTION_START)/1000 <= S:
            START = ticks_us()                         
            t = (ticks_ms()-EXECUTION_START)/1000
            
            
            MAX_VELOCITY = 0.9
            v = max(min(v, MAX_VELOCITY), -MAX_VELOCITY)                     
            
            # Cota de direccion
            BOUND_DEG = 40                             
            BOUND_RAD = BOUND_DEG*pi/180                
            ALPHA = max(min(ALPHA,BOUND_RAD),-BOUND_RAD) 
            ALPHA_DEG = ALPHA*180/pi
            print(ALPHA_DEG)
            w = tan(ALPHA)*v/D;
            
            
            # Calculo del diferencial
            w_R = (v + w*L)/R
            w_L = (v - w*L)/R
            w_R = (w_R+w_L)/2
            w_L = w_R
            
            # Saturacion
            MAX_VELOCITY = 25
            w_R = max(min(w_R, MAX_VELOCITY), -MAX_VELOCITY)
            w_L = max(min(w_L, MAX_VELOCITY), -MAX_VELOCITY)
            print(w_L,w_R)
            
            # Odometria
            DELTA_counter_LEFT = counter_LEFT - LAST_counter_LEFT
            LAST_counter_LEFT = counter_LEFT            
            
            DELTA_counter_RIGHT = counter_RIGHT - LAST_counter_RIGHT
            LAST_counter_RIGHT = counter_RIGHT
            
            w_R_o = 2*pi*DELTA_counter_RIGHT/(N*dt)
            w_L_o = 2*pi*DELTA_counter_LEFT/(N*dt)
            
            #w_R_o = w_R
            #w_L_o = w_L
         
            #################### CONTROL ADAPTATIVO ###################
            
            integral_R, V_R = Adaptative_Control(w_R,w_R_o,integral_R,dt)
            integral_L, V_L = Adaptative_Control(w_L,w_L_o,integral_L,dt)
            
            Left_Motor(V_L*100/12)                             
            Right_Motor(V_R*100/12)                            
            Direction_Angle(ALPHA_DEG*K_DIRECCION)  
            
            vs = (2*pi*R*(DELTA_counter_LEFT/N) + 2*pi*R*(DELTA_counter_RIGHT/N))/2
            DELTA_p = [vs*cos(p[2]), vs*sin(p[2]), vs*tan(ALPHA)/D] 
            
            p[0] = p[0] + DELTA_p[0]
            p[1] = p[1] + DELTA_p[1]
            
            p[2] = p[2] + DELTA_p[2] # Actualizar theta
            
            e_theta = p[2] - pi/2
            
            print(e_theta)
            if abs(e_theta) < 0.03:
                break
            
            END = ticks_us()
            LAPSE = END-START
            if LAPSE < 1000000*dt:                          
                sleep_us(int(1000000*dt - LAPSE)) # sleep_us requiere entero

    Left_Motor(0)                               
    Right_Motor(0)                          
    Direction_Angle(0)
    
    return p
### FINAL DE def Second_adjustment

################### MANIOBRA DE ESTACIONADO ############3

def Parking(x,y,p):
    
    global D,L,ALPHA,R,N,integral_R,integral_L,dt,Kx,Ky,Kv,Kw,K_ALPHA,K_DIRECCION

    num_points = len(x)
    S = 7.82 
    tx = [i * (S / (num_points - 1)) for i in range(num_points)]
    ty = [i * (S / (num_points - 1)) for i in range(num_points)]
    qx = x
    qy = y

    p = [p[0], p[1], p[2]]

    LAST_counter_LEFT = counter_LEFT
    LAST_counter_RIGHT = counter_RIGHT
    EXECUTION_START = ticks_ms()

    ALPHA_DEG = 40
    flag = 0
    # WHILE
    while (ticks_ms()-EXECUTION_START)/1000 <= S:
            START = ticks_us()                         
            t = (ticks_ms()-EXECUTION_START)/1000
            
            
            v = 0.15

            if (t < S - 3.75):
                ALPHA_DEG = 40
            else:
                ALPHA_DEG = -40
                
            
            v = -v
            w = 0
            
            # Calculo del diferencial
            w_R = (v - w*L)/R
            w_L = (v + w*L)/R
            #w_R = (w_R + w_L)/2
            #w_L = w_R
            
            # Saturacion
            MAX_VELOCITY = 25
            w_R = max(min(w_R, MAX_VELOCITY), -MAX_VELOCITY)
            w_L = max(min(w_L, MAX_VELOCITY), -MAX_VELOCITY)
            print(w_L,w_R)
            
            # Odometria
            DELTA_counter_LEFT = counter_LEFT - LAST_counter_LEFT
            LAST_counter_LEFT = counter_LEFT            
            
            DELTA_counter_RIGHT = counter_RIGHT - LAST_counter_RIGHT
            LAST_counter_RIGHT = counter_RIGHT
            
            w_R_o = 2*pi*DELTA_counter_RIGHT/(N*dt)
            w_L_o = 2*pi*DELTA_counter_LEFT/(N*dt)
            
            #
            #w_R_o = w_R
            #w_L_o = w_L
         
            #################### CONTROL ADAPTATIVO ###################
            
            integral_R, V_R = Adaptative_Control(w_R,w_R_o,integral_R,dt)
            integral_L, V_L = Adaptative_Control(w_L,w_L_o,integral_L,dt)
            
            Left_Motor(V_L*100/12)                             
            Right_Motor(V_R*100/12)                            
            Direction_Angle(ALPHA_DEG*K_DIRECCION)  
            
            ###########################################################
            
            p[2] = p[2] - pi # Actualizar theta
            
            vs = (2*pi*R*(DELTA_counter_LEFT/N) + 2*pi*R*(DELTA_counter_RIGHT/N))/2
            DELTA_p = [vs*cos(p[2]), vs*sin(p[2]), vs*tan(ALPHA)/D] 
            
            p[0] = p[0] + DELTA_p[0]
            p[1] = p[1] + DELTA_p[1]
            
            p[2] = p[2] + DELTA_p[2] # Actualizar theta
            
            END = ticks_us()
            LAPSE = END-START
            if LAPSE < 1000000*dt:                          
                sleep_us(int(1000000*dt - LAPSE)) # sleep_us requiere entero

    Left_Motor(0)                               
    Right_Motor(0)                          
    Direction_Angle(0)
################# FIN DE MANIOBRA DE ESTACIONADO ######################

####################################### ASIGNACION DE INTERRUPCIONES ############################################

INTERRUPT_LEFT = Pin(36,Pin.IN,Pin.PULL_UP)
INTERRUPT_LEFT.irq(trigger = 3, handler = handle_interrupt_LEFT)
INTERRUPT_RIGHT = Pin(39, Pin.IN,Pin.PULL_UP)
INTERRUPT_RIGHT.irq(trigger = 3, handler = handle_interrupt_RIGHT)
INTERRUPT_LEFT_A = Pin(34,Pin.IN,Pin.PULL_UP)
INTERRUPT_LEFT_A.irq(trigger = 3, handler = handle_interrupt_LEFT)
INTERRUPT_RIGHT_A = Pin(35, Pin.IN,Pin.PULL_UP)
INTERRUPT_RIGHT_A.irq(trigger = 3, handler = handle_interrupt_RIGHT)

########################################### MEDIDAS TIPO COCHE ##############################################
global D,L,ALPHA,R,N,integral_R,integral_L,dt,Kx,Ky,Kv,Kw,K_ALPHA,K_DIRECCION

D = 0.29                    # Distancia entre las ruedas a lo largo
L = 0.1675
ALPHA = 0                   # Angulo inicial
R = 0.033                   # Radio de las ruedas
N = 900              		# Espacios del encoder
integral_R = 0.57825108
integral_L = 0.48061204

########################################### CÓDIGO TIPO COCHE SPLINES ##############################################

dt = 0.1                   # Paso de tiempo
#S = 10                     # Tiempo de ejecucion TOTAL

Kx = 1
Ky = 1
Kv = 1                   # Ganancia v
Kw = 1                   # Ganancia w
K_ALPHA = 1                 # Ganancia Alpha
K_DIRECCION = -2        # Ganancia Direccion - servo

##################### CARGAR RUTA #######################

with open('path.csv', 'r') as archivo:
    contador = 1
    for linea in archivo:
        # Eliminar saltos de línea y separar por coma
        fila = linea.strip().split(',')
        if contador:
            x=[]
            for numero in fila:
                x.append(float(numero)/10)
            contador = contador - 1
        else:
            y=[]
            for numero in fila:
                y.append(float(numero)*1/10)
                
global offset_angle

offset_angle = 90

while INICIO.value() == 0:
    pass

Slot_detection()
print("Lugar de estacionamiento encontrado")

while INICIO.value() == 0:
    pass

Kv = 1
Kw = 10
p = First_adjustment()
print("Primer ajuste terminado")
p = Second_adjustment(x[0],y[0],p[2])
print("Segundo ajuste terminado")
Kv = 1
sleep(3)
p = [x[0], y[0], 0]
Kv = 1
Kw = 1
Parking(x,y,p)
print("Coche estacionado")


