# tareas-5-micros-
# Brazo Robótico con Pinza · PyBullet + ESP32

**Actividad 5 · Universidad Militar Nueva Granada**
Control en tiempo real de un brazo robótico simulado (URDF) mediante potenciómetros leídos por un ESP32.

`ESP32` · `UART 115200` · `ADC 12 bits` · `PyBullet` · `URDF` · `Python`

---

## Demostración

[![Ver video de funcionamiento](https://img.youtube.com/vi/ID_DEL_VIDEO/0.jpg)](https://youtu.be/ID_DEL_VIDEO)


---

## Contenido

1. [Objetivo](#1-objetivo)
2. [Arquitectura del sistema](#2-arquitectura-del-sistema)
3. [Modelo del robot (URDF)](#3-modelo-del-robot-urdf)
4. [Hardware y conexiones](#4-hardware-y-conexiones)
5. [Firmware del ESP32](#5-firmware-del-esp32)
6. [Software de control en Python](#6-software-de-control-en-python)
7. [Instalación y ejecución](#7-instalación-y-ejecución)
8. [Validación en tiempo real](#8-validación-en-tiempo-real)
9. [Solución de problemas](#9-solución-de-problemas)
10. [Estructura del repositorio](#10-estructura-del-repositorio)

---

## 1. Objetivo

Controlar en tiempo real un brazo robótico con pinza, descrito en un archivo URDF y simulado en PyBullet, usando señales analógicas leídas por un microcontrolador ESP32 y transmitidas al computador por UART.

| Entrada física | Articulación controlada |
|----------------|-------------------------|
| Potenciómetro 1 | `joint_1` · giro de la base |
| Potenciómetro 2 | `joint_2` · movimiento del codo |
| Potenciómetro 3 | `joint_dedo_izq` y `joint_dedo_der` · apertura de la pinza |

---

## 2. Arquitectura del sistema

```
 ┌───────────────┐   ADC 12 bits    ┌──────────┐   USB / UART    ┌────────────────────┐
 │ 3 potenció-   │ ───────────────▶ │  ESP32   │ ──────────────▶ │  control_brazo.py  │
 │ metros 10 kΩ  │  GPIO 34/35/32   │ Brazo.ino│   115200 baud   │  (Python + pyserial)│
 └───────────────┘                  └──────────┘  "t_ms,a1,a2,a3"└─────────┬──────────┘
                                                                             │ setJointMotorControl2
                                                                             ▼
                                                                  ┌────────────────────┐
                                                                  │ PyBullet + brazo.urdf│
                                                                  │  simulación 3D      │
                                                                  └────────────────────┘
```

**Flujo de datos:**

1. El ESP32 lee 3 canales analógicos a 50 Hz, promediando 8 muestras por canal para reducir ruido.
2. Envía cada lectura como una línea CSV por el puerto serial.
3. Python interpreta la línea, convierte cada valor ADC a una posición articular y la aplica al robot.
4. PyBullet avanza la simulación y se actualiza la vista 3D.

---

## 3. Modelo del robot (URDF)

El archivo `brazo.urdf` define un robot de 6 eslabones y 5 articulaciones.

| Articulación | Tipo | Eje | Límites | Controlada por |
|--------------|------|:---:|---------|----------------|
| `joint_1` | Revoluta | Z | −2.5 a 2.5 rad | Potenciómetro 1 |
| `joint_2` | Revoluta | Y | −2.0 a 2.0 rad | Potenciómetro 2 |
| `joint_gripper` | Prismática | Z | 0 a 0.15 m | Fija en 0 (no se usa) |
| `joint_dedo_izq` | Prismática | −X | 0 a 0.05 m | Potenciómetro 3 |
| `joint_dedo_der` | Prismática | X | 0 a 0.05 m | Potenciómetro 3 |

**Eslabones:**

| Eslabón | Geometría |
|---------|-----------|
| `base_link` | Cilindro, 0.15 m de alto, radio 0.25 m |
| `brazo1_link` | Cilindro, 0.35 m de largo, radio 0.06 m |
| `brazo2_link` | Cilindro, 0.30 m de largo, radio 0.05 m |
| `gripper_base` | Caja de 0.08 × 0.12 × 0.04 m |
| `dedo_izquierdo` / `dedo_derecho` | Cajas de 0.03 × 0.08 × 0.12 m |

Los límites de cada articulación se leen directamente del URDF, así que el código no tiene valores fijos de ángulo.

---

## 4. Hardware y conexiones

### Materiales

- 1 × ESP32 DevKit
- 3 × potenciómetros de 10 kΩ
- Protoboard y cables jumper
- Cable USB con datos

### Conexiones

| Componente | Pin | ESP32 |
|------------|-----|-------|
| Potenciómetro 1 | Extremo 1 · Central · Extremo 2 | 3V3 · **GPIO 34** · GND |
| Potenciómetro 2 | Extremo 1 · Central · Extremo 2 | 3V3 · **GPIO 35** · GND |
| Potenciómetro 3 | Extremo 1 · Central · Extremo 2 | 3V3 · **GPIO 32** · GND |

> Los tres pines están en el ADC1 del ESP32, que funciona correctamente aunque el WiFi esté activo. Los potenciómetros se alimentan con **3.3 V**, nunca con 5 V.

---

## 5. Firmware del ESP32

Archivo: [`Brazo/Brazo.ino`](Brazo/Brazo.ino)

| Parámetro | Valor |
|-----------|-------|
| Velocidad serial | 115200 baudios |
| Periodo de muestreo | 20 ms (50 Hz) |
| Resolución ADC | 12 bits (0 a 4095) |
| Atenuación ADC | 11 dB (rango de aproximadamente 0 a 3.3 V) |
| Filtrado | Promedio de 8 muestras por canal |

**Formato de la trama** (una línea por lectura):

```
t_ms,a1,a2,a3
```

Ejemplo:

```
15320,2048,1990,510
```

Donde `t_ms` es el tiempo en milisegundos desde el arranque y `a1`, `a2`, `a3` son los valores ADC de cada potenciómetro.

### Compilar y subir

1. Instala el **Arduino IDE 2.x** y el paquete de placas *esp32 by Espressif Systems*.
2. Abre `Brazo/Brazo.ino`.
3. Elige *Herramientas → Placa → ESP32 Dev Module* y el puerto COM.
4. Pulsa **Verificar** (✔) y luego **Subir** (→). Si aparece `Connecting.....`, mantén presionado el botón BOOT.
5. Para comprobar, abre el Monitor Serie a 115200 baudios y gira los potenciómetros. Ciérralo después.

---

## 6. Software de control en Python

Archivo: [`control_brazo.py`](control_brazo.py)

### Conversión de ADC a posición

Para las dos articulaciones del brazo, el valor ADC se mapea linealmente al rango del URDF:

```
posición = límite_inf + (ADC / 4095) × (límite_sup − límite_inf)
```

Para la pinza, el potenciómetro 3 define directamente la apertura de los dos dedos:

```
apertura = ADC / 4095          (0 = cerrada, 1 = abierta)
```

### Otros detalles

- Control por posición (`POSITION_CONTROL`) en todas las articulaciones.
- Cada lectura serial va seguida de 5 pasos de simulación.
- Las líneas incompletas o corruptas se descartan sin detener el programa.
- El dedo izquierdo y el derecho reciben la misma apertura para que la pinza se mueva simétricamente.

### Opciones de línea de comandos

| Opción | Descripción | Por defecto |
|--------|-------------|-------------|
| `--port` | Puerto serial del ESP32 | `COM3` |
| `--baud` | Velocidad serial | `115200` |
| `--urdf` | Archivo URDF del robot | `brazo.urdf` |
| `--invertir-pinza` | Invierte el sentido de apertura de la pinza | desactivado |
| `--sim` | Simula el ESP32 con señales senoidales, sin hardware | desactivado |

---

## 7. Instalación y ejecución

### Requisitos

- Python 3.11 o 3.12 (recomendado para que `pybullet` instale sin compilar)
- Arduino IDE 2.x
- Windows, Linux o macOS

### Pasos

**1. Clonar el repositorio y crear el entorno virtual**

```bash
git clone <URL_DEL_REPOSITORIO>
cd <NOMBRE_DEL_REPOSITORIO>
python -m venv venv
```

Activarlo:

```bash
venv\Scripts\activate        # Windows
source venv/bin/activate     # Linux / macOS
```

**2. Instalar dependencias**

```bash
pip install pybullet pyserial
```

**3. Verificar el modelo**

```bash
python main.py
```

Se abre la ventana de PyBullet con el brazo y la consola lista las articulaciones encontradas.

**4. Probar sin hardware**

```bash
python control_brazo.py --sim
```

**5. Ejecutar con el ESP32**

Con el firmware ya cargado y el Monitor Serie cerrado:

```bash
python control_brazo.py --port COM3        # Windows
python control_brazo.py --port /dev/ttyUSB0  # Linux
```

Para detener el programa presiona `Ctrl + C`.

---

## 8. Validación en tiempo real

El programa imprime cada segundo una línea de estado:

```
ADC=(2048,1990, 510)  pinza=0.12  dedo=0.006 m  tasa: 49.8 Hz
```

| Campo | Significado |
|-------|-------------|
| `ADC=(a1,a2,a3)` | Valores crudos de los tres potenciómetros |
| `pinza` | Apertura normalizada (0 a 1) |
| `dedo` | Posición real del dedo izquierdo en metros |
| `tasa` | Frecuencia de actualización efectiva |

Una tasa cercana a **50 Hz** confirma que el sistema funciona al ritmo de muestreo del ESP32, es decir, en tiempo real.

---

## 9. Solución de problemas

| Problema | Causa probable | Solución |
|----------|----------------|----------|
| `could not open port` | Monitor Serie abierto o puerto incorrecto | Cerrar el Monitor Serie y revisar el número COM |
| No aparece el puerto | Falta el driver USB | Instalar el driver CP2102 o CH340, o probar otro cable |
| `Failed to connect` al subir | El ESP32 no entra en modo de carga | Mantener presionado BOOT durante la subida |
| El brazo no se mueve | Firmware sin cargar o cableado incorrecto | Comprobar los datos en el Monitor Serie |
| La pinza abre al revés | Sentido invertido | Ejecutar con `--invertir-pinza` |
| `pybullet` no instala | Versión de Python muy nueva | Usar Python 3.11 o 3.12 |
| Valores ADC inestables | Contacto flojo en la protoboard | Revisar conexiones y GND común |

---

## 10. Estructura del repositorio

```
.
├── Brazo/
│   └── Brazo.ino          # Firmware del ESP32
├── brazo.urdf             # Modelo del robot
├── control_brazo.py       # Control en tiempo real desde el ESP32
├── main.py                # Prueba de carga del URDF
├── .gitignore             # Excluye el entorno virtual
└── README.md
```

---

## Autor

**Carlos** · Universidad Militar Nueva Granada
Asignatura: Sensores y Microcontroladores · Actividad 5
