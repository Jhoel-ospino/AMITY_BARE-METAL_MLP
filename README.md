# 🤖 AMITY: Bare-Metal MLP Sim-to-Real Transfer

![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.0+-red.svg)
![Hardware](https://img.shields.io/badge/Hardware-XIAO_ESP32S3_SENSE-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

Implementación **Bare-Metal** de una política de Aprendizaje por Refuerzo Profundo (DRL) entrenada en el entorno **NVIDIA Isaac Lab** e inferida directamente en un microcontrolador **XIAO ESP32-S3 SENSE** para locomoción bípeda.

> [!WARNING]
> **Aviso sobre Hardware y CAD:** Este repositorio libera de manera abierta *exclusivamente* el código fuente del firmware y los scripts para la transferencia y ejecución bare-metal del modelo neuronal. **Los archivos de diseño mecánico 3D (CAD/STL) son de carácter privativo y NO están incluidos.**

## 🧠 Amity Cerebro

Este directorio contiene el firmware del robot AMITY, desarrollado en **C++** sobre el framework **ESP-IDF** (basado en FreeRTOS). El sistema orquesta de forma autónoma la locomoción del robot sin depender de interpretadores como TensorFlow Lite, garantizando máxima velocidad, mínima huella de memoria y alto determinismo temporal.

### Arquitectura de Control Híbrida

El sistema permite alternar dinámicamente entre dos modos de locomoción mediante los sensores táctiles del ESP32:

| Modo de Control | Descripción | Demostración |
|:---:|:---|:---:|
| **CPG / PID Clásico** | Oscilador de Patrón Central (CPG) acoplado a un lazo PID clásico utilizando retroalimentación de la IMU. | <img src="source/IMG_4717%20(1).gif" width="220" /> |
| **MLP Bare-Metal** | Inferencia matricial nativa de una política neuronal Actor. Recibe 20 observaciones cinemáticas y genera 4 acciones. | <img src="source/IMG_4718%20(1).gif" width="220" /> |

## 📊 Métricas de Desempeño y Validación

La aproximación DRL Bare-Metal ha sido sometida a pruebas experimentales rigurosas en la plataforma física, obteniendo una estabilización superior respecto al control analítico:

* **Latencia de Inferencia MLP:** $\approx 180 \ \mu s$.
* **Latencia de Ciclo de Control (I2C + Inferencia + PWM):** $3.4 \text{ ms media}$ (cumple holgadamente la ventana de $33.3 \text{ ms}$ a $30 \text{ Hz}$).
* **Consumo de Memoria Flash:** $257 \text{ KB}$.
* **RAM (Heap) Libre:** $\approx 374 \text{ KB}$, confirmando un perfil libre de fugas de memoria.

### Comparativa CPG vs MLP (Resultados In-Vivo)

| Métrica | CPG / PID | MLP Bare-Metal |
|:---|:---:|:---:|
| **Varianza de Balanceo (*Roll*)** | $0.0088 \text{ rad}^2$ | $0.0069 \text{ rad}^2$ |
| **Varianza de Cabeceo (*Pitch*)** | $0.0033 \text{ rad}^2$ | $0.0031 \text{ rad}^2$ |
| **Amplitud Oscilación (*Yaw*)** | $\pm 8.4^\circ$ | $\pm 4.1^\circ$ (Mejora del 51%) |
| **Deriva Direccional (*Yaw*)** | $-3.2^\circ/\text{s}$ | $-0.8^\circ/\text{s}$ |

*Nota sobre Guiñada (Yaw): Dado que el sensor MPU6050 utilizado carece de magnetómetro, el robot sufre de deriva natural por acumulación de error del giroscopio. A pesar de esta limitación de hardware, la política MLP reduce significativamente la amplitud de oscilación durante el avance.*

---

## 🛠 Instalación y Despliegue

### Requisitos Previos
* **ESP-IDF** (versión compatible con ESP32-S3).
* Entorno virtual de ESP-IDF inicializado (variables de entorno cargadas con `. export.sh` o `export.bat`).

### Compilación y Ejecución
Navega a la carpeta principal (`Amity_Cerebro` o directorio raíz) y ejecuta:

```bash
idf.py build
idf.py flash monitor
```

## 🎮 Interacción (Sensores Touch)

El ESP32 emplea sensores táctiles capacitivos para operar la máquina de estados:
* **Sensor 0**: Alterna entre Iniciar (Reanudar) y Detener (Pausar) la marcha.
* **Sensor 1**: Conmuta el modo de control (`MODE_NN` $\leftrightarrow$ `MODE_PID`).
* **Sensor 3 (Mantener pulsado)**: Pone al sistema en letargo de bajo consumo (Deep Sleep). Una nueva pulsación sostenida lo despierta.

## 🚀 Guía de Transferencia Sim-to-Real (Isaac Lab)

Para actualizar el comportamiento del robot con una nueva política:

1. **Exportación de Pesos:** Tras concluir el entrenamiento en **Nvidia Isaac Lab**, emplea los *scripts* de la carpeta `source/` para extraer desde el `.pt` los pesos, sesgos y parámetros estadísticos de normalización (medias y varianzas).
2. **Actualización de Matrices C++:** El script te generará código C/C++. Cópialo y pégalo en el archivo `main/amity_policy.h`, sustituyendo los valores de `obs_mean`, `obs_var`, `w1`, `b1`, etc.
3. **Flasheo:** Ejecuta `idf.py build flash`. Al reiniciarse, el ESP32 instanciará la nueva red neuronal en *bare-metal*, logrando el traspaso *Zero-Shot* al mundo físico.
