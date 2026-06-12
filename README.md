# Amity Cerebro

Este directorio contiene el código fuente principal (el "Cerebro") del robot AMITY. Está desarrollado en C++ utilizando el framework **ESP-IDF** de Espressif para el microcontrolador ESP32.

## ¿En qué consiste?

El proyecto implementa un sistema de control de locomoción híbrido y autónomo basado en FreeRTOS. Sus características principales son:

- **Sistema de Control Híbrido**: Permite cambiar en tiempo real entre dos modos de locomoción:
  - **Red Neuronal (MODE_NN)**: Ejecuta una inferencia bare-metal de un modelo MLP pre-entrenado (Observación de 20 variables -> 2 capas ocultas de 64 neuronas -> 4 acciones).
  - **Control PID Clásico (MODE_PID)**: Utiliza un oscilador de patrón central (CPG) ajustado por la retroalimentación de la IMU mediante un controlador PID para mantener el equilibrio.
- **Gestión de Hardware**: Abstracción para el manejo de los Servomotores (PCA9685 por I2C), la unidad de medición inercial (IMU MPU6050/MPU9250), sensores táctiles y LEDs de estado.
- **Eficiencia Energética**: Lógica de Deep Sleep y Wake-up controlada mediante sensores táctiles capacitivos para ahorrar batería cuando el robot no está en uso.

## ¿Cómo usar este directorio?

### Requisitos Previos
- **ESP-IDF** (versión compatible con ESP32).
- Entorno configurado (variables de entorno cargadas con `export.sh` o `export.bat`).

### Compilación y Carga
Para compilar y cargar el firmware al ESP32, asegúrate de estar en este directorio (`Amity_Cerebro`) y ejecuta:

```bash
idf.py build
idf.py flash monitor
```

### Interacción con el Robot
El control físico se realiza a través de los sensores táctiles (Touch):
- **Sensor 0**: Alterna entre iniciar (REANUDAR) y detener (PAUSAR) la caminata.
- **Sensor 1**: Cambia el modo de control entre Red Neuronal (`MODE_NN`) y PID Clásico (`MODE_PID`).
- **Sensor 3 (Mantener pulsado)**: Pone al robot en modo reposo (Deep Sleep). Mantener pulsado nuevamente lo despierta.

---

## Pasos para cargar una nueva política aprendida en Isaac Lab

El robot utiliza una red neuronal (MLP) entrenada en un simulador (NVIDIA Isaac Lab). Para actualizar la "inteligencia" del robot con una nueva política, debes seguir estos pasos:

### 1. Exportar los Pesos de Isaac Lab
Una vez finalizado el entrenamiento en Isaac Lab, debes extraer los parámetros de la red. Necesitarás un script en Python que lea el checkpoint del modelo y genere el código en C/C++ correspondiente a:
- **Pesos y Sesgos (Weights & Biases)**: `w1`, `b1`, `w2`, `b2`, `w3`, `b3` de la estructura MLP (tamaños 20->64, 64->64, 64->4).
- **Parámetros de Normalización**: Las medias (`obs_mean`) y desviaciones estándar (`obs_std` u `obs_var`) de las observaciones con las que fue entrenada la política.

### 2. Modificar el archivo `amity_policy.h`
Dirígete a la ruta `main/amity_policy.h` dentro de este directorio. Este archivo contiene los arrays estáticos de la red neuronal.
- Copia el código C/C++ generado en el paso anterior.
- Reemplaza el contenido de `amity_policy.h` cuidando de mantener el mismo formato y nombres de variables (`obs_mean`, `obs_std`, `w1`, `w2`, etc.).

### 3. Recompilar y Flashear
Una vez guardado el archivo `amity_policy.h` actualizado, vuelve a compilar el proyecto y cárgalo al ESP32:

```bash
idf.py build
idf.py flash
```

El robot ahora despertará utilizando la nueva política que entrenaste en Isaac Lab.
