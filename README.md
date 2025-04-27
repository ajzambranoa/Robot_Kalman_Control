# Proyecto de Estimación de Estados con Filtro de Kalman para Robot Elegoo Tumbller

Este repositorio contiene el proyecto de implementación de un sistema de estimación de estados utilizando el Filtro de Kalman para el robot **Elegoo Tumbller**. El objetivo es mejorar el control del robot mediante la estimación precisa de su ángulo y velocidad, integrando el modelo dinámico del robot con un filtro de Kalman.

## Contenido

1. **Identificación del Modelo Dinámico**
   - Archivos de MATLAB para la identificación del modelo dinámico del robot Elegoo Tumbller.
   - Parametrización de los parámetros físicos del robot y su validación.

2. **Parámetros del Robot**
   - Variables como la masa, longitud, radio y momentos de inercia para el modelo.

3. **Simulación del Filtro de Kalman**
   - Simulación y comparación de estimaciones de ángulo, velocidad y posición utilizando Filtro de Kalman.
   - Generación de gráficos de comparación entre estimaciones y mediciones reales.

## Requisitos

- **MATLAB/Simulink**: Para ejecutar la simulación del modelo dinámico, estimaciones y generar gráficos.
- **Arduino IDE**: Para cargar el código de control al robot Elegoo Tumbller (código base para el robot).
- **Sensor de Ángulo y Velocidad**: Los datos del sensor son proporcionados desde el Arduino, incluyendo ángulo, velocidad angular, posición y velocidad lineal.
- **Código de Arduino**: El código original para el control del robot ya está disponible y se encuentra en la carpeta `Arduino_Code`.

## Descripción del Proyecto

El robot **Elegoo Tumbller** es un robot de balanceo que se encuentra en posición vertical en todo momento. El proyecto busca mejorar la capacidad de control de este robot al incorporar un sistema de estimación de estados que considere tanto las mediciones directas del robot como estimaciones del ángulo y otros estados no medidos directamente (como el giro).

### Filtro de Kalman

El Filtro de Kalman se implementa para estimar los estados faltantes del robot, específicamente el giro, basándose en las mediciones disponibles (ángulo, velocidad angular, posición y velocidad lineal). El objetivo es mejorar la estabilidad del robot, incluso cuando hay ruidos o errores en los sensores.

### Control Adaptativo

A lo largo del proyecto se integrará un control adaptativo que ajusta los parámetros del sistema dependiendo de la variabilidad de las condiciones operativas del robot.

## Estructura del Repositorio

- **Arduino_Code/**: Contiene el código de Arduino necesario para operar el robot Elegoo Tumbller.
- **Matlab_Code/**: Archivos MATLAB para la identificación del modelo dinámico y simulaciones con el Filtro de Kalman.
- **Experimental_Data/**: Datos experimentales utilizados para validar el modelo.
- **README.md**: Este archivo.

## Pasos para Ejecutar el Proyecto

### 1. Ejecutar la Identificación del Modelo

1. Abre los archivos MATLAB en la carpeta `Matlab_Code/`.
2. Ejecuta el código para la identificación del modelo dinámico.
3. Asegúrate de que el modelo se valide correctamente utilizando los parámetros que has identificado.

### 2. Implementar Filtro de Kalman

1. Implementa el Filtro de Kalman utilizando los datos obtenidos del Arduino (ángulo, velocidad angular, posición y velocidad lineal).
2. Compara las estimaciones del Filtro de Kalman con las mediciones reales del robot.

### 3. Control del Robot en Arduino

1. Abre el código de Arduino en la carpeta `Arduino_Code/`.
2. Conecta el robot Elegoo Tumbller a tu PC y carga el código base.
3. Realiza las pruebas con el código base para verificar que el robot mantenga el ángulo 0°.

### 4. Visualización de Resultados

1. Abre los gráficos generados por MATLAB para ver las comparaciones entre las estimaciones y las mediciones reales.
2. Ajusta los parámetros de control según sea necesario para mejorar la estabilidad y el rendimiento del robot.

## Contribuciones

Este proyecto está abierto para contribuciones. Si tienes alguna mejora, sugerencia o corrección, no dudes en abrir un *issue* o hacer un *pull request*.

## Licencia

Este proyecto no tiene una licencia explícita, pero puedes usarlo y modificarlo bajo tu propio riesgo. El código de Arduino es el proporcionado por Elegoo para el robot Elegoo Tumbller.
