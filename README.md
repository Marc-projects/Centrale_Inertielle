# CENTRALE INERTIELLE À FUSION DE CAPTEURS (IMU)


**Informations Documentaires**
* **Version :** 1.0.0
* **Cible Matérielle :** Espressif ESP32-WROOM-32E
* **Capteur :** InvenSense MPU6050 (Accéléromètre 3 axes + Gyromètre 3 axes)
* **Frameworks :** ESP-IDF master (v5.x), FreeRTOS, Unity Test Framework, PyQt5

---


# 1. Hardware and Physical Architecture

## 1.1 Interconnection Diagram
Communication between the ESP32 microcontroller and the MPU6050 Inertial Measurement Unit (IMU) is established via a bidirectional synchronous I2C serial bus. The system utilizes the ESP-IDF master peripheral driver subsystem (`driver/i2c_master.h`), which manages resource allocation in a thread-safe manner using internal semaphores.

| MPU6050 Signal | ESP32 Pin (GPIO) | Role and Description |
| :--- | :--- | :--- |
| **VCC** | 5V | Main sensor power supply |
| **GND** | GND | Common reference ground |
| **SCL** | GPIO 22 | Serial Clock (I2C Clock) |
| **SDA** | GPIO 21 | Serial Data (I2C Data) |

## 1.2 I2C Electronic Bus Specifications
* **Clock Frequency ($f_{SCL}$)**: $400\text{ kHz}$ (*Fast Mode*).
* **Hardware Filtering**: Configuration of the glitch suppression register (`glitch_ignore_cnt = 7`) to eliminate switching noise on high-impedance lines.
* **Pull-up Resistors**: Software activation of the ESP32 internal pull-up resistors.
* **Slave Address**: `0x68`.

## 1.3 Register Map and Sensor Data Conditioning

### 1.3.1 Raw Data Frame via I2C Bus
The internal registers of the MPU6050 are refreshed using a sequential burst read of 14 bytes, starting from the base address of the acquisition subsystem: `0x3B`.

```
[0x3B-0x3C] : Accelerometer X
[0x3D-0x3E] : Accelerometer Y
[0x3F-0x40] : Accelerometer Z
[0x41-0x42] : Temperature Sensor
[0x43-0x44] : Gyroscope X
[0x45-0x46] : Gyroscope Y
[0x47-0x48] : Gyroscope Z
```

### 1.3.2 Full-Scale Ranges and Conversion Factors
1. **Accelerometer**: Configured by default to a $\pm 2\text{ g}$ range. Since the Analog-to-Digital Converter (ADC) outputs 16-bit signed integers ($\pm 32768$), the sensor sensitivity is $16384\text{ LSB/g}$.
   $$a_{\text{physical}} = \frac{\text{Value}}{16384.0}$$
2. **Gyroscope**: Configured by default to a $\pm 250^\circ/\text{s}$ range. With a 16-bit signed raw output, the factory scale factor yields $131\text{ LSB}/(^\circ/\text{s})$. To obtain the angular velocity directly in radians per second ($\text{rad/s}$), the application applies a unified conversion constant of $7509.9\text{ LSB}/(\text{rad/s})$:
   $$\omega_{\text{physical}} = \frac{\text{Value}}{7509.9}$$

# 2. Mathematical Modeling and Signal Processing

Attitude estimation relies on data fusion driven by a gradient descent orientation filter (optimized Madgwick algorithm). The orientation of the mobile rigid body frame relative to the earth reference frame is modeled by a unit quaternion:

$$\mathbf{q} = \begin{bmatrix} w & x & y & z \end{bmatrix}^T = \begin{bmatrix} q_0 & q_1 & q_2 & q_3 \end{bmatrix}^T$$

## 2.1 Angular Velocity Integration Equations
The time-dependent orientation kinematics derived purely from gyroscopic data are modeled by the following differential equation:

$$\dot{\mathbf{q}}_{\omega, t} = \frac{1}{2} \mathbf{q}_{t-1} \otimes \mathbf{\omega}_t$$

Where $\otimes$ denotes the non-commutative quaternion product, implemented in the driver under the function `quaternion_product`. The pure angular velocity quaternion $\mathbf{\omega}_t$ is defined as:

$$\mathbf{\omega}_t = \begin{bmatrix} 0 & \omega_x & \omega_y & \omega_z \end{bmatrix}^T$$

Discrete numerical integration over the sampling period $\Delta t$ yields:

$$\mathbf{q}_{\omega, t} = \mathbf{q}_{t-1} + \dot{\mathbf{q}}_{\omega, t} \cdot \Delta t$$

## 2.2 Gradient Descent Correction (Gravity Alignment)
Because gyroscopes suffer from slow runtime drift (bias), it is necessary to counteract this divergence by using the accelerometer as an absolute reference of the gravity vector $\mathbf{g} = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix}^T$ in the fixed earth frame.

The goal is to minimize the objective error function $\mathbf{f}(\mathbf{q}, \hat{\mathbf{a}})$, which measures the discrepancy between the projected theoretical gravity vector and the actual normalized accelerometer measurement $\hat{\mathbf{a}} = \begin{bmatrix} a_x & a_y & a_z \end{bmatrix}^T$:

$$\mathbf{f}(\mathbf{q}, \hat{\mathbf{a}}) = \begin{bmatrix}
2(q_1q_3 - q_0q_2) - a_x \\
2(q_0q_1 + q_2q_3) - a_y \\
1 - 2(q_1^2 + q_2^2) - a_z
\end{bmatrix} = \begin{bmatrix} f_x \\ f_y \\ f_z \end{bmatrix}$$

The error space gradient is computed by multiplying the transposed Jacobian matrix $\mathbf{J}^T(\mathbf{q})$ by the error function $\mathbf{f}(\mathbf{q}, \hat{\mathbf{a}})$:

$$\nabla \mathbf{f} = \mathbf{J}^T(\mathbf{q}) \mathbf{f}(\mathbf{q}, \hat{\mathbf{a}})$$

Where the Jacobian matrix $\mathbf{J}(\mathbf{q})$ is explicitly formulated through partial differentiation:

$$\mathbf{J}(\mathbf{q}) = \begin{bmatrix}
-2q_2 & 2q_3 & -2q_0 & 2q_1 \\
2q_1 & 2q_0 & 2q_3 & 2q_2 \\
0 & -4q_1 & -4q_2 & 0
\end{bmatrix}$$

The complete analytical expansion of this system dictates the code executed within `compute_gradient_descent_correction`:

$$\nabla \mathbf{f} = \begin{bmatrix}
-2q_2 \cdot f_x + 2q_1 \cdot f_y \\
2q_3 \cdot f_x + 2q_0 \cdot f_y - 4q_1 \cdot f_z \\
-2q_0 \cdot f_x + 2q_3 \cdot f_y - 4q_2 \cdot f_z \\
2q_1 \cdot f_x + 2q_2 \cdot f_y
\end{bmatrix}$$

## 2.3 Global Fusion Algorithm
To prevent anomalous corrections caused by dynamic non-gravitational accelerations (vibrations, rapid translations), the gradient descent correction is only executed if the acceleration vector magnitude is close to Earth's gravity ($1\text{ g}$), bounded by an acceptance margin $\epsilon_{\text{margin}} = 0.2\text{ g}$:

$$1.0 - \epsilon_{\text{margin}} \le \|\mathbf{a}\| \le 1.0 + \epsilon_{\text{margin}}$$

If this condition holds true, the gradient is scaled by the descent step size $\mu = 0.041$ (`GRADIENT_DESCENT_STEP`) and combined with the gyroscopic dynamics:

$$\dot{\mathbf{q}}_{\text{fusion}} = \dot{\mathbf{q}}_{\omega, t} - \mu \frac{\nabla \mathbf{f}}{\|\nabla \mathbf{f}\|}$$

The final attitude quaternion is then updated, followed by strict normalization to preserve its unit geometric properties ($\|\mathbf{q}\| = 1$):

$$\mathbf{q}_{t} = \mathbf{q}_{t-1} + \dot{\mathbf{q}}_{\text{fusion}} \cdot \Delta t$$
$$\mathbf{q}_{\text{final}} = \frac{\mathbf{q}_{t}}{\|\mathbf{q}_{t}\|}$$

## 2.4 Geometric Conversion to Axis-Angle Representation
For downstream data export and real-time graphic processing, the orientation represented by the quaternion $\mathbf{q} = \begin{bmatrix} q_0 & q_1 & q_2 & q_3 \end{bmatrix}^T$ is converted into Axis-Angle space $(\theta, v_1, v_2, v_3)$:

$$\theta = 2 \cdot \text{atan2}\left(\sqrt{q_1^2 + q_2^2 + q_3^2}, q_0\right)$$
$$\begin{bmatrix} v_1 \\ v_2 \\ v_3 \end{bmatrix} = \frac{1}{\sqrt{q_1^2 + q_2^2 + q_3^2}} \begin{bmatrix} q_1 \\ q_2 \\ q_3 \end{bmatrix}$$

# 3. Embedded Software Architecture

The firmware is developed on top of the **FreeRTOS** real-time operating system native to the **ESP-IDF** environment. Implementing a preemptive architecture guarantees strict temporal determinism required for real-time signal fusion computations.
```mermaid
 [ Hardware I2C Bus (MPU6050) ]
                     │
                     ▼ (Hardware sampling clocked at 500 Hz)
 ┌─────────────────────────────────────────────────────────┐
 │ Task 1 : task_collect_compute   [Priority : 23 (High)]  │
 ├─────────────────────────────────────────────────────────┤
 │  - 14-byte sequential I2C burst read                    │
 │  - Calibration bias subtraction                         │
 │  - Gradient descent data fusion execution (at 100 Hz)   │
 │  - Asynchronous queue notification & overwrite          │
 └───────────────────────────┬─────────────────────────────┘
                             │
                             │ xQueueOverwrite (Overwrites if unread)
                             ▼
                    ┌─────────────────┐
                    │   data_queue    │ (Size: 1 element, Quaternion)
                    └─────────────────┘
                             │
                             │ xQueuePeek (Read without destruction)
                             ▼
 ┌─────────────────────────────────────────────────────────┐
 │ Task 2 : task_format_send       [Priority : 22 (Low)]   │
 ├─────────────────────────────────────────────────────────┤
 │  - Periodically clocked at 60 Hz (Refresh rate)         │
 │  - Geometric conversion: Quaternion ──► Axis-Angle      │
 │  - Raw textual CSV serialization over UART serial port  │
 └─────────────────────────────────────────────────────────┘
```


## 3.1 Task Topology and Scheduling
The application instantiates two concurrent high-priority computing tasks:

1. **`task_collect_compute` (Priority 23 - Real-Time Critical)**:
   * **Periodicity**: Strictly clocked every $2\text{ ms}$ ($\Delta t = 2\text{ ms}$, resulting in a sampling frequency $f_s = 500\text{ Hz}$) utilizing the deterministic blocking function `vTaskDelayUntil`.
   * **Role**: Handles I2C data extraction, instrumental bias subtraction, mathematical fusion model execution, and system state updates.
2. **`task_format_send` (Priority 22 - Outbound Stream)**:
   * **Periodicity**: Clocked at $16.67\text{ ms}$ ($60\text{ Hz}$, matching standard video monitor refresh rates).
   * **Role**: Evaluates the latest estimated attitude, executes geometric transformations, and transmits telemetry across the UART serial communication bus at 115200 baud.

## 3.2 Memory Management and Inter-Task Communication
Data exchange between the high-speed computing task ($500\text{ Hz}$) and the telemetry transmission task ($60\text{ Hz}$) is coupled through a thread-safe FreeRTOS queue named `data_queue`, sized to hold exactly one single `quaternion` structure ($16\text{ bytes}$).

* **Write Mechanism (`xQueueOverwrite`)**: The critical computation task must never stall due to down-stream transmission latency. `xQueueOverwrite` immediately replaces the old entry in the queue. This ensures the queue perpetually holds the freshest attitude state, guaranteeing zero-latency tracking.
* **Read Mechanism (`xQueuePeek`)**: The transmission task uses `xQueuePeek` to inspect data without removing it from the queue. This permits the formatting task to operate independently on the latest valid frame without destroying the data stream pipeline.

# 4. Testing Strategy and Continuous Integration (CI)

To meet rigorous industrial reliability standards, the project features automated unit testing utilizing the **Unity** testing framework. These test suites run locally on the host machine (`platform = native`) or on-target to validate the mathematical core library completely isolated from hardware components.

## 4.1 Unit Test Coverage (`test_main.c`)
The software validation suite is divided into four main categories:

1. **Linear Algebraic Operations**: Validation of baseline geometric primitives: `quaternion_addition`, `quaternion_subtraction`, and `quaternion_scalar_product`.
2. **Spatial Composition (Hamilton Product)**:
   * *Identity element test*: Verifies that multiplication by the identity quaternion $\mathbf{q}_{\text{id}} = \begin{bmatrix} 1 & 0 & 0 & 0 \end{bmatrix}^T$ does not alter spatial orientation.
   * *Rotation composition test*: Sequences a pure $90^\circ$ rotation about the $Z$-axis followed by a $90^\circ$ rotation about the $Y$-axis, evaluated against a strict numerical tolerance ($\le 10^{-4}$).
3. **Numerical Robustness and Boundary Cases**:
   * Validation of Euclidean norm evaluations.
   * *Anti-division-by-zero protection*: Validates the `quaternion_normalize` function. If the input vector exhibits a zero or near-zero norm ($\le 10^{-6}$), the algorithm catches the exception and safe-fails the quaternion back to the identity state $\begin{bmatrix} 1 & 0 & 0 & 0 \end{bmatrix}^T$, preventing runtime hardware crashes caused by `NaN` (Not a Number) propagating through registers.
4. **Specific Algorithmic Validation**:
   * *Perfect Alignment*: Verifies that the error gradient evaluates to exactly zero when the sensor is stationary and aligned perfectly with the theoretical model orientation.
   * *Dynamic Increment*: Verifies that a non-zero, directional correction vector is properly generated whenever the estimated quaternion deviates from the gravitational reference read by the accelerometer.

# 5. Graphical Interface and Real-Time Visualization

The ground station visualization ecosystem consists of a Python application built with the **PyQt5** high-performance UI framework coupled with the hardware-accelerated 3D rendering library **PyQtGraph.opengl**.

## 5.1 Graphical Pipeline Architecture and Metrics
1. **Serial Stream Connection (PySerial)**: Establishes a point-to-point asynchronous serial connection over port `COM3` configured at a speed of $115200\text{ baud}$. The polling timeout is set to an ultra-low threshold ($1\text{ ms}$) to prevent blocking the main PyQt UI thread.
2. **Buffer Flush Mechanism (Anti-Lag)**: Driven by a `QTimer` firing at $60\text{ Hz}$, the serial processing loop entirely flushes the operating system's serial buffer (`while ser.in_waiting:`). Only the final, complete telemetry line is decoded. This approach completely prevents visual lag or phase delays caused by historical data piling up in the OS buffer.
3. **Parsing and Transformation Application**: The inbound CSV text frame (`angle,vx,vy,vz`) is tokenized and parsed into floats. The OpenGL scene clears the current matrix state of the 3D asset via `resetTransform()` and re-applies the updated spatial rotation converted into degrees:
   $$\theta_{\text{degrees}} = \theta_{\text{radians}} \cdot \frac{180}{\pi}$$

# 6. Deployment and Operations Guide

## 6.1 Final PlatformIO Project Structure

```text
.
├── 3d_display.py # Ground station 3D OpenGL UI application
├── platformio.ini # PlatformIO multi-environment configuration file
├── include/
│   ├── collect_compute.h # Data acquisition prototypes & timing constants
│   ├── format_send.h # UART telemetry formatting & transmission prototypes
│   ├── i2c.h # Physical I2C peripheral configurations
│   ├── main.h # Global types declarations & FreeRTOS handles
│   └── mpu_6050.h # Factory sensor register maps & addresses
├── lib/
│   └── quaternion/
│       ├── quaternion.c # Mathematical core & Madgwick algorithm files
│       └── quaternion.h # Quaternion & axis-angle geometric structures
├── src/
│   ├── collect_compute.c # 500 Hz data fusion execution code
│   ├── format_send.c # 60 Hz ground station telemetry code
│   ├── i2c.c # ESP-IDF v5 Master I2C bus allocation
│   ├── main.c # OS Entry point (app_main) & task initializations
│   └── mpu6050.c # Sensor hardware initialization sequence
└── test/
    └── test_quaternion/
        └── test_main.c # Unity Framework unit testing suite
```


## 6.2 Multi-Target Environment Configuration (`platformio.ini`)
The PlatformIO configuration file defines two isolated target environments. The first targets cross-compilation for the physical embedded ESP32 microcontroller, while the second targets native compilation on the host development machine for instant test-suite execution.

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = espidf
monitor_speed = 115200

[env:native]
platform = native

```

## 6.3 Automatic Sensor Bias Calibration Procedure

Upon system startup, calling the `init_bias` routine forces the IMU into an automated stationary auto-calibration sequence lasting $20\text{ cycles}$ (`NBR_INIT_CYCLE`), equivalent to a $40\text{ ms}$ stabilized duration.

-   **Operational Assumption**: The system must remain perfectly motionless and level relative to the local horizon for the entire duration of this phase.
    
-   **Compensation Algorithm**: The device samples background noise vectors. For the vertical axis of the accelerometer ($Z$), the firmware dynamically subtracts the theoretical $1\text{ g}$ gravity vector component before accumulating the bias offset:
    

$$\text{Data}_{\text{corrected}, Z} = \text{Data}_{\text{raw}, Z} - 1.0\text{ f}$$

-   **Model Application**: The computed mean bias for each axis is cached in volatile RAM. During steady-state operations, every incoming raw physical sample undergoes systematic bias vector subtraction before being passed into the data fusion algorithm:
    

$$X_{\text{fusion}} = X_{\text{measured}} - \text{Bias}_{X}$$