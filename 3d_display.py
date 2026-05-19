import sys
import math
import numpy as np

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

import pyqtgraph.opengl as gl
from pyqtgraph.opengl import MeshData

# =========================
# SERIAL
# =========================

SERIAL_ENABLED = False

try:
    import serial

    ser = serial.Serial(
        port='COM3',      # modifier si besoin
        baudrate=115200,
        timeout=0.001
    )

    SERIAL_ENABLED = True
    print("ESP32 connecté")

except:
    print("Aucun port série -> mode simulation")


# =========================
# APP QT
# =========================

app = QApplication(sys.argv)

view = gl.GLViewWidget()
view.show()
view.setWindowTitle("IMU Viewer")
view.setCameraPosition(distance=6)
view.setCameraPosition(azimuth=250, distance=6)

grid = gl.GLGridItem()
view.addItem(grid)


# =========================
# OBJET 3D
# =========================

def create_box():

    vertices = np.array([
    [-1, -1, -0.1],
    [ 1, -1, -0.1],
    [ 1,  1, -0.1],
    [-1,  1, -0.1],

    [-1, -1,  0.1],
    [ 1, -1,  0.1],
    [ 1,  1,  0.1],
    [-1,  1,  0.1],
    ])

    faces = np.array([
    [0,1,2],[0,2,3],   # face bas
    [4,5,6],[4,6,7],   # face haut

    [0,1,5],[0,5,4],   # côtés
    [1,2,6],[1,6,5],
    [2,3,7],[2,7,6],
    [3,0,4],[3,4,7]
    ])

    return MeshData(vertexes=vertices, faces=faces)


mesh = create_box()

imu = gl.GLMeshItem(
    meshdata=mesh,
    smooth=False,
    shader='shaded',
    drawEdges=True
)

view.addItem(imu)


# =========================
# SIMULATION
# =========================

t = 0


def simulated_data():
    global t

    angle = 45 * math.sin(t)

    vx = math.cos(t)
    vy = math.sin(t)
    vz = 0.5

    norm = math.sqrt(vx*vx + vy*vy + vz*vz)

    vx /= norm
    vy /= norm
    vz /= norm

    t += 0.05

    return angle, vx, vy, vz


# =========================
# UPDATE
# =========================

"""def update():

    # =====================
    # LECTURE SERIAL
    # =====================

    if SERIAL_ENABLED:

        line = ser.readline().decode().strip()
        if not line:
            return

        error = True
        while error:
            try:
                angle, vx, vy, vz = map(float, line.split(','))
                error = False
            except:
                return

      

    # =====================
    # SIMULATION
    # =====================

    else:

        angle, vx, vy, vz = simulated_data()

    # =====================
    # APPLICATION ROTATION
    # =====================

    imu.resetTransform()

    imu.rotate(angle * 180 / math.pi, vx, vy, vz)"""

def update():

    if SERIAL_ENABLED:

        latest = None

        while ser.in_waiting:
            try:
                latest = ser.readline().decode().strip()
            except:
                return

        if latest is None:
            return

        try:
            angle, vx, vy, vz = map(float, latest.split(','))
        except:
            return

    else:
        angle, vx, vy, vz = simulated_data()

    imu.resetTransform()
    imu.rotate(math.degrees(angle), vx, vy, vz)

# =========================
# TIMER
# =========================

timer = QTimer()

timer.timeout.connect(update)

# 60 Hz
timer.start(int(1000/60))

sys.exit(app.exec_())