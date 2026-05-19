import sys
import numpy as np

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

import pyqtgraph.opengl as gl
from pyqtgraph.opengl import MeshData

try:
    import serial

    ser = serial.Serial(
        port='COM3',
        baudrate=115200,
        timeout=0.001
    )

    SERIAL_ENABLED = True
    print("ESP32 connecté")

except:
    print("Aucun port série -> mode simulation")
    sys.exit(1)

app = QApplication(sys.argv)

view = gl.GLViewWidget()
view.show()
view.setWindowTitle("IMU Viewer")
view.setCameraPosition(azimuth=250, distance=6)

grid = gl.GLGridItem()
view.addItem(grid)

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

def update():

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


    imu.resetTransform()
    imu.rotate(angle * 180 / np.pi, vx, vy, vz)

timer = QTimer()

timer.timeout.connect(update)

timer.start(int(1000/60))

sys.exit(app.exec_())