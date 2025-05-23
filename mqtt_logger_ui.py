import sys
import json
from collections import deque
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QGridLayout
)
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import paho.mqtt.client as mqtt

# Enable anti-aliasing and OpenGL for smooth rendering
pg.setConfigOptions(antialias=True, useOpenGL=True)

class MqttSensorUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MQTT IMU Sensor Oscilloscope")

        # MQTT client setup
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect("broker.hivemq.com", 1883, 60)
        self.client.loop_start()

        # UI Controls
        self.connect_button = QPushButton("Disconnect")
        self.connect_button.clicked.connect(self.toggle_mqtt)
        self.labels = {"TEMP": QLabel("Temperature: 0°C")}

        # Layout for connection button and temperature
        label_layout = QHBoxLayout()
        label_layout.addWidget(self.connect_button)
        label_layout.addStretch()
        label_layout.addWidget(self.labels["TEMP"])

        # Labels for showing numeric sensor data
        self.sensor_types = {
            "ACCEL": ["X", "Y", "Z"],
            "GYRO": ["X", "Y", "Z"],
            "ANGLE": ["Roll", "Pitch", "Yaw"]
        }

        self.sensor_value_labels = {}
        sensor_data_layout = QGridLayout()
        row = 0
        for sensor, axes in self.sensor_types.items():
            self.sensor_value_labels[sensor] = {}
            sensor_data_layout.addWidget(QLabel(f"<b>{sensor}</b>"), row, 0)
            for col, axis in enumerate(axes, start=1):
                label = QLabel(f"{axis}: 0.00")
                label.setStyleSheet("font-family: monospace;")
                sensor_data_layout.addWidget(label, row, col)
                self.sensor_value_labels[sensor][axis] = label
            row += 1

        self.max_points = 200
        self.data_buffers = {}
        self.curves = {}
        self.plot_widgets = {}

        # Oscilloscope-style plot layout
        plot_layout = QVBoxLayout()
        for sensor, axes in self.sensor_types.items():
            plot = pg.PlotWidget(title=f"{sensor} Data")
            plot.setBackground((30, 30, 30))  # Dark oscilloscope background
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setLabel("left", sensor, color='w', size='10pt')
            plot.setLabel("bottom", "Samples", color='w', size='10pt')
            plot.addLegend(labelTextColor='w')

            if sensor == "ANGLE":
                plot.setYRange(-180, 180)
            elif sensor == "GYRO":
                plot.setYRange(-500, 500)
            else:
                plot.setYRange(-20, 20)

            self.plot_widgets[sensor] = plot
            self.data_buffers[sensor] = {}
            self.curves[sensor] = {}

            axis_colors = {
                "X": (255, 0, 0),
                "Y": (0, 255, 0),
                "Z": (0, 128, 255),
                "Roll": (255, 0, 0),
                "Pitch": (0, 255, 0),
                "Yaw": (0, 128, 255)
            }

            for axis in axes:
                self.data_buffers[sensor][axis] = deque([0] * self.max_points, maxlen=self.max_points)
                pen = pg.mkPen(color=axis_colors[axis], width=2)
                curve = plot.plot(pen=pen, name=axis)
                self.curves[sensor][axis] = curve

            plot_layout.addWidget(plot)

        # Set main layout
        main_layout = QVBoxLayout()
        main_layout.addLayout(label_layout)
        main_layout.addLayout(sensor_data_layout)
        main_layout.addLayout(plot_layout)

        self.setLayout(main_layout)

        # Timer for plot update and GUI updates (runs in main thread)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)

        # Latest sensor data
        self.latest_data = {
            "ACCEL": [0, 0, 0],
            "GYRO": [0, 0, 0],
            "ANGLE": [0, 0, 0],
            "TEMP": 0
        }

    def toggle_mqtt(self):
        if self.client.is_connected():
            self.client.disconnect()
            self.connect_button.setText("Connect")
        else:
            self.client.reconnect()
            self.connect_button.setText("Disconnect")

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code", rc)
        client.subscribe("device")  # Adjust the topic as needed

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            accel = payload.get("accel", {})
            gyro = payload.get("gyro", {})
            angle = payload.get("angle", {})
            temp = payload.get("temp", 0)

            self.latest_data["ACCEL"] = [
                accel.get("x", 0),
                accel.get("y", 0),
                accel.get("z", 0)
            ]
            self.latest_data["GYRO"] = [
                gyro.get("x", 0),
                gyro.get("y", 0),
                gyro.get("z", 0)
            ]
            self.latest_data["ANGLE"] = [
                angle.get("roll", 0),
                angle.get("pitch", 0),
                angle.get("yaw", 0)
            ]
            self.latest_data["TEMP"] = temp

        except Exception as e:
            print("Error parsing MQTT message:", e)

    def update_sensor_value_labels(self):
        """Update numeric sensor data labels on UI."""
        for sensor, axes in self.sensor_types.items():
            for i, axis in enumerate(axes):
                val = self.latest_data[sensor][i]
                self.sensor_value_labels[sensor][axis].setText(f"{axis}: {val:.2f}")

        # Update temperature label
        self.labels["TEMP"].setText(f"Temperature: {self.latest_data['TEMP']:.2f} °C")

    def update_plots(self):
        # Update sensor numeric labels
        self.update_sensor_value_labels()

        # Update oscilloscope plots with latest data
        for sensor in self.sensor_types:
            axes = self.sensor_types[sensor]
            for i, axis in enumerate(axes):
                self.data_buffers[sensor][axis].append(self.latest_data[sensor][i])
                self.curves[sensor][axis].setData(list(self.data_buffers[sensor][axis]), connect='all')

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MqttSensorUI()
    window.resize(1000, 800)
    window.show()
    sys.exit(app.exec_())
