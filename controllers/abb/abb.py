import sys, subprocess, math, ikpy, tempfile
from ikpy.chain import Chain

from controller import Supervisor

from PySide6 import QtWidgets
from ui.chat_window_ui import Ui_MainWindow

IKPY_MAX_ITERATIONS = 4

class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, gui):
        super(MainWindow, self).__init__()
        self.setupUi(self)

        self.gui = gui

        self.send_button.clicked.connect(self.send_demo)
        self.clear_button.clicked.connect(self.clear)

        self._init_env()

    def _init_env(self):
        # Initialize the Webots Supervisor.
        self.supervisor = Supervisor()
        self.timeStep = int(4 * self.supervisor.getBasicTimeStep())

        # Create the arm chain from the URDF
        filename = None
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            filename = file.name
            file.write(self.supervisor.getUrdf().encode('utf-8'))
        self.armChain = Chain.from_urdf_file(filename, active_links_mask=[False, True, True, True, True, True, True, False])

        # Initialize the arm motors and encoders.
        self.motors = []
        for link in self.armChain.links:
            if 'motor' in link.name:
                motor = self.supervisor.getDevice(link.name)
                motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(self.timeStep)
                self.motors.append(motor)

        # ! Get the arm and target nodes. ????
        target = self.supervisor.getFromDef('TARGET')
        arm = self.supervisor.getSelf()

    def clear(self):
        self.input_text_edit.clear()

    def send(self):
        # ! Can chatgpt recongnize \n ?
        content =  self.input_text_edit.toPlainText()
        if len(content) == 0:
            return
        proc = subprocess.Popen(content, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        try:
            outs, errs = proc.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            print("Command execution timed out.")
        else:
            if len(errs) == 0:
                msg = f"User:\t" + content
                self.show_message(msg)
                self.clear()

                msg = f"ChatGPT:\t" + outs.decode('gbk')
                self.show_message(msg)
            else:
                msg = f"ERROR:\t" + errs.decode('gbk')
                self.show_message(msg)

    def send_demo(self):
        content =  self.input_text_edit.toPlainText()
        if len(content) == 0:
            return

        if content == 'Draw a circle':
            msg = f"User:\t" + content
            self.show_message(msg)
            self.clear()

            msg = f"ChatGPT:\t" + "I'm drawing the cricle :)"
            self.show_message(msg)

            self.gui.processEvents()

            self.draw()

        elif content == 'Stupid robot':
            msg = f"User:\t" + content
            self.show_message(msg)
            self.clear()

            msg = f"ChatGPT:\t" + "You hurt me T_T"
            self.show_message(msg)

        else:
            msg = f"User:\t" + content
            self.show_message(msg)
            self.clear()

            msg = f"ChatGPT:\t" + "Give me money, then ask me to other things!"
            self.show_message(msg)

    def show_message(self, msg):
        self.output_text_edit.appendPlainText(msg)

    def draw(self):
        print('Draw a circle on the paper sheet...')
        t0 = self.supervisor.getTime()
        while self.supervisor.step(self.timeStep) != -1:
            t = self.supervisor.getTime()

            # Use the circle equation relatively to the arm base as an input of the IK algorithm.
            x = 0.25 * math.cos(t) + 1.1
            y = 0.25 * math.sin(t) - 0.95
            z = 0.05

            # Call "ikpy" to compute the inverse kinematics of the arm.
            initial_position = [0] + [m.getPositionSensor().getValue() for m in self.motors] + [0]
            ikResults = self.armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)

            # Actuate the 3 first arm motors with the IK results.
            for i in range(3):
                self.motors[i].setPosition(ikResults[i + 1])
            # Keep the hand orientation down.
            self.motors[4].setPosition(-ikResults[2] - ikResults[3] + math.pi / 2)
            # Keep the hand orientation perpendicular.
            self.motors[5].setPosition(ikResults[1])

            # Conditions to start/stop drawing and leave this loop.
            if self.supervisor.getTime() > t0 + 2 * math.pi + 1.5:
                break
            elif self.supervisor.getTime() > t0 + 1.5:
                # Note: start to draw at 1.5 second to be sure the arm is well located.
                self.supervisor.getDevice('pen').write(True)


app = QtWidgets.QApplication(sys.argv)

window = MainWindow(app)
window.show()
app.exec()