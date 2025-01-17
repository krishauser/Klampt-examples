from klampt import WorldModel
from klampt import vis

vis.init('PyQt')
from klampt.vis import glinit
if glinit.active() == 'PyQt6':
    from PyQt6.QtCore import *
    from PyQt6.QtGui import *
    from PyQt6.QtWidgets import *
else:
    from PyQt5.QtCore import *
    from PyQt5.QtGui import *
    from PyQt5.QtWidgets import *

def make_world():
    w = WorldModel()
    w.readFile("../../../data/tx90cupscupboard.xml")
    #w.readFile("../../../data/robots/tx90ball.rob")
    return w


def test_custom_gui():
    w = make_world()
    def make_gui(glwidget):
        #place your Qt code here and place the glwidget where it needs to be
        w = QMainWindow()
        w.resize(800,800)
        glwidget.setMaximumSize(4000,4000)
        if glinit.active() == 'PyQt6':
            glwidget.setSizePolicy(QSizePolicy(QSizePolicy.Policy.Maximum,QSizePolicy.Policy.Maximum))
        else:
            glwidget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum,QSizePolicy.Maximum))
        area = QWidget(w)
        layout = QVBoxLayout()
        layout.addWidget(glwidget)
        layout.addWidget(QPushButton("Click me"))
        area.setLayout(layout)
        w.setCentralWidget(area)
        return w

    vis.customUI(make_gui)
    vis.add("world",w)
    #vis.loop()
    vis.show()
    vis.spin(float('inf'))
    vis.kill()

if __name__ == '__main__':
    test_custom_gui()