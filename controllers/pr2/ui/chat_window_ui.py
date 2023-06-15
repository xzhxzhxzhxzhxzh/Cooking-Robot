# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'chat_window.ui'
##
## Created by: Qt User Interface Compiler version 6.5.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QHBoxLayout, QMainWindow,
    QMenuBar, QPlainTextEdit, QPushButton, QSizePolicy,
    QStatusBar, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(355, 269)
        self.workspace = QWidget(MainWindow)
        self.workspace.setObjectName(u"workspace")
        self.verticalLayout = QVBoxLayout(self.workspace)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.widgets = QWidget(self.workspace)
        self.widgets.setObjectName(u"widgets")
        self.verticalLayout_2 = QVBoxLayout(self.widgets)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.outputs_widgets = QWidget(self.widgets)
        self.outputs_widgets.setObjectName(u"outputs_widgets")
        self.verticalLayout_3 = QVBoxLayout(self.outputs_widgets)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.output_text_edit = QPlainTextEdit(self.outputs_widgets)
        self.output_text_edit.setObjectName(u"output_text_edit")
        self.output_text_edit.setFrameShadow(QFrame.Plain)
        self.output_text_edit.setReadOnly(True)

        self.verticalLayout_3.addWidget(self.output_text_edit)


        self.verticalLayout_2.addWidget(self.outputs_widgets)

        self.input_widgets = QWidget(self.widgets)
        self.input_widgets.setObjectName(u"input_widgets")
        self.horizontalLayout = QHBoxLayout(self.input_widgets)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.input_text_edit = QPlainTextEdit(self.input_widgets)
        self.input_text_edit.setObjectName(u"input_text_edit")
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.input_text_edit.sizePolicy().hasHeightForWidth())
        self.input_text_edit.setSizePolicy(sizePolicy)
        self.input_text_edit.setMinimumSize(QSize(0, 59))
        self.input_text_edit.setMaximumSize(QSize(16777215, 59))
        self.input_text_edit.setFrameShadow(QFrame.Plain)
        self.input_text_edit.setReadOnly(False)

        self.horizontalLayout.addWidget(self.input_text_edit)

        self.send_button = QPushButton(self.input_widgets)
        self.send_button.setObjectName(u"send_button")
        sizePolicy1 = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.send_button.sizePolicy().hasHeightForWidth())
        self.send_button.setSizePolicy(sizePolicy1)
        self.send_button.setMinimumSize(QSize(50, 59))
        self.send_button.setMaximumSize(QSize(50, 59))

        self.horizontalLayout.addWidget(self.send_button)

        self.clear_button = QPushButton(self.input_widgets)
        self.clear_button.setObjectName(u"clear_button")
        sizePolicy1.setHeightForWidth(self.clear_button.sizePolicy().hasHeightForWidth())
        self.clear_button.setSizePolicy(sizePolicy1)
        self.clear_button.setMinimumSize(QSize(50, 59))
        self.clear_button.setMaximumSize(QSize(50, 59))

        self.horizontalLayout.addWidget(self.clear_button)


        self.verticalLayout_2.addWidget(self.input_widgets)


        self.verticalLayout.addWidget(self.widgets)

        MainWindow.setCentralWidget(self.workspace)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 355, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.output_text_edit.setPlainText("")
        self.input_text_edit.setPlainText("")
        self.send_button.setText(QCoreApplication.translate("MainWindow", u"Send", None))
        self.clear_button.setText(QCoreApplication.translate("MainWindow", u"Clear", None))
    # retranslateUi

