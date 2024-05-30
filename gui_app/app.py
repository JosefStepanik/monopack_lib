# Import common modules
import os, sys

# Finding the path of the current file
current_path = os.path.dirname(__file__)
lib_path = os.path.join(current_path, '..')
sys.path.append(lib_path)

# Adding the path to the sys.path
lib_path = os.path.join(current_path, '..', 'src')
sys.path.append(lib_path)


# Import own modules
from src.stages_monopack import *        ## PCAN-Basic library import
from Peak_PCAN.src.tool_can import *
from PyQt6.QtWidgets import QApplication, QHBoxLayout, QWidget, QPushButton, QLineEdit, QComboBox, QVBoxLayout, QTextEdit


class TestApp(QWidget):
    '''
    This class is used to test tool_can module.
    '''
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("CAN Communication")
        self.setGeometry(100, 100, 400, 400)
        self.layout = QVBoxLayout()
        container = QHBoxLayout()
        
        self.m_NonPnPHandles = HW_HANDLES
        self.m_BAUDRATES = HW_BAUDRATES
        self.m_errors = ERRORS
        

        # Add widgets.
        self.hw_combo = QComboBox()
        self.hw_rate = QComboBox()
        for item in self.m_NonPnPHandles:
            self.hw_combo.addItem(item)
        for item in self.m_BAUDRATES:
            self.hw_rate.addItem(item)
            
        self.buttons = []
        for i in range(6):
            btn = QPushButton()
            btn.setEnabled(False)
            self.buttons.append(btn)
            container.addWidget(btn)
            
        self.buttons[0].setText('Init Stage')
        self.buttons[0].clicked.connect(self.init_stage)
        self.buttons[1].setText('Center')
        self.buttons[1].clicked.connect(self.center_stage)
        self.buttons[2].setText('Service pos')
        self.buttons[2].clicked.connect(self.service_pos)
        self.buttons[3].setText('Home')
        self.buttons[3].clicked.connect(self.home_stage)
        self.buttons[4].setText('Data 3')
        self.buttons[5].setText('Data 4')

        self.init_btn = QPushButton('Initialize')
        self.init_btn.clicked.connect(self.init_can)
        
        self.test_btn = QPushButton('Test Stage')
        self.test_btn.setEnabled(False)
        self.test_btn.clicked.connect(self.test_stage)
        
        self.textbox = QTextEdit()
        
        # Add to layout.
        self.layout.addWidget(self.hw_combo)
        self.layout.addWidget(self.hw_rate)
        self.layout.addWidget(self.init_btn)
        self.layout.addLayout(container)
        self.layout.addWidget(self.test_btn)
        self.layout.addWidget(self.textbox)
        
        # Set layout.
        self.setLayout(self.layout)
        
        
    def init_can(self):
        '''
        This function is used to initialize the CAN.
        '''
        try:
            self.can = NewPCANBasic(PcanHandle=self.m_NonPnPHandles[self.hw_combo.currentText()])
            status = self.can.Initialize(self.can.PcanHandle, self.m_BAUDRATES[self.hw_rate.currentText()])
            if PCAN_ERROR_OK != status:
                raise
            self.textbox.setText('CAN initialized.\n')
            self.test_btn.setEnabled(True)
            self.buttons[0].setEnabled(True)
        except Exception as err:
            error_message = self.can.get_formatted_error(status)
            del self.can
            self.textbox.setText('Error in initializing CAN device: {}.\n'.format(error_message))
            
    def init_stage(self):
        '''
        This function is used to initialize the stage.
        '''
        try:
            self.stage = StagesPI( self.can, verbose=False)
            self.stage.init_stages()
            self.textbox.append('Initialization of stage.\n')
            self.textbox.append(self.stage.init_msg[0])
            self.textbox.append(self.stage.init_msg[1])
            self.buttons[1].setEnabled(True)
            self.buttons[2].setEnabled(True)
            self.buttons[3].setEnabled(True)
        except Exception as err:
            self.textbox.append('Error in initializing stage: {}.\n'.format(err))
            
    def center_stage(self):
        '''
        This function is used to center the stage.
        '''
        try:
            self.stage.move_to_center()
            self.textbox.append('Stage go to the centre.\n')
        except Exception as err:
            self.textbox.append('Error in centering stage: {}.\n'.format(err))
            
    def home_stage(self):
        '''
        This function is used to go to the home position.
        '''
        try:
            self.stage.go_home()
            self.textbox.append('Stage go to the home position.\n')
        except Exception as err:
            self.textbox.append('Error in going to home position: {}.\n'.format(err))
            
    def service_pos(self):
        '''
        This function is used to go to the service position.
        '''
        try:
            self.stage.move_to_xy(200, 5)
            self.textbox.append('Stage go to the service position.\n')
        except Exception as err:
            self.textbox.append('Error in going to service position: {}.\n'.format(err))
            
    def test_stage(self):
        """
        Run basic stages tests
        """
        self.textbox.append('Running test of stages\n')

        try:
            stage = StagesPI(self, self.can, verbose=False)

            stage.reference_stages(force_reference=False)
            logger.info('REFERENCED')
            time.sleep(2)

            logger.info('Going to 20, 10')
            stage.move_to_x(20)
            stage.move_to_y(10)

            stage.is_ready('X')
            stage.is_ready('Y')
            stage.print_current_positions()

            logger.info('Going to 50, 50')
            stage.move_to_x(50)
            stage.move_to_y(50)

            stage.is_ready('X')
            stage.is_ready('Y')
            stage.print_current_positions()

            logger.info('Going home')
            stage.go_home('X')
            stage.go_home('Y')

            time.sleep(1)
   
            stage.is_ready('X')
            stage.is_ready('Y')
            stage.print_current_positions()
            logger.info('Stages at 0, 0. Test procedure finished')
            
            self.textbox.append('Stages at 0, 0. Test procedure finished\n')
           
        except Exception as err:
            self.textbox.append('Error during making test of stage: {}.\n'.format(err))
            
        

# Main function
if __name__ == '__main__':
   app = QApplication(sys.argv)
   test = TestApp()
   test.show()
   app.exec()
