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
            
        self.line_edits = []
        for _ in range(9):
            line_edit = QLineEdit()
            self.line_edits.append(line_edit)
            container.addWidget(line_edit)
        self.line_edits[0].setPlaceholderText('Address')
        self.line_edits[1].setPlaceholderText('Data 1')
        self.line_edits[2].setPlaceholderText('Data 2')
        self.line_edits[3].setPlaceholderText('Data 3')
        self.line_edits[4].setPlaceholderText('Data 4')
        self.line_edits[5].setPlaceholderText('Data 5')
        self.line_edits[6].setPlaceholderText('Data 6')
        self.line_edits[7].setPlaceholderText('Data 7')
        self.line_edits[8].setPlaceholderText('Data 8')

        self.init_btn = QPushButton('Initialize')
        self.init_btn.clicked.connect(self.init_can)
        
        self.send_btn = QPushButton('Test Stage')
        #self.send_btn.setEnabled(False)
        self.send_btn.clicked.connect(self.test_stage)
        
        self.textbox = QTextEdit()
        
        # Add to layout.
        self.layout.addWidget(self.hw_combo)
        self.layout.addWidget(self.hw_rate)
        self.layout.addWidget(self.init_btn)
        self.layout.addLayout(container)
        self.layout.addWidget(self.send_btn)
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
            self.textbox.setText('Initialized.\n')
            self.send_btn.setEnabled(True)
        except Exception as err:
            error_message = self.can.get_formatted_error(status)
            self.textbox.setText('Error in initializing CAN device: {}.\n'.format(error_message))
            
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
            logger.info('Stages at 0, 0. Test exposure finished')
            
            self.textbox.append('Stages at 0, 0. Test exposure finished\n')
           
        except Exception as err:
            self.textbox.append('Error in sending the CAN message: {}.\n'.format(err))
            
        

# Main function
if __name__ == '__main__':
   app = QApplication(sys.argv)
   test = TestApp()
   test.show()
   app.exec()
