#!/usr/bin/env python
'''
Simple QT gui for ploting ADC data
Currently, just dummy data is used ...

cezary.sydlo@desy.de

Changed to use AD16 library by martin.hierholzer@desy.de
'''

import sys
import time

# GUI is QT
from pyqtgraph.Qt import QtCore, QtGui

# plotting is done with PyQtGraph
import pyqtgraph as pg

# a bit of calculation
import numpy as np

import libad16

class MainWindow(QtGui.QMainWindow):

    counter = 0

    def __init__(self):
        super(MainWindow, self).__init__()

        
        self.grid = QtGui.QGridLayout()
        self.grid.setSpacing(10)

        self.plot = pg.PlotWidget()
        self.curve = self.plot.plot()
        self.plot.setRange(QtCore.QRectF(0, -300000, 1000, 600000))
        self.grid.addWidget(self.plot,1,0,1,2)
        
        self.text1 = QtGui.QLabel('Sampling rate:')
        self.grid.addWidget(self.text1,2,0)
        self.samplingRate = QtGui.QComboBox()
        self.samplingRate.addItem("100 kHz", libad16.rate.Hz100000)
        self.samplingRate.addItem("50 kHz", libad16.rate.Hz50000)
        self.samplingRate.addItem("10 kHz", libad16.rate.Hz10000)
        self.samplingRate.addItem("5 kHz", libad16.rate.Hz5000)
        self.samplingRate.addItem("1 kHz", libad16.rate.Hz1000)
        self.grid.addWidget(self.samplingRate,2,1)
        
        self.text2 = QtGui.QLabel('Number of samples per channel:')
        self.grid.addWidget(self.text2,3,0)
        self.samples = QtGui.QSpinBox()
        self.samples.setRange(1,100000)
        self.samples.setValue(1024)
        self.grid.addWidget(self.samples,3,1)

        self.widget = QtGui.QWidget();
        self.widget.setLayout(self.grid);
        self.setCentralWidget(self.widget);

        self.setWindowTitle("PyQT ADC scope")
        self.resize(800, 600)
        self.makeMenu()

        self.statusBar = QtGui.QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage('Ready')
        
        self.show()
        
        # open AD16 and start first conversion
        self.ad16 = libad16.ad16()
        self.ad16.open("ad16dummy.map","ad16dummy.map")
        self.ad16.startConversion()
        
        # start the automatic update
        self.start_time = time.time()
        self.timer = QtCore.QTimer()
        self.timer.start(1)    # timeout in milliseconds ... 50ms => 20 frames per second
        self.connect(self.timer, QtCore.SIGNAL('timeout()'), self.updateplot)
    
    def makeMenu(self):
        self._exitAction = QtGui.QAction("&Close", None)
        self.connect(self._exitAction, QtCore.SIGNAL('triggered()'), self.slotClose)
        
        self._helpAction = QtGui.QAction("Help", None)
        self.connect(self._helpAction, QtCore.SIGNAL('triggered()'), self.slotHelp)
        
        menuBar = self.menuBar()
        fileMenu = menuBar.addMenu("&File")
        fileMenu.addAction(self._exitAction)
        
        helpMenu = menuBar.addMenu("&Help")
        helpMenu.addAction(self._helpAction)

    def slotClose(self):
        self.close()

    def slotHelp(self):
        QtGui.QMessageBox.information(None, "Help", "Ask Cezary!")
        
    def updateplot(self):
        '''
        Here goes all the magic
        '''
        
        # check if conversion is complete
        if self.ad16.conversionComplete() == 0:
            return

        # compute fps
        fps = 1/(time.time() - self.start_time)
        self.counter += 1

        # get data
        self.start_time = time.time()
        self.ad16.read()
        signal = self.ad16.getChannelData(1)

        # calculate something
        time_get_image = time.time()
        
        # draw everything
        time_calc_image = time.time()
        
        # plot the signal
        self.curve.setData(signal)
        time_plot_image = time.time()
        
        # What the hell is going on ... put info into statusbar
        self.statusBar.showMessage(
            'Block: %i\
        \tfps: %.4fpps\
        \tAcq: %.3fms\
        \tCalc: %.3fms\
        \tPlot: %.3fms'
            %(self.counter, fps,
            1000 * (time_get_image - self.start_time),
            1000 * (time_calc_image - time_get_image),
            1000 * (time_plot_image - time_calc_image)))

        # start the next conversion
        val = self.samplingRate.itemData(self.samplingRate.currentIndex()).toPyObject()
        self.ad16.setSamplingRate(val)
        self.ad16.setSamplesPerBlock( self.samples.value() )
        self.ad16.startConversion()
        

       
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    mainWin = MainWindow()
    sys.exit(app.exec_())
