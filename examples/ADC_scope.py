#!/usr/bin/env python
'''
Simple QT gui for ploting ADC data
Currently, just dummy data is used ...

cezary.sydlo@desy.de

Changed to use AD16 library by martin.hierholzer@desy.de
'''

import sys
import time
from PyQt4.QtGui import QFileDialog
from PyQt4.Qt import QMessageBox

NUMBER_OF_CHANNELS = 16
# the number of samples (per channel) is currently fixed, this is a firmware limitation and will change in future
NUMBER_OF_SAMPLES = 65536

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
        self.curve = []
        for i in range(0,NUMBER_OF_CHANNELS):
            myplot = self.plot.plot()
            myplot.setPen(pg.mkPen(pg.intColor(i)))
            self.curve.append(myplot)
        self.plot.setRange(QtCore.QRectF(0, -2**17, 1./79.*NUMBER_OF_SAMPLES, 2**18))
        self.plot.setLabel('bottom','Time [ms]')
        self.plot.setLabel('left','ADC value')
        self.grid.addWidget(self.plot,1,0,1,33)

        self.plot2 = pg.PlotWidget()
        self.curve2 = []
        for i in range(0,NUMBER_OF_CHANNELS):
            myplot = self.plot2.plot()
            myplot.setPen(pg.mkPen(pg.intColor(i)))
            self.curve2.append(myplot)
        self.plot2.setLogMode(0,1)
        #self.plot2.setRange(QtCore.QRectF(0, 0.0000000001, 79./2., 1.0))
        self.plot2.setLabel('bottom','Frequency [kHz]')
        self.plot2.setLabel('left','Power Spectrum Density')
        self.grid.addWidget(self.plot2,2,0,1,33)
        
        self.text1 = QtGui.QLabel('Sampling rate:')
        self.grid.addWidget(self.text1,3,0)
        self.samplingRate = QtGui.QComboBox()
        #self.samplingRate.addItem("100 kHz", libad16.rate.Hz100000)
        #self.samplingRate.addItem("50 kHz", libad16.rate.Hz50000)
        #self.samplingRate.addItem("10 kHz", libad16.rate.Hz10000)
        #self.samplingRate.addItem("5 kHz", libad16.rate.Hz5000)
        #self.samplingRate.addItem("1 kHz", libad16.rate.Hz1000)
        self.samplingRate.addItem("79 kHz", libad16.rate.Hz100000)
        self.grid.addWidget(self.samplingRate,3,1,1,3)
        
        self.text2 = QtGui.QLabel('Number of samples per channel:')
        self.grid.addWidget(self.text2,4,0)
        self.samples = QtGui.QSpinBox()
        self.samples.setRange(NUMBER_OF_SAMPLES,NUMBER_OF_SAMPLES)
        self.samples.setValue(NUMBER_OF_SAMPLES)
        self.grid.addWidget(self.samples,4,1,1,3)
        
        self.startStopButton = QtGui.QPushButton('Start')
        self.grid.addWidget(self.startStopButton,3,29,1,4)
        self.DAQrunning = 0
        self.startStopButton.clicked.connect(self.clickStartStop)
        
        
        self.text3 = QtGui.QLabel('Channels:')
        self.grid.addWidget(self.text3,5,0)
        self.chn = []
        self.chnlabel = []
        for i in range(0,NUMBER_OF_CHANNELS):
            self.chn.append(QtGui.QCheckBox())
            if i == 1:
                self.chn[i].setChecked(1)
            self.grid.addWidget(self.chn[i],5,1+2*i)
            self.chnlabel.append(QtGui.QLabel(str(i)))
            
            pal = QtGui.QPalette()
            pal.setColor(QtGui.QPalette.WindowText, pg.intColor(i))
            self.chnlabel[i].setPalette(pal)
            
            self.grid.addWidget(self.chnlabel[i],5,2+2*i)
        
        self.text4 = QtGui.QLabel('FFT:')
        self.grid.addWidget(self.text4,6,0)
        self.fft = []
        self.fftlabel = []
        for i in range(0,NUMBER_OF_CHANNELS):
            self.fft.append(QtGui.QCheckBox())
            if i == 1:
                self.fft[i].setChecked(1)
            self.grid.addWidget(self.fft[i],6,1+2*i)
            self.fftlabel.append(QtGui.QLabel(str(i)))
            
            pal = QtGui.QPalette()
            pal.setColor(QtGui.QPalette.WindowText, pg.intColor(i))
            self.fftlabel[i].setPalette(pal)
            
            self.grid.addWidget(self.fftlabel[i],6,2+2*i)

        self.widget = QtGui.QWidget();
        self.widget.setLayout(self.grid);
        self.setCentralWidget(self.widget);

        self.setWindowTitle("PyQT ADC scope")
        self.resize(800, 600)
          
        self.makeMenu()

        self.statusBar = QtGui.QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage('Ready.')
        
        self.show()
        
        # open AD16 and start first conversion
        self.ad16 = libad16.ad16()
        self.ad16.open("ad16dummy.map","ad16dummy.map")
        #self.ad16.setMode(libad16.mode.SOFTWARE_TRIGGER)
        #self.ad16.startConversion()
        
        # start the automatic update
        self.start_time = time.time()
        self.timer = QtCore.QTimer()
        self.timer.start(1)    # timeout in milliseconds ... 50ms => 20 frames per second
        self.connect(self.timer, QtCore.SIGNAL('timeout()'), self.updateplot)
        
        # initialise variable
        self.signal = None
    
    def clickStartStop(self):
        if self.DAQrunning == 0:
            self.startStopButton.setText('Stop')
            self.DAQrunning = 1
            self.ad16.startConversion()
        else:
            self.startStopButton.setText('Start')
            self.DAQrunning = 0
    
    def makeMenu(self):
        self._saveAction = QtGui.QAction("&Save current data...", None)
        self.connect(self._saveAction, QtCore.SIGNAL('triggered()'), self.slotSave)

        self._exitAction = QtGui.QAction("&Close", None)
        self.connect(self._exitAction, QtCore.SIGNAL('triggered()'), self.slotClose)
        
        self._helpAction = QtGui.QAction("Help", None)
        self.connect(self._helpAction, QtCore.SIGNAL('triggered()'), self.slotHelp)
        
        menuBar = self.menuBar()
        fileMenu = menuBar.addMenu("&File")
        fileMenu.addAction(self._saveAction)
        fileMenu.addAction(self._exitAction)
        
        helpMenu = menuBar.addMenu("&Help")
        helpMenu.addAction(self._helpAction)

    def slotSave(self):
        
        # check if data has been acquired already
        if self.signal == None:
            QtGui.QMessageBox.information(self, 'Error', 'No data to be saved. Start data acquisition first!')
            return
        
        # create file-save dialog
        dlg = QFileDialog(self)
        dlg.setAcceptMode(QFileDialog.AcceptSave)
        dlg.setWindowTitle('Save current data')
        dlg.setViewMode( QtGui.QFileDialog.Detail )
        dlg.setNameFilters( [self.tr('CSV Files (*.csv)'), self.tr('Compressed CSV Files (*.csv.gz)'), self.tr('All Files (*)')] )
        dlg.setDefaultSuffix('csv')
        
        # show dialog, save only if user did not cancel
        if dlg.exec_() :
            # file name must be converted into standard python string
            name = str(dlg.selectedFiles()[0])
            # save in coma separated values format. The array is transposed so we have the samples in rows and the channels in columns
            np.savetxt(name, np.transpose(self.signal), '%.18e', ',', '\n')
            # show message of success
            QtGui.QMessageBox.information(self, 'Data saved', 'Current data has been saved to file "'+name+'".')

    def slotClose(self):
        self.close()

    def slotHelp(self):
        QtGui.QMessageBox.information(self, "Help", "Ask Cezary!")
        
    def updateplot(self):
        '''
        Here goes all the magic
        '''
        
        # check if conversion is complete
        if self.ad16.conversionComplete() == 0:
            return

        # check if acquisition running
        if self.DAQrunning == 0:
            self.statusBar.showMessage('Stopped. Click "Start" to run the acquisition.')
            return

        # compute fps
        fps = 1/(time.time() - self.start_time)
        self.counter += 1
        self.start_time = time.time()

        # get data
        self.ad16.read()
        self.signal = np.zeros( (NUMBER_OF_CHANNELS,NUMBER_OF_SAMPLES), dtype=np.int32)
        for i in range(0,NUMBER_OF_CHANNELS):
            self.ad16.getChannelData(i, self.signal[i])

        # compute x-axis values
        time_step = 1./79.
        times = np.linspace(0., time_step*NUMBER_OF_SAMPLES, NUMBER_OF_SAMPLES, 0)
        
        # trigger next conversion
        self.ad16.startConversion()
        
        # take time for obtaining the data
        time_get_image = time.time()

        # calculate fourier transform
        powerspectrum = np.zeros( (NUMBER_OF_CHANNELS,NUMBER_OF_SAMPLES), dtype=np.float64)
        for i in range(0,NUMBER_OF_CHANNELS):
            if self.fft[i].isChecked():
                powerspectrum[i] = np.abs(np.fft.fft(self.signal[i]))**2
                powerspectrum[i] = powerspectrum[i]/np.linalg.norm(powerspectrum[i])
        
        freqs = np.fft.fftfreq(self.signal[0].size, time_step)
        idx   = np.argsort(freqs)
        
        # take time for the calculations
        time_calc_image = time.time()
        
        # plot the signal
        for i in range(0,NUMBER_OF_CHANNELS):
            if self.chn[i].isChecked():
                self.curve[i].setData(times,self.signal[i])
            else:
                self.curve[i].clear()
        
        # plot the power spectrum
        for i in range(0,NUMBER_OF_CHANNELS):
            if self.fft[i].isChecked():
                self.curve2[i].setData(freqs[idx[NUMBER_OF_SAMPLES/2:NUMBER_OF_SAMPLES-1]],powerspectrum[i][idx[NUMBER_OF_SAMPLES/2:NUMBER_OF_SAMPLES-1]])
            else:
                self.curve2[i].clear()

        # take time for plotting
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

        # update sample rate and number of samples
        #val = self.samplingRate.itemData(self.samplingRate.currentIndex()).toPyObject()
        #self.ad16.setSamplingRate(val)
        #self.ad16.setSamplesPerBlock( self.samples.value() )
        

       
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    mainWin = MainWindow()
    sys.exit(app.exec_())
