#!/usr/bin/env python
'''
AD16 scope application with FFT

Authors:
cezary.sydlo@desy.de
martin.hierholzer@desy.de
'''

# system libraries and numpy
import sys
import time
import datetime
import numpy as np

# GUI is QT
from PyQt4.QtGui import QFileDialog
from PyQt4.Qt import QMessageBox
from pyqtgraph.Qt import QtCore, QtGui

# plotting is done with PyQtGraph
import pyqtgraph as pg

# ad16 library
import libad16

# some constants: number of channels is fixed in hardware
NUMBER_OF_CHANNELS = 16
# the number of samples (per channel) is currently fixed, this is a firmware limitation and will change in future
NUMBER_OF_SAMPLES = 65536

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
        self.plot.setRange(QtCore.QRectF(0, -2**17, 1./79.1*NUMBER_OF_SAMPLES, 2**18))
        self.plot.setLabel('bottom','Time [ms]')
        self.plot.setLabel('left','ADC value')
        self.grid.addWidget(self.plot,1,0,1,35)

        self.plot2 = pg.PlotWidget()
        self.curve2 = []
        for i in range(0,NUMBER_OF_CHANNELS):
            myplot = self.plot2.plot()
            myplot.setPen(pg.mkPen(pg.intColor(i)))
            self.curve2.append(myplot)
        self.plot2.setLogMode(0,1)
        #self.plot2.setRange(QtCore.QRectF(0, 0.0000000001, 79.1/2., 1.0))
        self.plot2.setLabel('bottom','Frequency [kHz]')
        self.plot2.setLabel('left','Power Spectrum Density')
        self.grid.addWidget(self.plot2,2,0,1,35)
        
        self.text1 = QtGui.QLabel('Sampling rate:')
        self.grid.addWidget(self.text1,3,0)
        self.samplingRate = QtGui.QComboBox()
        #self.samplingRate.addItem("100 kHz", libad16.rate.Hz100000)
        #self.samplingRate.addItem("50 kHz", libad16.rate.Hz50000)
        #self.samplingRate.addItem("10 kHz", libad16.rate.Hz10000)
        #self.samplingRate.addItem("5 kHz", libad16.rate.Hz5000)
        #self.samplingRate.addItem("1 kHz", libad16.rate.Hz1000)
        self.samplingRate.addItem("79.1 kHz", libad16.rate.Hz100000)
        self.grid.addWidget(self.samplingRate,3,1,1,3)
        
        self.text2 = QtGui.QLabel('Number of samples per channel:')
        self.grid.addWidget(self.text2,4,0)
        self.samples = QtGui.QSpinBox()
        self.samples.setRange(NUMBER_OF_SAMPLES,NUMBER_OF_SAMPLES)
        self.samples.setValue(NUMBER_OF_SAMPLES)
        self.grid.addWidget(self.samples,4,1,1,3)
        
        self.startStopButton = QtGui.QPushButton('Start')
        self.grid.addWidget(self.startStopButton,3,32,1,3)
        self.DAQrunning = 0
        self.startStopButton.clicked.connect(self.clickStartStop)
        
        
        self.text3 = QtGui.QLabel('Channels:')
        self.grid.addWidget(self.text3,5,0)
        self.chn = []
        self.chnlabel = []
        for i in range(0,NUMBER_OF_CHANNELS):
            self.chn.append(QtGui.QCheckBox())
            if i == 0:
                self.chn[i].setChecked(1)
            self.grid.addWidget(self.chn[i],5,1+2*i)
            self.chnlabel.append(QtGui.QLabel(str(i)))
            
            pal = QtGui.QPalette()
            pal.setColor(QtGui.QPalette.WindowText, pg.intColor(i))
            self.chnlabel[i].setPalette(pal)
            
            self.grid.addWidget(self.chnlabel[i],5,2+2*i)
        
        self.toggleChannelsButton = QtGui.QPushButton('toggle all')
        self.grid.addWidget(self.toggleChannelsButton,5,34)
        self.toggleChannelsButton.clicked.connect(self.clickToggleChannels)
        
        self.text4 = QtGui.QLabel('FFT:')
        self.grid.addWidget(self.text4,6,0)
        self.fft = []
        self.fftlabel = []
        for i in range(0,NUMBER_OF_CHANNELS):
            self.fft.append(QtGui.QCheckBox())
            if i == 0:
                self.fft[i].setChecked(1)
            self.grid.addWidget(self.fft[i],6,1+2*i)
            self.fftlabel.append(QtGui.QLabel(str(i)))
            
            pal = QtGui.QPalette()
            pal.setColor(QtGui.QPalette.WindowText, pg.intColor(i))
            self.fftlabel[i].setPalette(pal)
            
            self.grid.addWidget(self.fftlabel[i],6,2+2*i)

        self.toggleFFTsButton = QtGui.QPushButton('toggle all')
        self.grid.addWidget(self.toggleFFTsButton,6,34)
        self.toggleFFTsButton.clicked.connect(self.clickToggleFFTs)

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
    
    def clickToggleChannels(self):
        for i in range(0,NUMBER_OF_CHANNELS):
            self.chn[i].toggle()

    def clickToggleFFTs(self):
        for i in range(0,NUMBER_OF_CHANNELS):
            self.fft[i].toggle()

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
            # open output file and write header (savetxt does not support headers in older versions)
            f = open(name,'w')
            f.write('# AD16 data dump\n')
            f.write('# Sample frequency:              79.1 kHz\n')
            f.write('# Number of channels:            16\n')
            f.write('# Number of samples per channel: 65536\n')
            f.write('# File created on:               '+datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')+'\n')
            f.write('# Time [ms]')
            for i in range(0,NUMBER_OF_CHANNELS):
                f.write(',      {0:2d}'.format(i))
            f.write('\n')
            # prepare array to be saved: transpose (to have samples in rows) and add time column
            savedata = self.signal.astype(np.float32)
            savedata = np.transpose(savedata)
            savedata = np.insert(savedata, 0, self.times, axis=1)
            # write data in coma separated values format
            fmt = []
            fmt.append('%11.3f')
            for i in range(1,NUMBER_OF_CHANNELS+1):
                fmt.append(' %7d')
            np.savetxt(f, savedata, fmt, ',', '\n')
            # close file
            f.close()
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
        time_step = 1./79.1
        self.times = np.linspace(0., time_step*float(NUMBER_OF_SAMPLES), num=NUMBER_OF_SAMPLES, endpoint=False)
        
        # trigger next conversion
        self.ad16.startConversion()
        
        # take time for obtaining the data
        time_get_image = time.time()

        # calculate fourier transform
        powerspectrum = np.zeros( (NUMBER_OF_CHANNELS,NUMBER_OF_SAMPLES), dtype=np.float64)
        for i in range(0,NUMBER_OF_CHANNELS):
            if self.fft[i].isChecked():
                powerspectrum[i] = np.abs(np.fft.fft(self.signal[i]))**2 / NUMBER_OF_SAMPLES 
                #powerspectrum[i] = powerspectrum[i]/np.linalg.norm(powerspectrum[i]) * np.linalg.norm(np.abs(self.signal[i])**2)
        
        freqs = np.fft.fftfreq(self.signal[0].size, time_step)
        idx   = np.argsort(freqs)
        
        # take time for the calculations
        time_calc_image = time.time()
        
        # plot the signal
        for i in range(0,NUMBER_OF_CHANNELS):
            if self.chn[i].isChecked():
                self.curve[i].setData(self.times,self.signal[i])
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
