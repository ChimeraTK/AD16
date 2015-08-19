#!/usr/bin/env python
'''
AD16 scope application with FFT

Authors:
cezary.sydlo@desy.de
martin.hierholzer@desy.de
'''

# system libraries and numpy
import os.path
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
from numpy import NaN, mean


class MainWindow(QtGui.QMainWindow):

    # some globals:
    # number of channels is fixed in hardware
    NUMBER_OF_CHANNELS = 16
    # the number of samples (per channel) is currently fixed, this is a firmware limitation and might change in future
    NUMBER_OF_SAMPLES = 65536
    # maximum number of 1/10 seconds to display in amplitude vs. time view
    NUMBER_OF_TIME_SLICES = 6000
    # to determine the amplitude, the FFT is integrated over a window around the peak: (-FFT_INTEGRATION_HALF_WINDOW:FFT_INTEGRATION_HALF_WINDOW)
    FFT_INTEGRATION_HALF_WINDOW = np.int( np.round(250./(100000./NUMBER_OF_SAMPLES)) )  # 250 Hz
    # time step in milliseconds (inverse sampling rate)
    TIME_STEP = 1./100.

    counter = 0

#######################################################################################################################
# Constructor: Create panel layout and initialise AD16
    def __init__(self):
        super(MainWindow, self).__init__()

        # Create grid laoyout
        self.grid = QtGui.QGridLayout()
        self.grid.setSpacing(10)

        # Upper plot window for time domain view
        self.plot = pg.PlotWidget()
        self.curve = []
        for i in range(0,self.NUMBER_OF_CHANNELS):
            myplot = self.plot.plot()
            myplot.setPen(pg.mkPen(pg.intColor(i)))
            self.curve.append(myplot)
        self.plot.setRange(QtCore.QRectF(0, -2**17, 1./100.*self.NUMBER_OF_SAMPLES, 2**18))
        self.plot.setLabel('bottom','Time [ms]')
        self.plot.setLabel('left','ADC value')
        self.plot.getPlotItem().getViewBox().setMouseMode(pg.ViewBox.RectMode)
        self.grid.addWidget(self.plot,1,0,1,18)

        # lower plot window for frequency domain view or measurements
        self.plot2 = pg.PlotWidget()
        self.curve2 = []
        for i in range(0,self.NUMBER_OF_CHANNELS):
            myplot = self.plot2.plot()
            myplot.setPen(pg.mkPen(pg.intColor(i)))
            self.curve2.append(myplot)
        self.plot2.setLogMode(0,1)
        self.plot2.setLabel('bottom','Frequency [kHz]')
        self.plot2.setLabel('left','Power Spectrum Density')
        self.plot2.getPlotItem().getViewBox().setMouseMode(pg.ViewBox.RectMode)
        self.grid.addWidget(self.plot2,2,0,1,18)

        # input for sampling rate (free editable but with some predefined choices)
        self.grid.addWidget(QtGui.QLabel('Sampling rate [kHz]:'),3,0)
        self.samplingRate = QtGui.QComboBox()
        self.samplingRate.setEditable(True)
        self.samplingRate.addItem("100.0")
        self.samplingRate.addItem("50.0")
        self.samplingRate.addItem("10.0")
        self.samplingRate.addItem("5.0")
        self.samplingRate.addItem("1.0")
        self.grid.addWidget(self.samplingRate,3,1,1,3)
        self.samplingFrequency = 100000.

        # input for oversampling (fixed choices)
        self.grid.addWidget(QtGui.QLabel('Oversampling:'),3,4,1,2)
        self.oversampling = QtGui.QComboBox()
        self.oversampling.addItem("None", libad16.oversampling.NO_OVERSAMPLING)
        self.oversampling.addItem("2", libad16.oversampling.RATIO_2)
        self.oversampling.addItem("4", libad16.oversampling.RATIO_4)
        self.oversampling.addItem("8", libad16.oversampling.RATIO_8)
        self.oversampling.addItem("16", libad16.oversampling.RATIO_16)
        self.oversampling.addItem("32", libad16.oversampling.RATIO_32)
        self.oversampling.addItem("64", libad16.oversampling.RATIO_64)
        self.grid.addWidget(self.oversampling,3,6,1,3)
        
        # button to start and stop the DAQ
        self.startStopButton = QtGui.QPushButton('(Open device first)')
        self.startStopButton.setEnabled(False)
        self.grid.addWidget(self.startStopButton,3,15,1,3)
        self.DAQrunning = 0
        self.startStopButton.clicked.connect(self.clickStartStop)
        
        # selection of channels shown in time domain view
        self.grid.addWidget(QtGui.QLabel('Channels:'),4,0)
        self.chn = []
        for i in range(0,self.NUMBER_OF_CHANNELS):
            self.chn.append(QtGui.QCheckBox(str(i)))
            if i == 0:
                self.chn[i].setChecked(1)
            pal = QtGui.QPalette()
            pal.setColor(QtGui.QPalette.WindowText, pg.intColor(i))
            self.chn[i].setPalette(pal)
            self.grid.addWidget(self.chn[i],4,1+i)
        
        self.toggleChannelsButton = QtGui.QPushButton('toggle all')
        self.grid.addWidget(self.toggleChannelsButton,4,17)
        self.toggleChannelsButton.clicked.connect(self.clickToggleChannels)
        
        # selection of channels in lower plot
        self.grid.addWidget(QtGui.QLabel('FFT:'),5,0)
        self.fft = []
        for i in range(0,self.NUMBER_OF_CHANNELS):
            self.fft.append(QtGui.QCheckBox(str(i)))
            if i == 0:
                self.fft[i].setChecked(True)
            pal = QtGui.QPalette()
            pal.setColor(QtGui.QPalette.WindowText, pg.intColor(i))
            self.fft[i].setPalette(pal)
            self.grid.addWidget(self.fft[i],5,1+i)

        self.toggleFFTsButton = QtGui.QPushButton('toggle all')
        self.grid.addWidget(self.toggleFFTsButton,5,17)
        self.toggleFFTsButton.clicked.connect(self.clickToggleFFTs)

        # special measurements: enable performing the measurements
        self.grid.addWidget(QtGui.QLabel('Measurements:'),6,0)
        self.enableMeasurements = QtGui.QCheckBox('enable')
        self.grid.addWidget(self.enableMeasurements,6,1,1,2)

        # reset measurements (i.e. delete measured data)
        self.resetMeasurementsButton = QtGui.QPushButton('reset')
        self.grid.addWidget(self.resetMeasurementsButton,6,3,1,2)
        self.resetMeasurementsButton.clicked.connect(self.resetMeasurements)

        # select measurement shown in lower plot
        self.measurementsSelection = QtGui.QComboBox()
        self.measurementsSelection.addItem("Power Spectrum")
        self.measurementsSelection.addItem("Amplitude Spectrum")
        self.measurementsSelection.addItem("Crosstalk")
        self.measurementsSelection.addItem("Frequency Response")
        self.measurementsSelection.addItem("Amplitude over Frequency")
        self.measurementsSelection.addItem("Amplitude over Time")
        self.measurementsSelection.addItem("RMS over Frequency")
        self.measurementsSelection.addItem("Mean over Frequency")
        self.grid.addWidget(self.measurementsSelection,6,5,1,5)
        self.connect(self.measurementsSelection, QtCore.SIGNAL('activated(QString)'), self.updateLowerPlot)

        # allow locking singal channel to current detected channel
        self.lockSignalChannel = QtGui.QCheckBox('lock signal channel')
        self.grid.addWidget(self.lockSignalChannel,6,10,1,3)

        # output fields for channel mean and RMS values
        self.grid.addWidget(QtGui.QLabel('Channel Mean:'),7,0)
        self.textChannelMean = []
        for i in range(0,self.NUMBER_OF_CHANNELS):
            self.textChannelMean.append(QtGui.QLabel('n/a'))
            self.grid.addWidget(self.textChannelMean[i],7,1+i)

        self.grid.addWidget(QtGui.QLabel('Channel RMS:'),8,0)
        self.textChannelRMS = []
        for i in range(0,self.NUMBER_OF_CHANNELS):
            self.textChannelRMS.append(QtGui.QLabel('n/a'))
            self.grid.addWidget(self.textChannelRMS[i],8,1+i)

        # input fields for measurement parameters
        self.grid.addWidget(QtGui.QLabel('Measurement Parameters:'),9,0)
        self.grid.addWidget(QtGui.QLabel('Averaging window size:'),9,1,1,3)
        self.inpAverageWindow = QtGui.QSpinBox()
        self.inpAverageWindow.setRange(0,4096)
        self.inpAverageWindow.setValue(64)
        self.grid.addWidget(self.inpAverageWindow,9,4,1,2)

        # finalise the layout, add menu and status bar and show the window
        self.widget = QtGui.QWidget();
        self.widget.setLayout(self.grid);
        self.setCentralWidget(self.widget);

        self.setWindowTitle("PyQT ADC scope")
        self.resize(800, 800)
          
        self.makeMenu()

        self.statusBar = QtGui.QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage('Ready.')
        
        self.show()
        
        # initialise arrays by resetting the measurements once
        self.resetMeasurements()
        
        # start timer for polling the AD16
        self.start_time = time.time()
        self.timer = QtCore.QTimer()
        self.timer.start(1)     # 1 millisecond
        self.connect(self.timer, QtCore.SIGNAL('timeout()'), self.updateplot)
        
        # erase time domain array (variable needs to be initialised to prevent runtime error)
        self.ad16 = None
        self.signal = None

#######################################################################################################################
# create the menu
    def makeMenu(self):

        self._realDeviceAction = QtGui.QAction("Open &real device...", None)
        self.connect(self._realDeviceAction, QtCore.SIGNAL('triggered()'), self.openRealDevice)

        self._dummyDeviceAction = QtGui.QAction("Open &dummy device", None)
        self.connect(self._dummyDeviceAction, QtCore.SIGNAL('triggered()'), self.openDummyDevice)

        self._closeDeviceAction = QtGui.QAction("&Close device", None)
        self.connect(self._closeDeviceAction, QtCore.SIGNAL('triggered()'), self.closeDevice)

        self._saveAction = QtGui.QAction("Save current &time-domain data...", None)
        self.connect(self._saveAction, QtCore.SIGNAL('triggered()'), self.slotSave)

        self._saveMeasurementsAction = QtGui.QAction("Save current &measurement data...", None)
        self.connect(self._saveMeasurementsAction, QtCore.SIGNAL('triggered()'), self.saveMeasurements)

        self._loadMeasurementsAction = QtGui.QAction("&Load measurement data...", None)
        self.connect(self._loadMeasurementsAction, QtCore.SIGNAL('triggered()'), self.loadMeasurements)

        self._exitAction = QtGui.QAction("&Quit", None)
        self.connect(self._exitAction, QtCore.SIGNAL('triggered()'), self.slotClose)
        
        self._helpAction = QtGui.QAction("Help", None)
        self.connect(self._helpAction, QtCore.SIGNAL('triggered()'), self.slotHelp)
        
        menuBar = self.menuBar()
        fileMenu = menuBar.addMenu("&File")
        fileMenu.addAction(self._realDeviceAction)
        fileMenu.addAction(self._dummyDeviceAction)
        fileMenu.addAction(self._closeDeviceAction)
        fileMenu.addAction(self._saveAction)
        fileMenu.addAction(self._saveMeasurementsAction)
        fileMenu.addAction(self._loadMeasurementsAction)
        fileMenu.addAction(self._exitAction)
        
        helpMenu = menuBar.addMenu("&Help")
        helpMenu.addAction(self._helpAction)

#######################################################################################################################
# open dummy device (from menu)
    def openDummyDevice(self):
        
        if self.ad16 != None:
            QtGui.QMessageBox.information(self, 'Error', 'Device already open. Close first!')
        
        # open AD16 dummy device
        myad16 = libad16.ad16()
        myad16.open("ad16_scope_fmc25_r1224.mapp","ad16_scope_fmc25_r1224.mapp")
        myad16.setTriggerMode(libad16.trigger.PERIODIC,1)
        self.ad16 = myad16
        
        # update start button
        self.startStopButton.setText("Start")
        self.startStopButton.setEnabled(True)

#######################################################################################################################
# open real device (from menu)
    def openRealDevice(self):
        
        if self.ad16 != None:
            QtGui.QMessageBox.information(self, 'Error', 'Device already open. Close first!')
        
        # create open device dialog
        dlg = QFileDialog(self)
        dlg.setAcceptMode(QFileDialog.AcceptOpen)
        dlg.setWindowTitle('Open AD16 device')
        dlg.setViewMode( QtGui.QFileDialog.Detail )
        dlg.setDirectory("/dev")
        dlg.setNameFilters( [self.tr('Device Files (*)')] )
        
        # show dialog, open only if user did not cancel
        if dlg.exec_() :
            # file name must be converted into standard python string
            name = str(dlg.selectedFiles()[0])
        
            # open AD16 real device
            myad16 = libad16.ad16()
            try:
                myad16.open(name,"ad16_scope_fmc25_r1224.mapp")
            except:
                QtGui.QMessageBox.information(self, 'Error', 'Device cannot be opened.')
                return
            myad16.setTriggerMode(libad16.trigger.PERIODIC,1)
            self.ad16 = myad16
        
            # update start button
            self.startStopButton.setText("Start")
            self.startStopButton.setEnabled(True)

#######################################################################################################################
# close device (from menu)
    def closeDevice(self):
        
        if self.ad16 == None:
            QtGui.QMessageBox.information(self, 'Error', 'Device already closed.')
        
        # update start button
        self.startStopButton.setText("(Open device first)")
        self.startStopButton.setEnabled(False)
        
        # close device
        self.ad16.close()
        self.ad16 = None

#######################################################################################################################
# save time domain data (from menu)
    def slotSave(self):
        
        # check if data has been acquired already
        if self.signal == None:
            QtGui.QMessageBox.information(self, 'Error', 'No data to be saved. Start data acquisition first!')
            return
        
        # create file-save dialog
        dlg = QFileDialog(self)
        dlg.setAcceptMode(QFileDialog.AcceptSave)
        dlg.setWindowTitle('Save current time-domain data')
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
            f.write('# Sample frequency:              '+str(self.samplingFrequency)+'\n')
            f.write('# Number of channels:            '+str(self.NUMBER_OF_CHANNELS)+'\n')
            f.write('# Number of samples per channel: '+str(self.NUMBER_OF_SAMPLES)+'\n')
            f.write('# File created on:               '+datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')+'\n')
            f.write('# Time [ms]')
            for i in range(0,self.NUMBER_OF_CHANNELS):
                f.write(',      {0:2d}'.format(i))
            f.write('\n')
            # prepare array to be saved: transpose (to have samples in rows) and add time column
            savedata = self.signal.astype(np.float32)
            savedata = np.transpose(savedata)
            savedata = np.insert(savedata, 0, self.times, axis=1)
            # write data in coma separated values format
            fmt = []
            fmt.append('%11.3f')
            for i in range(1,self.NUMBER_OF_CHANNELS+1):
                fmt.append(' %7d')
            np.savetxt(f, savedata, fmt, ',', '\n')
            # close file
            f.close()
            # show message of success
            QtGui.QMessageBox.information(self, 'Data saved', 'Current time-domain data has been saved to file "'+name+'".')

#######################################################################################################################
# save measurements data (from menu)
    def saveMeasurements(self):
        
        # create file-save dialog
        dlg = QFileDialog(self)
        dlg.setAcceptMode(QFileDialog.AcceptSave)
        dlg.setWindowTitle('Save current measurement data')
        dlg.setViewMode( QtGui.QFileDialog.Detail )
        dlg.setNameFilters( [self.tr('numpy compressed archve (*.npz)'), self.tr('All Files (*)')] )
        dlg.setDefaultSuffix('npz')
        
        # show dialog, save only if user did not cancel
        if dlg.exec_() :
            # file name must be converted into standard python string
            name = str(dlg.selectedFiles()[0])

            np.savez_compressed(name,ratio=self.ratio, ratioCounter=self.ratioCounter,
                                frequencyResponse=self.frequencyResponse, frequencyResponseCounter=self.frequencyResponseCounter,
                                amplitudeVsTime=self.amplitudeVsTime, amplitudeVsTimeCounter=self.amplitudeVsTimeCounter,
                                rmsVsFrequency=self.rmsVsFrequency, meanVsFrequency=self.meanVsFrequency, rmsVsFrequencyCounter=self.rmsVsFrequencyCounter,
                                channelMean=self.channelMean, channelRMS=self.channelRMS,
                                powerspectrum=self.powerspectrum, freqs=self.freqs, idx=self.idx,
                                singleValues=(self.sweepCounter, self.sweepStart, self.lastFrequency, self.signalChannel,self.counterMeanAndRMS) ) 
            # show message of success
            QtGui.QMessageBox.information(self, 'Data saved', 'Measurement data has been saved to file "'+name+'".')
            
#######################################################################################################################
# load measurements data (from menu)
    def loadMeasurements(self):
        
        # create file-save dialog
        dlg = QFileDialog(self)
        dlg.setAcceptMode(QFileDialog.AcceptOpen)
        dlg.setWindowTitle('Load measurement data (current data will be overwritten!)')
        dlg.setViewMode( QtGui.QFileDialog.Detail )
        dlg.setNameFilters( [self.tr('numpy compressed archve (*.npz)'), self.tr('All Files (*)')] )
        dlg.setDefaultSuffix('npz')
        
        # show dialog, save only if user did not cancel
        if dlg.exec_() :
            # file name must be converted into standard python string
            name = str(dlg.selectedFiles()[0])

            data = np.load(name)
            self.ratio = data['ratio']
            self.ratioCounter = data['ratioCounter']
            self.frequencyResponse = data['frequencyResponse']
            self.frequencyResponseCounter = data['frequencyResponseCounter']
            self.amplitudeVsTime = data['amplitudeVsTime']
            self.amplitudeVsTimeCounter = data['amplitudeVsTimeCounter']
            self.rmsVsFrequency = data['rmsVsFrequency']
            self.meanVsFrequency = data['meanVsFrequency']
            self.rmsVsFrequencyCounter = data['rmsVsFrequencyCounter']
            self.channelMean = data['channelMean']
            self.channelRMS = data['channelRMS']
            self.powerspectrum = data['powerspectrum']
            self.freqs = data['freqs']
            self.idx = data['idx']
            (self.sweepCounter, self.sweepStart, self.lastFrequency, self.signalChannel,self.counterMeanAndRMS) = data['singleValues'] 
            # show message of success
            QtGui.QMessageBox.information(self, 'Data loaded', 'Measurement data has been loaded from file "'+name+'".')
            self.updateLowerPlot()
            
#######################################################################################################################
# close the app (from menu)
    def slotClose(self):
        self.close()

#######################################################################################################################
# show help (from menu)
    def slotHelp(self):
        QtGui.QMessageBox.information(self, "Help", "Ask Cezary!")

#######################################################################################################################
# start/stop button clicked: start/stop the DAQ
    def clickStartStop(self):
        if self.DAQrunning == 0:
            self.startStopButton.setText('Stop')
            self.DAQrunning = 1
            self.ad16.enableDaq();
        else:
            self.startStopButton.setText('Start')
            self.ad16.enableDaq(False);
            self.DAQrunning = 0
    
#######################################################################################################################
# toggle all time-domain channels (button)
    def clickToggleChannels(self):
        for i in range(0,self.NUMBER_OF_CHANNELS):
            self.chn[i].toggle()

#######################################################################################################################
# toggle all FFT channels (button)
    def clickToggleFFTs(self):
        for i in range(0,self.NUMBER_OF_CHANNELS):
            self.fft[i].toggle()

#######################################################################################################################
# reset the measurements (button)
    def resetMeasurements(self):
        self.ratio = np.zeros( (self.NUMBER_OF_CHANNELS,self.NUMBER_OF_SAMPLES/2), dtype=np.float64)
        self.ratioCounter = np.zeros( (self.NUMBER_OF_CHANNELS,self.NUMBER_OF_SAMPLES/2), dtype=np.int32)
        self.frequencyResponse = np.zeros( (self.NUMBER_OF_SAMPLES/2), dtype=np.float64)
        self.frequencyResponseCounter = np.zeros( (self.NUMBER_OF_SAMPLES/2), dtype=np.int32)
        self.amplitudeVsTime = np.zeros( self.NUMBER_OF_TIME_SLICES, dtype=np.float64)  # x-axis in 1/10 seconds
        self.amplitudeVsTimeCounter = np.zeros( self.NUMBER_OF_TIME_SLICES, dtype=np.int32)
        self.rmsVsFrequency = np.zeros( (self.NUMBER_OF_SAMPLES/2), dtype=np.float64)
        self.meanVsFrequency = np.zeros( (self.NUMBER_OF_SAMPLES/2), dtype=np.float64)
        self.rmsVsFrequencyCounter = np.zeros( (self.NUMBER_OF_SAMPLES/2), dtype=np.int32)
        self.channelMean =  np.zeros(self.NUMBER_OF_CHANNELS, dtype=np.float64)
        self.channelRMS =  np.zeros(self.NUMBER_OF_CHANNELS, dtype=np.float64)
        self.sweepCounter = 0
        self.sweepStart = time.time()
        self.lastFrequency = -999
        self.signalChannel = -1
        self.counterMeanAndRMS = 0
        for k in range(0,self.NUMBER_OF_SAMPLES/2):
            self.frequencyResponse[k] = np.NaN
            self.rmsVsFrequency[k] = np.NaN
            self.meanVsFrequency[k] = np.NaN
            for i in range(0,self.NUMBER_OF_CHANNELS):
                self.ratio[i][k] = np.NaN
        for k in range(0,self.NUMBER_OF_TIME_SLICES):
            self.amplitudeVsTime[k] = np.NaN
        for k in range(0,self.NUMBER_OF_CHANNELS):
            self.channelMean[k] = np.NaN
            self.channelRMS[k] = np.NaN

#######################################################################################################################
# periodic timer: poll the AD16 and update plots and measurements, if new data present
    def updateplot(self):
        
        # check if AD16 is opened already
        if self.ad16 == None:
            return
        
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
        self.signal = np.zeros( (self.NUMBER_OF_CHANNELS,self.NUMBER_OF_SAMPLES), dtype=np.int32)
        for i in range(0,self.NUMBER_OF_CHANNELS):
            self.ad16.getChannelData(i, self.signal[i])

        # compute x-axis values
        self.times = np.linspace(0., self.TIME_STEP*float(self.NUMBER_OF_SAMPLES), num=self.NUMBER_OF_SAMPLES, endpoint=False)
        
        # take time for obtaining the data
        time_get_image = time.time()

        # calculate fourier transform
        self.powerspectrum = np.zeros( (self.NUMBER_OF_CHANNELS,self.NUMBER_OF_SAMPLES), dtype=np.float64)
        for i in range(0,self.NUMBER_OF_CHANNELS):
            signal = self.signal[i] * np.hamming(self.NUMBER_OF_SAMPLES)
            self.powerspectrum[i] = np.abs(np.fft.fft(signal))**2 / self.NUMBER_OF_SAMPLES 
        
        self.freqs = np.fft.fftfreq(self.signal[0].size, self.TIME_STEP)
        self.idx   = np.argsort(self.freqs)

        # perform measurements, if enabled
        if self.enableMeasurements.isChecked():
            self.performMeasurements()
            
        # take time for the calculations
        time_calc_image = time.time()
        
        # plot the signal
        for i in range(0,self.NUMBER_OF_CHANNELS):
            if self.chn[i].isChecked():
                self.curve[i].setData(self.times,self.signal[i])
            else:
                self.curve[i].clear()
        
        self.updateLowerPlot()

        # take time for plotting
        time_plot_image = time.time()
        
        # What the hell is going on ... put info into statusbar
        f = self.freqs[self.lastFrequency]
        if self.signalChannel == -1:
            signalChannelText = '(not yet determined)'
            f = np.NaN
        elif self.signalChannel == -2:
            signalChannelText = '*** CHANGED DURING MEASUREMENT ***'
            f = np.NaN
        else:
            signalChannelText = str(self.signalChannel)
        self.statusBar.showMessage(
            'Block: %i\
        \tfps: %.4fpps\
        \tAcq: %.3fms\
        \tCalc: %.3fms\
        \tPlot: %.3fms\
        \tSignal Channel: %s\
        \tFrequency: %.3f kHz\
        \tSweeps: %d'
            %(self.counter, fps,
            1000 * (time_get_image - self.start_time),
            1000 * (time_calc_image - time_get_image),
            1000 * (time_plot_image - time_calc_image),
             signalChannelText, f, self.sweepCounter))

        # update sample rate and number of samples
        self.samplingFrequency = float(self.samplingRate.currentText())*1000.
        self.oversamplingRatio = libad16.oversampling(self.oversampling.currentIndex())
        try:
            self.ad16.setSamplingRate(self.samplingFrequency, self.oversamplingRatio)
        except libad16.ad16Exception, e:
            QtGui.QMessageBox.information(self, 'Error', e.what)
            self.clickStartStop()
        
        # get actual sample rate (after rounding to next-possible value) and update input field
        self.samplingFrequency = self.ad16.getSamplingRate()
        self.samplingRate.lineEdit().setText( str(self.samplingFrequency/1000) )
        
        # compute trigger frequency for continuous trigger mode
        self.ad16.setTriggerMode(libad16.trigger.PERIODIC, self.samplingFrequency/65536.)
        
        # update timing information used for calculations and display
        self.FFT_INTEGRATION_HALF_WINDOW = np.int( np.round(250./(self.samplingFrequency/self.NUMBER_OF_SAMPLES)) )  # 250 Hz
        self.TIME_STEP = 1./self.samplingFrequency * 1000.

        
#######################################################################################################################
# compute signal power from FFT
    def getFFTpower(self,frequencyIndex,spectrum):
        integral = 0
        for f in range(frequencyIndex-self.FFT_INTEGRATION_HALF_WINDOW,frequencyIndex+self.FFT_INTEGRATION_HALF_WINDOW):
            integral += spectrum[f]
        return integral

#######################################################################################################################
# perform measurements
    def performMeasurements(self):
        
        # window width to average measurement data over
        avgWindow = self.inpAverageWindow.value()
        
        # Mean and RMS of all channels
        for i in range(0,self.NUMBER_OF_CHANNELS):
            mean = np.mean(self.signal[i])
            rms = np.std(self.signal[i])
            if self.counterMeanAndRMS == 0:
                self.channelMean[i] = mean
                self.channelRMS[i] = rms
            else:
                self.channelMean[i] = (self.channelMean[i]*self.counterMeanAndRMS + mean) / (self.counterMeanAndRMS+1) 
                self.channelRMS[i] =(self.channelRMS[i]*self.counterMeanAndRMS + rms) / (self.counterMeanAndRMS+1) 
        self.counterMeanAndRMS += 1

        # obtain frequency and power of maximum signal in each channel
        chnSignalFrequency = np.zeros(self.NUMBER_OF_CHANNELS, dtype=np.int)
        chnSignalPower = np.zeros(self.NUMBER_OF_CHANNELS, dtype=np.float64)
        for i in range(0,self.NUMBER_OF_CHANNELS):
            chnSignalFrequency[i] = np.argmax(self.powerspectrum[i][10:self.NUMBER_OF_SAMPLES/2])+10  # ignore first 10 FFT samples to get rid of DC part
            chnSignalPower[i] = self.getFFTpower(chnSignalFrequency[i],self.powerspectrum[i])

        # find signal channel (channel with maximum signal power)
        maxSignalPower = np.amax(chnSignalPower)
        minSignalPower = 1000.*np.amin(chnSignalPower)            # 1000 times more than minimum channel power ( = noise, hopefully) STRANGE UNITS!!!
        # require signal power over threshold, otherwise keep signal channel unchanged. Also keep it unchanged if signal channel is locked
        if(not (self.signalChannel >=0 and self.lockSignalChannel.isChecked() ) and maxSignalPower > minSignalPower):
            theSignalChannel = np.argmax(chnSignalPower)
        else:
            theSignalChannel = self.signalChannel
        theSignalPower = chnSignalPower[theSignalChannel]
        if theSignalPower <= minSignalPower:
            theSignalFrequency = -2*avgWindow   # out of range, will be ignored
        else:
            theSignalFrequency = chnSignalFrequency[theSignalChannel]

        # check for signal channel change
        if self.signalChannel != theSignalChannel:
            if self.signalChannel == -1:
                self.signalChannel = theSignalChannel
            elif self.signalChannel != -2:
                self.signalChannel = -2     # will give also a warning in the status bar
                QtGui.QMessageBox.information(self, "Warning", "Signal channel changed. Resetting measurements recommended!")
        
        # divide the signal power of all channels through signal power of the signal channel
        for i in range(0,self.NUMBER_OF_CHANNELS):
            power = self.getFFTpower(theSignalFrequency,self.powerspectrum[i])
            ratio = power/theSignalPower
            for f in range(theSignalFrequency-avgWindow,theSignalFrequency+avgWindow):
                if theSignalFrequency < 0 or theSignalFrequency > self.NUMBER_OF_SAMPLES/2:
                    continue
                if self.ratioCounter[i][f] == 0:
                    self.ratio[i][f] = ratio
                else:
                    self.ratio[i][f] = (self.ratio[i][f]*self.ratioCounter[i][f] + ratio) / (self.ratioCounter[i][f]+1)
                self.ratioCounter[i][f] += 1
        
        # measure frequency response (using information determined in the cross-talk measurement)
        for f in range(theSignalFrequency-avgWindow,theSignalFrequency+avgWindow):
            if theSignalFrequency < 0 or theSignalFrequency > self.NUMBER_OF_SAMPLES/2:
                continue
            if self.frequencyResponseCounter[f] == 0:
                self.frequencyResponse[f] = theSignalPower
            else:
                self.frequencyResponse[f] = (self.frequencyResponse[f]*self.frequencyResponseCounter[f] + theSignalPower) / (self.frequencyResponseCounter[f]+1)
            self.frequencyResponseCounter[f] += 1
        
        # signal amplitude vs. time
        timeIndex = np.round(10.*(time.time()-self.sweepStart))  # in 1/10 seconds
        if timeIndex < self.NUMBER_OF_TIME_SLICES:
            if self.amplitudeVsTimeCounter[timeIndex] == 0:
                self.amplitudeVsTime[timeIndex] = np.sqrt(theSignalPower)
            else:
                self.amplitudeVsTime[timeIndex] = (self.amplitudeVsTime[timeIndex]*self.amplitudeVsTimeCounter[timeIndex] + np.sqrt(theSignalPower)) / (self.amplitudeVsTimeCounter[timeIndex]+1)
                self.amplitudeVsTimeCounter[timeIndex] += 1
        
        # rms and mean vs frequency
        signalRms = np.std(self.signal[theSignalChannel])
        signalMean = np.mean(self.signal[theSignalChannel])
        for f in range(theSignalFrequency-avgWindow,theSignalFrequency+avgWindow):
            if theSignalFrequency < 0 or theSignalFrequency > self.NUMBER_OF_SAMPLES/2:
                continue
            if self.rmsVsFrequencyCounter[f] == 0:
                self.rmsVsFrequency[f] = signalRms
                self.meanVsFrequency[f] = signalMean
            else:
                self.rmsVsFrequency[f] = (self.rmsVsFrequency[f]*self.rmsVsFrequencyCounter[f] + signalRms) / (self.rmsVsFrequencyCounter[f]+1)
                self.meanVsFrequency[f] = (self.meanVsFrequency[f]*self.rmsVsFrequencyCounter[f] + signalMean) / (self.rmsVsFrequencyCounter[f]+1)
            self.rmsVsFrequencyCounter[f] += 1
        
        # check if frequency sweep completed (i.e. signal frequency lower than last)
        if theSignalFrequency > 0:
            if theSignalFrequency < self.lastFrequency*0.9:
                self.sweepCounter += 1
                self.sweepStart = time.time()
            self.lastFrequency = theSignalFrequency

#######################################################################################################################
# update the lower plot (i.e. show user-selected measurement)
    def updateLowerPlot(self):
        
        # plot the power spectrum
        if self.measurementsSelection.currentIndex() == 0:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','Power')
            for i in range(0,self.NUMBER_OF_CHANNELS):
                if self.fft[i].isChecked():
                    self.curve2[i].setData(self.freqs[self.idx[self.NUMBER_OF_SAMPLES/2:self.NUMBER_OF_SAMPLES-1]],
                                           self.powerspectrum[i][self.idx[self.NUMBER_OF_SAMPLES/2:self.NUMBER_OF_SAMPLES-1]],
                                           connect="finite")
                else:
                    self.curve2[i].clear()
        # plot the amplitude spectrum
        elif self.measurementsSelection.currentIndex() == 1:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','Amplitude')
            for i in range(0,self.NUMBER_OF_CHANNELS):
                if self.fft[i].isChecked():
                    self.curve2[i].setData(self.freqs[self.idx[self.NUMBER_OF_SAMPLES/2:self.NUMBER_OF_SAMPLES-1]],
                                           np.sqrt(self.powerspectrum[i][self.idx[self.NUMBER_OF_SAMPLES/2:self.NUMBER_OF_SAMPLES-1]]),
                                           connect="finite")
                else:
                    self.curve2[i].clear()
        # plot cross talk
        elif self.measurementsSelection.currentIndex() == 2:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','Crosstalk')
            for i in range(0,self.NUMBER_OF_CHANNELS):
                if self.fft[i].isChecked():
                    self.curve2[i].setData(self.freqs[self.idx[self.NUMBER_OF_SAMPLES/2:self.NUMBER_OF_SAMPLES]],self.ratio[i],
                                           connect="finite")
                else:
                    self.curve2[i].clear()
        # plot frequency response
        elif self.measurementsSelection.currentIndex() == 3:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','Response')
            for i in range(0,self.NUMBER_OF_CHANNELS):
                if i == self.signalChannel:
                    respMax = np.nanmax(self.frequencyResponse[1024:self.NUMBER_OF_SAMPLES/8])
                    if respMax != respMax:
                        respMax = 1
                    self.curve2[i].setData(self.freqs[self.idx[self.NUMBER_OF_SAMPLES/2:self.NUMBER_OF_SAMPLES]],
                                           self.frequencyResponse / respMax,
                                           connect="finite")
                else:
                    self.curve2[i].clear()
        # plot amplitude over frequency
        elif self.measurementsSelection.currentIndex() == 4:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','Amplitude')
            for i in range(0,self.NUMBER_OF_CHANNELS):
                if i == self.signalChannel:
                    self.curve2[i].setData(self.freqs[self.idx[self.NUMBER_OF_SAMPLES/2:self.NUMBER_OF_SAMPLES]],
                                           np.sqrt(self.frequencyResponse),
                                           connect="finite")
                else:
                    self.curve2[i].clear()
        # plot amplitude vs time
        elif self.measurementsSelection.currentIndex() == 5:
            self.plot2.setLabel('bottom','Time [0.1s]')
            self.plot2.setLabel('left','Amplitude')
            for i in range(0,self.NUMBER_OF_CHANNELS):
                if i == self.signalChannel:
                    self.curve2[i].setData(self.amplitudeVsTime,connect="finite")
                else:
                    self.curve2[i].clear()
        # plot RMS over frequency
        elif self.measurementsSelection.currentIndex() == 6:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','RMS')
            for i in range(0,self.NUMBER_OF_CHANNELS):
                if i == self.signalChannel:
                    self.curve2[i].setData(self.freqs[self.idx[self.NUMBER_OF_SAMPLES/2:self.NUMBER_OF_SAMPLES]],
                                           self.rmsVsFrequency,
                                           connect="finite")
                else:
                    self.curve2[i].clear()
        # plot Mean over frequency
        elif self.measurementsSelection.currentIndex() == 7:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','MEAN')
            for i in range(0,self.NUMBER_OF_CHANNELS):
                if i == self.signalChannel:
                    self.curve2[i].setData(self.freqs[self.idx[self.NUMBER_OF_SAMPLES/2:self.NUMBER_OF_SAMPLES]],
                                           self.meanVsFrequency,
                                           connect="finite")
                else:
                    self.curve2[i].clear()
        else:
            QtGui.QMessageBox.information(self, "Error", "Unknown measurement type selected: "+str(self.measurementsSelection.currentIndex()))
            sys.exit(1)

        # update mean and RMS values
        for i in range(0,self.NUMBER_OF_CHANNELS):
            self.textChannelMean[i].setText('%4.2f'%(self.channelMean[i]))
            self.textChannelRMS[i].setText('%4.2f'%(self.channelRMS[i]))

#######################################################################################################################
# main function
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    mainWin = MainWindow()
    code = app.exec_()
    if mainWin.ad16 != None:
        mainWin.ad16.enableDaq(False)
        mainWin.ad16.close()
    sys.exit(code)
