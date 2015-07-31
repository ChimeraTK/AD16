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

# some constants: number of channels is fixed in hardware
NUMBER_OF_CHANNELS = 16
# the number of samples (per channel) is currently fixed, this is a firmware limitation and will change in future
NUMBER_OF_SAMPLES = 65536
# maximum number of 1/10 seconds to display in amplitude vs. time view
NUMBER_OF_TIME_SLICES = 6000 
# to determine the amplitude, the FFT is integrated over a window around the peak: (-FFT_INTEGRATION_HALF_WINDOW:FFT_INTEGRATION_HALF_WINDOW)
FFT_INTEGRATION_HALF_WINDOW = np.int( np.round(250./(79100./NUMBER_OF_SAMPLES)) )  # 250 Hz
# time step in milliseconds (inverse sampling rate)
TIME_STEP = 1./79.1


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
        self.plot.getPlotItem().getViewBox().setMouseMode(pg.ViewBox.RectMode)
        self.grid.addWidget(self.plot,1,0,1,18)

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
        self.plot2.getPlotItem().getViewBox().setMouseMode(pg.ViewBox.RectMode)
        self.grid.addWidget(self.plot2,2,0,1,18)
        
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
        self.grid.addWidget(self.startStopButton,3,15,1,3)
        self.DAQrunning = 0
        self.startStopButton.clicked.connect(self.clickStartStop)
        
        
        self.text3 = QtGui.QLabel('Channels:')
        self.grid.addWidget(self.text3,5,0)
        self.chn = []
        for i in range(0,NUMBER_OF_CHANNELS):
            self.chn.append(QtGui.QCheckBox(str(i)))
            if i == 0:
                self.chn[i].setChecked(1)
            pal = QtGui.QPalette()
            pal.setColor(QtGui.QPalette.WindowText, pg.intColor(i))
            self.chn[i].setPalette(pal)
            self.grid.addWidget(self.chn[i],5,1+i)
        
        self.toggleChannelsButton = QtGui.QPushButton('toggle all')
        self.grid.addWidget(self.toggleChannelsButton,5,17)
        self.toggleChannelsButton.clicked.connect(self.clickToggleChannels)
        
        self.text4 = QtGui.QLabel('FFT:')
        self.grid.addWidget(self.text4,6,0)
        self.fft = []
        for i in range(0,NUMBER_OF_CHANNELS):
            self.fft.append(QtGui.QCheckBox(str(i)))
            if i == 0:
                self.fft[i].setChecked(True)
            pal = QtGui.QPalette()
            pal.setColor(QtGui.QPalette.WindowText, pg.intColor(i))
            self.fft[i].setPalette(pal)
            self.grid.addWidget(self.fft[i],6,1+i)

        self.toggleFFTsButton = QtGui.QPushButton('toggle all')
        self.grid.addWidget(self.toggleFFTsButton,6,17)
        self.toggleFFTsButton.clicked.connect(self.clickToggleFFTs)
        
        self.text5 = QtGui.QLabel('Measurements:')
        self.grid.addWidget(self.text5,7,0)
        self.enableMeasurements = QtGui.QCheckBox('enable')
        self.grid.addWidget(self.enableMeasurements,7,1,1,2)

        self.resetMeasurementsButton = QtGui.QPushButton('reset')
        self.grid.addWidget(self.resetMeasurementsButton,7,3,1,2)
        self.resetMeasurementsButton.clicked.connect(self.resetMeasurements)

        self.measurementsSelection = QtGui.QComboBox()
        self.measurementsSelection.addItem("Power Spectrum")
        self.measurementsSelection.addItem("Amplitude Spectrum")
        self.measurementsSelection.addItem("Crosstalk")
        self.measurementsSelection.addItem("Frequency Response")
        self.measurementsSelection.addItem("Amplitude over Frequency")
        self.measurementsSelection.addItem("Amplitude over Time")
        self.measurementsSelection.addItem("RMS over Frequency")
        self.measurementsSelection.addItem("Mean over Frequency")
        self.grid.addWidget(self.measurementsSelection,7,5,1,5)
        self.connect(self.measurementsSelection, QtCore.SIGNAL('activated(QString)'), self.updateLowerPlot)

        self.lockSignalChannel = QtGui.QCheckBox('lock signal channel')
        self.grid.addWidget(self.lockSignalChannel,7,10,1,3)

        self.grid.addWidget(QtGui.QLabel('Channel Mean:'),8,0)
        self.textChannelMean = []
        for i in range(0,NUMBER_OF_CHANNELS):
            self.textChannelMean.append(QtGui.QLabel('n/a'))
            self.grid.addWidget(self.textChannelMean[i],8,1+i)

        self.grid.addWidget(QtGui.QLabel('Channel RMS:'),9,0)
        self.textChannelRMS = []
        for i in range(0,NUMBER_OF_CHANNELS):
            self.textChannelRMS.append(QtGui.QLabel('n/a'))
            self.grid.addWidget(self.textChannelRMS[i],9,1+i)

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
        
        # initialise arrays by resetting the measurements once
        self.resetMeasurements()
        
        # open AD16 and start first conversion
        self.ad16 = libad16.ad16()
        if os.path.exists('/dev/llrfutcs5'):
            self.ad16.open("/dev/llrfutcs5","ad16dummy.map")
        else:
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
        self._saveAction = QtGui.QAction("&Save current time-domain data...", None)
        self.connect(self._saveAction, QtCore.SIGNAL('triggered()'), self.slotSave)

        self._saveMeasurementsAction = QtGui.QAction("&Save current measurement data...", None)
        self.connect(self._saveMeasurementsAction, QtCore.SIGNAL('triggered()'), self.saveMeasurements)

        self._loadMeasurementsAction = QtGui.QAction("&Load current measurement data...", None)
        self.connect(self._loadMeasurementsAction, QtCore.SIGNAL('triggered()'), self.loadMeasurements)

        self._exitAction = QtGui.QAction("&Close", None)
        self.connect(self._exitAction, QtCore.SIGNAL('triggered()'), self.slotClose)
        
        self._helpAction = QtGui.QAction("Help", None)
        self.connect(self._helpAction, QtCore.SIGNAL('triggered()'), self.slotHelp)
        
        menuBar = self.menuBar()
        fileMenu = menuBar.addMenu("&File")
        fileMenu.addAction(self._saveAction)
        fileMenu.addAction(self._saveMeasurementsAction)
        fileMenu.addAction(self._loadMeasurementsAction)
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
            

    def loadMeasurements(self):
        
        # create file-save dialog
        dlg = QFileDialog(self)
        dlg.setAcceptMode(QFileDialog.AcceptOpen)
        dlg.setWindowTitle('Load measurement data (current measurement data will be overwritten!)')
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
            

    def slotClose(self):
        self.close()

    def slotHelp(self):
        QtGui.QMessageBox.information(self, "Help", "Ask Cezary!")
        
    def resetMeasurements(self):
        self.ratio = np.zeros( (NUMBER_OF_CHANNELS,NUMBER_OF_SAMPLES/2), dtype=np.float64)
        self.ratioCounter = np.zeros( (NUMBER_OF_CHANNELS,NUMBER_OF_SAMPLES/2), dtype=np.int32)
        self.frequencyResponse = np.zeros( (NUMBER_OF_SAMPLES/2), dtype=np.float64)
        self.frequencyResponseCounter = np.zeros( (NUMBER_OF_SAMPLES/2), dtype=np.int32)
        self.amplitudeVsTime = np.zeros( NUMBER_OF_TIME_SLICES, dtype=np.float64)  # x-axis in 1/10 seconds
        self.amplitudeVsTimeCounter = np.zeros( NUMBER_OF_TIME_SLICES, dtype=np.int32)
        self.rmsVsFrequency = np.zeros( (NUMBER_OF_SAMPLES/2), dtype=np.float64)
        self.meanVsFrequency = np.zeros( (NUMBER_OF_SAMPLES/2), dtype=np.float64)
        self.rmsVsFrequencyCounter = np.zeros( (NUMBER_OF_SAMPLES/2), dtype=np.int32)
        self.channelMean =  np.zeros(NUMBER_OF_CHANNELS, dtype=np.float64)
        self.channelRMS =  np.zeros(NUMBER_OF_CHANNELS, dtype=np.float64)
        self.sweepCounter = 0
        self.sweepStart = time.time()
        self.lastFrequency = -999
        self.signalChannel = -1
        self.counterMeanAndRMS = 0
        for k in range(0,NUMBER_OF_SAMPLES/2):
            self.frequencyResponse[k] = np.NaN
            self.rmsVsFrequency[k] = np.NaN
            self.meanVsFrequency[k] = np.NaN
            for i in range(0,NUMBER_OF_CHANNELS):
                self.ratio[i][k] = np.NaN
        for k in range(0,NUMBER_OF_TIME_SLICES):
            self.amplitudeVsTime[k] = np.NaN
        for k in range(0,NUMBER_OF_CHANNELS):
            self.channelMean[k] = np.NaN
            self.channelRMS[k] = np.NaN

        
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
        self.times = np.linspace(0., TIME_STEP*float(NUMBER_OF_SAMPLES), num=NUMBER_OF_SAMPLES, endpoint=False)
        
        # trigger next conversion
        self.ad16.startConversion()
        
        # take time for obtaining the data
        time_get_image = time.time()

        # calculate fourier transform
        self.powerspectrum = np.zeros( (NUMBER_OF_CHANNELS,NUMBER_OF_SAMPLES), dtype=np.float64)
        for i in range(0,NUMBER_OF_CHANNELS):
            signal = self.signal[i] * np.hamming(NUMBER_OF_SAMPLES)
            self.powerspectrum[i] = np.abs(np.fft.fft(signal))**2 / NUMBER_OF_SAMPLES 
        
        self.freqs = np.fft.fftfreq(self.signal[0].size, TIME_STEP)
        self.idx   = np.argsort(self.freqs)

        # perform measurements, if enabled
        if self.enableMeasurements.isChecked():
            self.performMeasurements()
            
        # take time for the calculations
        time_calc_image = time.time()
        
        # plot the signal
        for i in range(0,NUMBER_OF_CHANNELS):
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
        #val = self.samplingRate.itemData(self.samplingRate.currentIndex()).toPyObject()
        #self.ad16.setSamplingRate(val)
        #self.ad16.setSamplesPerBlock( self.samples.value() )
        
        
    def getFFTpower(self,frequencyIndex,spectrum):
        integral = 0
        for f in range(frequencyIndex-FFT_INTEGRATION_HALF_WINDOW,frequencyIndex+FFT_INTEGRATION_HALF_WINDOW):
            integral += spectrum[f]
        return integral

    def performMeasurements(self):
        
        # Mean and RMS of all channels
        for i in range(0,NUMBER_OF_CHANNELS):
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
        chnSignalFrequency = np.zeros(NUMBER_OF_CHANNELS, dtype=np.int)
        chnSignalPower = np.zeros(NUMBER_OF_CHANNELS, dtype=np.float64)
        for i in range(0,NUMBER_OF_CHANNELS):
            chnSignalFrequency[i] = np.argmax(self.powerspectrum[i][10:NUMBER_OF_SAMPLES/2])+10  # ignore first 10 FFT samples to get rid of DC part
            chnSignalPower[i] = self.getFFTpower(chnSignalFrequency[i],self.powerspectrum[i])

        # find signal channel (channel with maximum signal power)
        maxSignalPower = np.amax(chnSignalPower)
        minSignalPower = 10.*np.amin(chnSignalPower)            # 10 times more than minimum channel power ( = noise, hopefully)
        # require signal power over threshold, otherwise keep signal channel unchanged. Also keep it unchanged if signal channel is locked
        if(not self.lockSignalChannel.isChecked() and maxSignalPower > minSignalPower):
            theSignalChannel = np.argmax(chnSignalPower)
        else:
            theSignalChannel = self.signalChannel
        theSignalPower = chnSignalPower[theSignalChannel]
        theSignalFrequency = chnSignalFrequency[theSignalChannel]

        # check for signal channel change
        if self.signalChannel != theSignalChannel:
            if self.signalChannel == -1:
                self.signalChannel = theSignalChannel
            elif self.signalChannel != -2:
                self.signalChannel = -2     # will give also a warning in the status bar
                QtGui.QMessageBox.information(self, "Warning", "Signal channel changed. Resetting measurements recommended!")
        
        # determine frequency of input signal (maximuim of fft)
        #theSignalFrequency = np.abs(self.idx[np.argmax(self.powerspectrum[theSignalChannel])] - NUMBER_OF_SAMPLES/2)
        
        # determine power of input signal (integrated FFT around peak)
        #theSignalPower = self.getFFTpower(theSignalFrequency,self.powerspectrum[theSignalChannel])
        
        # divide the signal power of all channels through signal power of the signal channel
        for i in range(0,NUMBER_OF_CHANNELS):
            power = self.getFFTpower(theSignalFrequency,self.powerspectrum[i])
            ratio = power/theSignalPower
            for f in range(theSignalFrequency-512,theSignalFrequency+512):
                if theSignalFrequency < 0 or theSignalFrequency > NUMBER_OF_SAMPLES/2:
                    continue
                if self.ratioCounter[i][f] == 0:
                    self.ratio[i][f] = ratio
                else:
                    self.ratio[i][f] = (self.ratio[i][f]*self.ratioCounter[i][f] + ratio) / (self.ratioCounter[i][f]+1)
                self.ratioCounter[i][f] += 1
        
        # measure frequency response (using information determined in the cross-talk measurement)
        for f in range(theSignalFrequency-512,theSignalFrequency+512):
            if theSignalFrequency < 0 or theSignalFrequency > NUMBER_OF_SAMPLES/2:
                continue
            if self.frequencyResponseCounter[f] == 0:
                self.frequencyResponse[f] = theSignalPower
            else:
                self.frequencyResponse[f] = (self.frequencyResponse[f]*self.frequencyResponseCounter[f] + theSignalPower) / (self.frequencyResponseCounter[f]+1)
            self.frequencyResponseCounter[f] += 1
        
        # check if frequency sweep completed (i.e. signal frequency lower than last)
        if theSignalFrequency < self.lastFrequency*0.9:
            self.sweepCounter += 1
            self.sweepStart = time.time()
        self.lastFrequency = theSignalFrequency
        
        # signal amplitude vs. time
        timeIndex = np.round(10.*(time.time()-self.sweepStart))  # in 1/10 seconds
        if timeIndex < NUMBER_OF_TIME_SLICES:
            if self.amplitudeVsTimeCounter[timeIndex] == 0:
                self.amplitudeVsTime[timeIndex] = np.sqrt(theSignalPower)
            else:
                self.amplitudeVsTime[timeIndex] = (self.amplitudeVsTime[timeIndex]*self.amplitudeVsTimeCounter[timeIndex] + np.sqrt(theSignalPower)) / (self.amplitudeVsTimeCounter[timeIndex]+1)
                self.amplitudeVsTimeCounter[timeIndex] += 1
        
        # rms and mean vs frequency
        signalRms = np.std(self.signal[theSignalChannel])
        signalMean = np.mean(self.signal[theSignalChannel])
        for f in range(theSignalFrequency-512,theSignalFrequency+512):
            if theSignalFrequency < 0 or theSignalFrequency > NUMBER_OF_SAMPLES/2:
                continue
            if self.rmsVsFrequencyCounter[f] == 0:
                self.rmsVsFrequency[f] = signalRms
                self.meanVsFrequency[f] = signalMean
            else:
                self.rmsVsFrequency[f] = (self.rmsVsFrequency[f]*self.rmsVsFrequencyCounter[f] + signalRms) / (self.rmsVsFrequencyCounter[f]+1)
                self.meanVsFrequency[f] = (self.meanVsFrequency[f]*self.rmsVsFrequencyCounter[f] + signalMean) / (self.rmsVsFrequencyCounter[f]+1)
            self.rmsVsFrequencyCounter[f] += 1


    def updateLowerPlot(self):
        
        # plot the power spectrum
        if self.measurementsSelection.currentIndex() == 0:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','Power')
            for i in range(0,NUMBER_OF_CHANNELS):
                if self.fft[i].isChecked():
                    self.curve2[i].setData(self.freqs[self.idx[NUMBER_OF_SAMPLES/2:NUMBER_OF_SAMPLES-1]],self.powerspectrum[i][self.idx[NUMBER_OF_SAMPLES/2:NUMBER_OF_SAMPLES-1]])
                else:
                    self.curve2[i].clear()
        # plot the amplitude spectrum
        elif self.measurementsSelection.currentIndex() == 1:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','Amplitude')
            for i in range(0,NUMBER_OF_CHANNELS):
                if self.fft[i].isChecked():
                    self.curve2[i].setData(self.freqs[self.idx[NUMBER_OF_SAMPLES/2:NUMBER_OF_SAMPLES-1]],np.sqrt(self.powerspectrum[i][self.idx[NUMBER_OF_SAMPLES/2:NUMBER_OF_SAMPLES-1]]))
                else:
                    self.curve2[i].clear()
        # plot cross talk
        elif self.measurementsSelection.currentIndex() == 2:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','Crosstalk')
            for i in range(0,NUMBER_OF_CHANNELS):
                if self.fft[i].isChecked():
                    self.curve2[i].setData(self.freqs[self.idx[NUMBER_OF_SAMPLES/2:NUMBER_OF_SAMPLES]],self.ratio[i])
                else:
                    self.curve2[i].clear()
        # plot frequency response
        elif self.measurementsSelection.currentIndex() == 3:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','Response')
            for i in range(0,NUMBER_OF_CHANNELS):
                if i == self.signalChannel:
                    respMax = np.nanmax(self.frequencyResponse[1024:NUMBER_OF_SAMPLES/8])
                    if respMax != respMax:
                        respMax = 1
                    self.curve2[i].setData(self.freqs[self.idx[NUMBER_OF_SAMPLES/2:NUMBER_OF_SAMPLES]],self.frequencyResponse / respMax)
                else:
                    self.curve2[i].clear()
        # plot amplitude over frequency
        elif self.measurementsSelection.currentIndex() == 4:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','Amplitude')
            for i in range(0,NUMBER_OF_CHANNELS):
                if i == self.signalChannel:
                    self.curve2[i].setData(self.freqs[self.idx[NUMBER_OF_SAMPLES/2:NUMBER_OF_SAMPLES]],np.sqrt(self.frequencyResponse))
                else:
                    self.curve2[i].clear()
        # plot amplitude vs time
        elif self.measurementsSelection.currentIndex() == 5:
            self.plot2.setLabel('bottom','Time [0.1s]')
            self.plot2.setLabel('left','Amplitude')
            for i in range(0,NUMBER_OF_CHANNELS):
                if i == self.signalChannel:
                    self.curve2[i].setData(self.amplitudeVsTime)
                else:
                    self.curve2[i].clear()
        # plot RMS over frequency
        elif self.measurementsSelection.currentIndex() == 6:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','RMS')
            for i in range(0,NUMBER_OF_CHANNELS):
                if i == self.signalChannel:
                    self.curve2[i].setData(self.freqs[self.idx[NUMBER_OF_SAMPLES/2:NUMBER_OF_SAMPLES]],self.rmsVsFrequency)
                else:
                    self.curve2[i].clear()
        # plot Mean over frequency
        elif self.measurementsSelection.currentIndex() == 7:
            self.plot2.setLabel('bottom','Frequency [kHz]')
            self.plot2.setLabel('left','MEAN')
            for i in range(0,NUMBER_OF_CHANNELS):
                if i == self.signalChannel:
                    self.curve2[i].setData(self.freqs[self.idx[NUMBER_OF_SAMPLES/2:NUMBER_OF_SAMPLES]],self.meanVsFrequency)
                else:
                    self.curve2[i].clear()
        else:
            QtGui.QMessageBox.information(self, "Error", "Unknown measurement type selected: "+str(self.measurementsSelection.currentIndex()))
            sys.exit(1)

        # update mean and RMS values
        for i in range(0,NUMBER_OF_CHANNELS):
            self.textChannelMean[i].setText('%4.2f'%(self.channelMean[i]))
            self.textChannelRMS[i].setText('%4.2f'%(self.channelRMS[i]))

       
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    mainWin = MainWindow()
    sys.exit(app.exec_())
