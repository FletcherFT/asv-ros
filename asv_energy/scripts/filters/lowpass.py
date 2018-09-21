import numpy as np
from scipy.signal import butter, lfilter, freqz

class LowPass:
    def __init__(self,buffsize,cutoff,fs,order=5):
        self._cutoff = cutoff
        self._fs = fs
        self._order = order
        self._buff = np.array([])
        self._buffsize = buffsize
        nyq = 0.5*fs
        normal_cutoff = cutoff /nyq
        self._b,self._a = butter(order, normal_cutoff, btype='low',analog=False)
        

    def update(self,datapoint):
        self._buff=np.append(self._buff,datapoint)
        if len(self._buff)>self._buffsize:
            self._buff=np.delete(self._buff,0)
        if len(self._buff)<2:
            return datapoint
        else:
            vals = lfilter(self._b,self._a,self._buff)
            return vals[-1]
