
import pandas as pd
import matplotlib.pyplot as plt

import numpy as np
from scipy.signal import hann, fftconvolve
from scipy.fftpack import fft, ifft

from scipy.signal import savgol_filter

def PTstepcalc(setPointVec, responseVec_, freq, Ycorrection, smoothFactor):
    """
    This function deconvolves the step responseVec function using
    setPointVec = set point (input), responseVec = filtered gyro (output)
    returns matrix/stack of estimated stepresponse functions, time [t] in ms
    """
    # "THE BEER-WARE LICENSE" (Revision 42):
    # <brian.white@queensu.ca> wrote this file. As long as you retain this notice you
    # can do whatever you want with this stuff. If we meet some day, and you think
    # this stuff is worth it, you can buy me a beer in return. -Brian White

    smoothVals = [1, 20, 40, 60]
    responseVec = savgol_filter(responseVec_, smoothVals[smoothFactor], 3)  # 'lowess' equivalent in python

    minInput = 20
    segment_length = int(freq*2)  # 2 sec segments
    wnd = (freq) * .5  # 500ms step responseVec function, length will depend on lograte  
    StepRespDuration = 2  # max dur of step resp in ms for plotting
    t = np.arange(0, StepRespDuration + 1/freq, 1/freq)  # time in s 

    fileDurSec = len(setPointVec) / freq
    subsampleFactor = 1
    if fileDurSec <= 20:
        subsampleFactor = 10
    elif 20 < fileDurSec <= 60:
        subsampleFactor = 7
    elif fileDurSec > 60:
        subsampleFactor = 3

    segment_vector = np.arange(0, len(setPointVec), round(segment_length/subsampleFactor))
    NSegs = np.max(np.where((segment_vector+segment_length) < segment_vector[-1]))
    if NSegs > 0:
        SPseg = []
        GYseg = []
        j = 0
        for i in range(NSegs):
            if np.max(np.abs(setPointVec[segment_vector[i]:(segment_vector[i]+segment_length)])) >= minInput:
                j += 1
                SPseg.append(setPointVec[segment_vector[i]:(segment_vector[i]+segment_length)])
                GYseg.append(responseVec[segment_vector[i]:(segment_vector[i]+segment_length)])

        padLength = 100  # 2^nextpow2(length(SPseg[i,:]))
        j = 0
        if SPseg:
            for i in range(len(SPseg)):
                a = GYseg[i] * hann(len(GYseg[i]))
                b = SPseg[i] * hann(len(SPseg[i]))
                a = fft(np.concatenate([np.zeros(padLength), a, np.zeros(padLength)]))
                b = fft(np.concatenate([np.zeros(padLength), b, np.zeros(padLength)]))
                G = a / len(a)
                H = b / len(b)
                Hcon = np.conj(H)

                imp = np.real(ifft((G * Hcon) / (H * Hcon + 0.0001)))  # impulse responseVec function
                resptmp = np.cumsum(imp)  # integrate impulse resp function 

                steadyStateWindow = np.where((t > 200) & (t < StepRespDuration_ms))
                steadyStateResp = resptmp[steadyStateWindow]
                if Ycorrection:
                    if np.nanmean(steadyStateResp) < 1 or np.nanmean(steadyStateResp) > 1:
                        yoffset = 1 - np.nanmean(steadyStateResp)
                        resptmp = resptmp * (yoffset+1)
                    steadyStateResp = resptmp[steadyStateWindow]

                if np.min(steadyStateResp) > 0.5 and np.max(steadyStateResp) < 3:  # Quality control     
                    j += 1
                    stepresponse = resptmp[:1+wnd]
    return stepresponse, t

def main():
    # Load data
    dirName = '/home/valentin/crazyflie/Recordings/tube/'
    fileNameList = ['sd_33_cf2414.csv']

    data =  pd.read_csv(dirName + fileNameList[0])

    motor = data['motor.m1'].values
    motorOnInd = np.where(motor > 100)
    inds = range(motorOnInd[0][0], motorOnInd[0][-1])

    target = data['FF_Angle.pitch_cmd'].values[inds]
    measurement = data['stateEstimate.pitch'].values[inds]

    freq = np.round(1000./np.mean(np.diff(data['tick'].values[inds])))

    Ycorrection = False
    smoothFactor = 1

    # Calculate step responseVec
    stepresponse, t = PTstepcalc(target, measurement, freq, Ycorrection, smoothFactor)

    # Plot
    plt.plot(t, stepresponse)
    plt.show()

if __name__ == '__main__':
    main()