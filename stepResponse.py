import csv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Replace 'your_file.csv' with the actual path to your CSV file


import numpy as np
from scipy.signal import hann, fftconvolve
from scipy.fftpack import fft, ifft

def PTstepcalc(setPointVec, responseVec, samplingRate, Ycorrection, smoothFactor, stepRespDuration = 1.0):
    """
    [stepresponse, t] = PTstepcalc(setPointVec, responseVec, samplingRate, Ycorrection)
    This function deconvolves the step response function using
    setPointVec = set point (input), responseVec = filtered gyro (output)
    Returns matrix/stack of estimated stepresponse functions, time [t] in ms

    """
    from scipy.signal import savgol_filter
    smoothVals = [1, 20, 40, 60]
    responseVec = savgol_filter(x= responseVec, window_length=smoothVals[smoothFactor], polyorder=3)  # 'lowess' equivalent in python

    minInput = max(abs(setPointVec)) * 0.05  # 5% of max input
    segment_length = int(samplingRate * 2)  # 2 sec segments
    
    wnd = int(samplingRate * stepRespDuration)  # 0.5 sec step response function, length will depend on samplingRate  
    t = np.arange(0, stepRespDuration + 1.0/samplingRate, 1.0/samplingRate)  # time in sec 

    fileDurSec = len(setPointVec) / (samplingRate)
    subsampleFactor = 1
    if fileDurSec <= 20:
        subsampleFactor = 10
    elif fileDurSec > 20 and fileDurSec <= 60:
        subsampleFactor = 7
    elif fileDurSec > 60:
        subsampleFactor = 3

    segment_vector = np.arange(1, len(setPointVec), round(segment_length/subsampleFactor))
    NSegs = max(np.where((segment_vector + segment_length) < segment_vector[-1])[0])
    if NSegs > 0:
        setPointSeg = []
        responseSeg = []
        j = 0
        for i in range(NSegs):
            if max(abs(setPointVec[segment_vector[i]:segment_vector[i] + segment_length])) >= minInput:
                j += 1
                setPointSeg.append(setPointVec[segment_vector[i]:segment_vector[i] + segment_length])
                responseSeg.append(responseVec[segment_vector[i]:segment_vector[i] + segment_length])

        padLength = 100  # 2^nextpow2(length(setPointSeg[i,:]))
        j = 0
        if setPointSeg:
            stepresponse = np.zeros((len(setPointSeg)+1, wnd + 1))
            for i in range(len(setPointSeg)):
                a = responseSeg[i] * hann(len(responseSeg[i]))
                b = setPointSeg[i] * hann(len(setPointSeg[i]))
                a = fft(np.concatenate([np.zeros(padLength), a, np.zeros(padLength)]))
                b = fft(np.concatenate([np.zeros(padLength), b, np.zeros(padLength)]))
                G = a / len(a)
                H = b / len(b)
                Hcon = np.conj(H)

                imp = np.real(ifft((G * Hcon) / (H * Hcon + 0.0001)))  # impulse response function
                resptmp = np.cumsum(imp)  # integrate impulse resp function 

                steadyStateWindow = np.where((t > 0.2) & (t < stepRespDuration))
                steadyStateResp = resptmp[steadyStateWindow]
                if Ycorrection:
                    if np.nanmean(steadyStateResp) < 1 or np.nanmean(steadyStateResp) > 1:
                        yoffset = 1 - np.nanmean(steadyStateResp)
                        resptmp = resptmp * (yoffset + 1)
                    steadyStateResp = resptmp[steadyStateWindow]

                j += 1
                stepresponse[j,:] = resptmp[:1 + wnd]
            
            stepresponse = stepresponse[:j,:]
        else:
            stepresponse = []
    else:
        stepresponse = []

    stepresponse = np.nanmean(stepresponse, axis=0)
    return stepresponse, t[:len(stepresponse)]



csv_file = '/home/valentin/robotics/crazyflie/testData/sd_33_cf2411.csv'

# Call the function to read the CSV file and store the result in a variable
df = pd.read_csv(csv_file)

inds= df['motor.m1'] > 100
startInd = np.where(inds)[0][0]
endind = np.where(inds)[0][-1]
inds=range(startInd,endind)

ticks = df['tick'][inds]
pitchCommand = df['FF_Angle.pitch_cmd'][inds]
pitchResponse = df['stateEstimate.pitch'][inds]

rollCommand = df['FF_Angle.roll_cmd'][inds]
rollResponse = df['stateEstimate.roll'][inds]

vxCommand = df['ctrltarget.vx'][inds]
vxResponse = df['stateEstimate.vx'][inds]

vyCommand = df['ctrltarget.vy'][inds]
vyResponse = df['stateEstimate.vy'][inds]

pitchRateCommand = df['controller.pitchRate'][inds]
rollRateCommand = df['controller.rollRate'][inds]


samplingRate = int(1000/np.mean(np.diff(ticks)))
smoothFactor = 1
Ycorrection = 1

pitchStepRes, pitchTime = PTstepcalc(pitchCommand, pitchResponse, samplingRate, Ycorrection, smoothFactor)
rollStepRes, rollTime = PTstepcalc(rollCommand, rollResponse, samplingRate, Ycorrection, smoothFactor)
vxStepRes, vxTime = PTstepcalc(vxCommand, vxResponse, samplingRate, Ycorrection, smoothFactor, stepRespDuration = 5)
vyStepRes, vyTime = PTstepcalc(vyCommand, vyResponse, samplingRate, Ycorrection, smoothFactor, stepRespDuration = 5)

# Plot the step response
plt.subplot(2, 1, 1)
plt.plot(pitchTime, pitchStepRes, label='Pitch Step Response')
plt.plot(rollTime, rollStepRes, label='Roll Step Response')
plt.grid(True)
plt.xlabel('Time [s]')
plt.ylabel('Step Response')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(vxTime, vxStepRes, label='Vx Step Response')
plt.plot(vyTime, vyStepRes, label='Vy Step Response')
plt.grid(True)
plt.xlabel('Time [s]')
plt.ylabel('Step Response')
plt.legend()
plt.show()
# Print the CSV data dictionary
print(df)