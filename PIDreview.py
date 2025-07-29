import csv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import colorsys

# Replace 'your_file.csv' with the actual path to your CSV file
import numpy as np
# from scipy.signal import hann, fftconvolve
# from scipy.fftpack import fft, ifft
# from scipy.signal.windows import hann
from pid_analysis import StepAnalysisType, Trace

import scipy.io


# def PTstepcalc(setPointVec_, responseVec_, samplingRate, Ycorrection, smoothTau, stepRespDuration = 1.0, segmentLength = 2):
#     """
#     [stepresponse, t] = PTstepcalc(setPointVec, responseVec, samplingRate, Ycorrection)
#     This function deconvolves the step response function using
#     setPointVec = set point (input), responseVec = filtered gyro (output)
#     Returns matrix/stack of estimated stepresponse functions, time [t] in ms

#     """
#     from scipy.signal import savgol_filter
#     smoothVals = int(smoothTau*samplingRate)
#     responseVec = savgol_filter(x= responseVec_, window_length=smoothVals, polyorder=3)  # 'lowess' equivalent in python
#     setPointVec = savgol_filter(x= setPointVec_, window_length=smoothVals, polyorder=3)  # 'lowess' equivalent in python

#     plt.figure(1)
#     plt.plot(setPointVec_, label='Set Point')
#     plt.plot(setPointVec, label='Filtered Set Point')
#     plt.plot(responseVec_, label='Response')
#     plt.plot(responseVec, label='Filtered Response')
#     plt.grid(True)
#     plt.xlabel('Time [s]')
#     plt.ylabel('Command/Response')
#     plt.legend()
#     plt.show()
    
#     minInput = max(abs(setPointVec)) * 0.05  # 5% of max input
#     segment_length = int(samplingRate * segmentLength)  # 2 sec segments
    
#     wnd = int(samplingRate * stepRespDuration)  # 0.5 sec step response function, length will depend on samplingRate  
#     t = np.arange(0, stepRespDuration + 1.0/samplingRate, 1.0/samplingRate)  # time in sec 

#     fileDurSec = len(setPointVec) / (samplingRate)
#     subsampleFactor = 1
#     if fileDurSec <= 20:
#         subsampleFactor = 10
#     elif fileDurSec > 20 and fileDurSec <= 60:
#         subsampleFactor = 7
#     elif fileDurSec > 60:
#         subsampleFactor = 3

#     segment_vector = np.arange(1, len(setPointVec), round(segment_length/subsampleFactor))
#     NSegs = max(np.where((segment_vector + segment_length) < segment_vector[-1])[0])
#     if NSegs > 0:
#         setPointSeg = []
#         responseSeg = []
#         j = 0
#         for i in range(NSegs):
#             if max(abs(setPointVec[segment_vector[i]:segment_vector[i] + segment_length])) >= minInput:
#                 j += 1
#                 setPointSeg.append(setPointVec[segment_vector[i]:segment_vector[i] + segment_length])
#                 responseSeg.append(responseVec[segment_vector[i]:segment_vector[i] + segment_length])

#         padLength = 100  # 2^nextpow2(length(setPointSeg[i,:]))
#         j = 0
#         if setPointSeg:
#             stepresponse = np.zeros((len(setPointSeg)+1, wnd + 1))
#             for i in range(len(setPointSeg)):
#                 a = responseSeg[i] * hann(len(responseSeg[i]))
#                 b = setPointSeg[i] * hann(len(setPointSeg[i]))
#                 a = fft(np.concatenate([np.zeros(padLength), a, np.zeros(padLength)]))
#                 b = fft(np.concatenate([np.zeros(padLength), b, np.zeros(padLength)]))
#                 G = a / len(a)
#                 H = b / len(b)
#                 Hcon = np.conj(H)

#                 imp = np.real(ifft((G * Hcon) / (H * Hcon + 0.0001)))  # impulse response function
#                 resptmp = np.cumsum(imp)  # integrate impulse resp function 

#                 steadyStateWindow = np.where((t > 0.2) & (t < stepRespDuration))
#                 steadyStateResp = resptmp[steadyStateWindow]
#                 if Ycorrection:
#                     if np.nanmean(steadyStateResp) < 1 or np.nanmean(steadyStateResp) > 1:
#                         yoffset = 1 - np.nanmean(steadyStateResp)
#                         resptmp = resptmp * (yoffset + 1)
#                     steadyStateResp = resptmp[steadyStateWindow]

#                 j += 1
#                 stepresponse[j,:] = resptmp[:1 + wnd]
            
#             stepresponse = stepresponse[:j,:]
#         else:
#             stepresponse = []
#     else:
#         stepresponse = []

#     stepresponse = np.nanmean(stepresponse, axis=0)
#     return stepresponse, t[:len(stepresponse)]
def plot_pid_response(trace, timeline, command, response, fignum=1, label='Rate'):
    """Plot PID response for one axis

    :param trace: Trace object
    :param data: ULog.data_list
    """

    def _color_palette(hue, N=20):
        """ Color palette for the 2D histogram """
        saturation = 0.75
        vmin = 0.5
        def sat(i, N, s):
            """ get saturation: s if i < N/2, otherwise linearly increase
            in range [s, 1] """
            if i * 2 < N: return s
            return s + (i-N/2) / (N/2) * (1-s)
        hsv_tuples = [(hue, sat(N/2-x, N, saturation), vmin + x*(1-vmin)/N) for x in reversed(range(N))]
        colors = []
        alpha_max = 0.5
        delta_alpha = alpha_max / N
        alpha = 0
        for rgb in hsv_tuples:
            rgb = list(map(lambda x: int(x*255), colorsys.hsv_to_rgb(*rgb)))
            colors.append('rgba({:.0f},{:.0f},{:.0f},{:.3f})'.format(rgb[0], rgb[1], rgb[2], alpha))
            alpha += delta_alpha
        return colors

    plt.figure(fignum)
    plt.subplot(3, 1, 1)
    plt.imshow(trace.resp_low[2][2], extent=[0, trace.resplen, min(trace.resp_low[2][1]), max(trace.resp_low[2][1])], aspect='auto', origin='lower')
    plt.ylim(0, 2.5)
    
    has_high_rates = trace.high_mask.sum() > 0
    low_rates_label = ''
    if has_high_rates:
        low_rates_label = ' (<500 deg/s)'

    plt.plot(trace.time_resp, trace.resp_low[0], label=label,    linewidth=4, color='red')
    plt.grid(True)
    plt.xlabel('Time [s]')
    plt.ylabel('Step Response')
    plt.legend()

    if has_high_rates:
        image = trace.resp_high[2][2] # 2D histogram
        plt.plot(trace.time_resp, trace.resp_high[0], label=trace.name.capitalize() + ' (>500 deg/s)', linewidth=4, color='blue')
    # horizonal marker line at 1
    plt.axhline(y=1, linestyle='--', linewidth=4, color='cyan')
   
    plt.subplot(3, 1, 2)
    plt.plot(timeline, command, label=label+' Command',    linewidth=2)
    plt.plot(timeline, response, label=label+' Response',    linewidth=1)
    plt.grid(True)
    plt.xlabel('Time [s]')
    plt.ylabel('Rate [deg/s]')
    plt.legend()
    
    # FFT of the command and response
    dt = timeline[1] - timeline[0]
    lastind = (len(command) // 1024) * 1024
    command = command[:lastind]
    response = response[:lastind]
    H = np.fft.fft(command, axis=-1)
    G = np.fft.fft(response,axis=-1)
    freq = np.abs(np.fft.fftfreq(len(command), dt))

    plt.subplot(3, 1, 3)   
    plt.semilogx(freq, np.abs(H), label='Command FFT')
    plt.semilogx(freq, np.abs(G), label='Response FFT')
    plt.grid(True)
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('FFT')
    plt.legend()

    pass
#############################################################################
def getData(data, tag, field):
    column = np.where(np.array([field]) == data[tag+'_label'])[0][0]
    try:
        return data[tag][:,column]
    except:
        return data[tag+'_0'][:,column]

####################################################################################################################
def getDataFromMatFile(matFile, startTime, endTime):
    data = scipy.io.loadmat(matFile)
    # ESCtime = getData(data, 'ESC', 'TimeUS')*1e-6
    # ESCrpm = getData(data, 'ESC', 'rpm')
    ATTtime = getData(data, 'ATT', 'TimeUS')*1e-6
    initTime = ATTtime[0]
    ATTtime = ATTtime - initTime

    throttle = getData(data, 'CTUN', 'ThO')
    throttleTime = getData(data, 'CTUN', 'TimeUS')*1e-6-initTime

    MODEtime = getData(data, 'MODE', 'TimeUS')*1e-6-initTime
    MODEmode = getData(data, 'MODE', 'ModeNum')

    ARSPtime = getData(data, 'ARSP', 'TimeUS')*1e-6
    ARSPairspeed = getData(data, 'ARSP', 'Airspeed')

    ATTpitchCommand = getData(data, 'ATT', 'DesPitch')
    ATTrollCommand = getData(data, 'ATT', 'DesRoll')
    ATTpitchResponse = getData(data, 'ATT', 'Pitch')
    ATTrollResponse = getData(data, 'ATT', 'Roll')

    PIDRtime = getData(data, 'PIDR', 'TimeUS')*1e-6-initTime
    PIDRtarget = getData(data, 'PIDR', 'Tar')
    PIDRactual = getData(data, 'PIDR', 'Act')

    PIDPtime = getData(data, 'PIDP', 'TimeUS')*1e-6-initTime
    PIDPtarget = getData(data, 'PIDP', 'Tar')
    PIDPactual = getData(data, 'PIDP', 'Act')
    
    timeLine = np.linspace(startTime, endTime, int((endTime-startTime)*400))

    pitchCommand = np.interp(timeLine, ATTtime, ATTpitchCommand)
    pitchResponse = np.interp(timeLine, ATTtime, ATTpitchResponse)
    pitchRateCommand = np.interp(timeLine, ATTtime, PIDPtarget)
    pitchRateResponse = np.interp(timeLine, ATTtime, PIDPactual)

    rollCommand = np.interp(timeLine, ATTtime, ATTrollCommand)
    rollResponse = np.interp(timeLine, ATTtime, ATTrollResponse)
    rollRateCommand = np.interp(timeLine, ATTtime, PIDRtarget)
    rollRateResponse = np.interp(timeLine, ATTtime, PIDRactual)

    throttle = np.interp(timeLine, throttleTime, throttle)

    return timeLine, pitchRateCommand, pitchRateResponse, rollRateCommand, rollRateResponse, throttle

def getDataFromUlog(ratesSetpointCSVFile, ratesResponseCSVFile, throttleSetpointCSVFile, startTime=0, endTime=1000):
    ratesSetpoint = pd.read_csv(ratesSetpointCSVFile)
    ratesResponse = pd.read_csv(ratesResponseCSVFile)
    throttleSetpoint = pd.read_csv(throttleSetpointCSVFile)
    
    timeLineSetpoint = np.array(ratesSetpoint['timestamp'])*1e-6
    rollRateSetpoint = np.array(ratesSetpoint['roll'])
    pitchRateSetpoint = np.array(ratesSetpoint['pitch'])

    timeLineResponse = np.array(ratesResponse['timestamp'])*1e-6   
    rollRateResponse = np.array(ratesResponse['xyz[0]'])
    pitchRateResponse = np.array(ratesResponse['xyz[1]'])    
    
    timeLineThrottle = np.array(throttleSetpoint['timestamp'])*1e-6
    throttle = np.array(throttleSetpoint['xyz[2]'])
    
    timeLine = timeLineResponse  #np.linspace(startTime, endTime, 10001)
    
    pitchRateCommand = np.interp(timeLine, timeLineSetpoint, pitchRateSetpoint)
    pitchRateResponse = np.interp(timeLine, timeLineResponse, pitchRateResponse)
    rollRateCommand = np.interp(timeLine, timeLineSetpoint, rollRateSetpoint)
    rollRateResponse = np.interp(timeLine, timeLineResponse, rollRateResponse)
    throttle = np.interp(timeLine, timeLineThrottle, throttle)
        
    return timeLine, pitchRateCommand, pitchRateResponse, rollRateCommand, rollRateResponse, throttle

####################################################################################################################
####################################################################################################################
####################################################################################################################

# plt.figure(1)
# plt.subplot(3, 1, 1)
# plt.plot(ATTtime, ATTpitchCommand, label='Command')
# plt.plot(ATTtime, ATTpitchResponse+1, label='Response')
# plt.plot(PIDPtime, PIDPtarget, label='PIDP Target')
# plt.plot(PIDPtime, PIDPactual, label='PIDP Actual')
# plt.grid(True)
# plt.xlabel('Time [s]')
# plt.ylabel('Pitch [deg]')
# plt.legend()
# plt.subplot(3, 1, 2)
# plt.plot(ATTtime, ATTrollCommand, label='Command')
# plt.plot(ATTtime, ATTrollResponse, label='Response')
# plt.plot(PIDRtime, PIDRtarget, label='Target')
# plt.plot(PIDRtime, PIDRactual, label='Actual')
# plt.grid(True)
# plt.xlabel('Time [s]')
# plt.ylabel('Roll [deg]')
# plt.legend()
# plt.subplot(3, 1, 3)
# plt.plot(MODEtime, MODEmode, label='Mode')
# plt.grid(True)
# plt.xlabel('Time [s]')
# plt.ylabel('Mode')
# plt.legend()

# matFile1 = '/home/valentin/Projects/Triangle/Records/11-6-25ein_yahav.BIN-1195761.mat'
# timeLine, pitchRateCommand, pitchRateResponse, rollRateCommand, rollRateResponse, throttle = \
#        getDataFromMatFile(matFile1, startTime=2500, endTime=3000)

matFile2 = '/home/valentin/Projects/Triangle/Records/t/00000015.BIN-322292.mat'
timeLine, pitchRateCommand, pitchRateResponse, rollRateCommand, rollRateResponse, throttle = \
       getDataFromMatFile(matFile2, startTime=510, endTime=710)

# ratesSetpointCSVFile = '/home/valentin/Projects/GambitonBiut/Recordings/t/log_7_UnknownDate_vehicle_rates_setpoint_0.csv'
# ratesResponseCSVFile = '/home/valentin/Projects/GambitonBiut/Recordings/t/log_7_UnknownDate_vehicle_angular_velocity_0.csv'
# throttleSetpointCSVFile = '/home/valentin/Projects/GambitonBiut/Recordings/t/log_7_UnknownDate_vehicle_thrust_setpoint_0.csv'
# timeLine, pitchRateCommand, pitchRateResponse, rollRateCommand, rollRateResponse, throttle = getDataFromUlog(ratesSetpointCSVFile, ratesResponseCSVFile, throttleSetpointCSVFile, startTime=70, endTime=170)
# pitchRateCommand = np.rad2deg(pitchRateCommand)
# rollRateCommand = np.rad2deg(rollRateCommand)
# pitchRateResponse = np.rad2deg(pitchRateResponse)
# rollRateResponse = np.rad2deg(rollRateResponse)

# # Plot the step response
# plt.figure(2)
# plt.subplot(2, 1, 1)
# plt.plot(timeLine, pitchRateCommand, label='Pitch Rate Command')
# plt.plot(timeLine, pitchRateResponse, label='Pitch Rate Response')
# plt.grid(True)
# plt.xlabel('Time [s]')
# plt.ylabel('Step Response')
# plt.legend()

# plt.subplot(2, 1, 2)
# plt.plot(timeLine, rollRateCommand, label='Roll Rate Command')
# plt.plot(timeLine, rollRateResponse, label='Roll Rate Response')
# plt.grid(True)
# plt.xlabel('Time [s]')
# plt.ylabel('Step Response')
# plt.legend()


samplingRate = int(1/np.mean(np.diff(timeLine)))

Ycorrection = 1
stepRespDuration = 10
segmentLength = stepRespDuration

trace = Trace(name='pitchRate', time=timeLine, result=pitchRateResponse, setpoint=pitchRateCommand, throttle=throttle, resplen=3)
plot_pid_response(trace, timeLine, pitchRateCommand, pitchRateResponse, fignum=10, label='Pitch Rate')

trace = Trace(name='rollRate', time=timeLine, result=rollRateResponse, setpoint=rollRateCommand, throttle=throttle, resplen=3)
plot_pid_response(trace, timeLine, rollRateCommand, rollRateResponse, fignum=11, label='Roll Rate')

plt.show()
pass



