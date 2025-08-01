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
from pyulog import ULog
from pyulog.px4 import PX4ULog

from ardupilot_log_reader import Ardupilot

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
def getDataFromBINFile(binFile, startTime, endTime):
    def type_request(): return ['ATT', 'CTUN', 'MODE', 'PIDR', 'PIDP', 'ARSP']
    data = Ardupilot.parse(binFile, types=type_request(), zero_time_base=True)   


    # ESCtime = getData(data, 'ESC', 'TimeUS')*1e-6
    # ESCrpm = getData(data, 'ESC', 'rpm')
    ATTtime = data.dfs['ATT']['TimeUS']*1e-6
    initTime = ATTtime[0]
    ATTtime = ATTtime - initTime

    throttle = data.dfs['CTUN']['ThO']
    throttleTime = data.dfs['CTUN']['TimeUS']*1e-6-initTime

    MODEtime = data.dfs['MODE']['TimeUS']*1e-6-initTime
    MODEmode = data.dfs['MODE']['ModeNum']

    ARSPtime = data.dfs['ARSP']['TimeUS']*1e-6
    ARSPairspeed = data.dfs['ARSP']['Airspeed']

    ATTpitchCommand = data.dfs['ATT']['DesPitch']
    ATTrollCommand = data.dfs['ATT']['DesRoll']
    ATTpitchResponse = data.dfs['ATT']['Pitch']
    ATTrollResponse = data.dfs['ATT']['Roll']
    ATTyawCommand = data.dfs['ATT']['DesYaw']
    ATTyawResponse = data.dfs['ATT']['Yaw']

    PIDRtime = data.dfs['PIDR']['TimeUS']*1e-6-initTime
    PIDRtarget = data.dfs['PIDR']['Tar']
    PIDRactual = data.dfs['PIDR']['Act']

    PIDPtime = data.dfs['PIDP']['TimeUS']*1e-6-initTime
    PIDPtarget = data.dfs['PIDP']['Tar']
    PIDPactual = data.dfs['PIDP']['Act']
    
    timeLine = np.linspace(startTime, endTime, int((endTime-startTime)*400))

    rollCommand = np.interp(timeLine, ATTtime, ATTrollCommand)
    rollResponse = np.interp(timeLine, ATTtime, ATTrollResponse)
    rollRateCommand = np.interp(timeLine, ATTtime, PIDRtarget)
    rollRateResponse = np.interp(timeLine, ATTtime, PIDRactual)
    pitchCommand = np.interp(timeLine, ATTtime, ATTpitchCommand)
    pitchResponse = np.interp(timeLine, ATTtime, ATTpitchResponse)
    pitchRateCommand = np.interp(timeLine, ATTtime, PIDPtarget)
    pitchRateResponse = np.interp(timeLine, ATTtime, PIDPactual)
    yawCommand = np.interp(timeLine, ATTtime, ATTyawCommand)
    yawResponse = np.interp(timeLine, ATTtime, ATTyawResponse)
    yawRateCommand = np.interp(timeLine, ATTtime, PIDRtarget)
    yawRateResponse = np.interp(timeLine, ATTtime, PIDRactual)

    throttle = np.interp(timeLine, throttleTime, throttle)

    attCommand = {'roll': rollCommand, 'pitch': pitchCommand, 'yaw': yawCommand}
    attResponse = {'roll': rollResponse, 'pitch': pitchResponse, 'yaw': yawResponse}
    ratesCommand = {'roll': rollRateCommand, 'pitch': pitchRateCommand, 'yaw': yawRateCommand}
    ratesResponse = {'roll': rollRateResponse, 'pitch': pitchRateResponse, 'yaw': yawRateResponse}
    thrustCommand = {'thrust': throttle}
    positionCommand = {'vx': 0, 'vy': 0, 'vz': 0}
    positionResponse = {'vx': 0, 'vy': 0, 'vz': 0}
    return timeLine, attCommand, attResponse, ratesCommand, ratesResponse, thrustCommand, positionCommand, positionResponse

def getDataFromUlog(ulogFileName, startTime=0, endTime=1000):
    ulog = ULog(ulogFileName)
    ulogData=ulog.data_list
    px4_ulog = PX4ULog(ulog)
    px4_ulog.add_roll_pitch_yaw()
    ulogData=px4_ulog._ulog
    # ATTITUDE
    attitudeSetpoint = ulog.get_dataset('vehicle_attitude_setpoint')
    attitudeTimeLineSetpoint = np.array(attitudeSetpoint.data['timestamp'])*1e-6
    rollSetpoint = np.rad2deg(np.array(attitudeSetpoint.data['roll_body']))
    pitchSetpoint = np.rad2deg(np.array(attitudeSetpoint.data['pitch_body']))
    yawSetpoint = np.rad2deg(np.array(attitudeSetpoint.data['yaw_body']))

    attitudeResponse = ulog.get_dataset('vehicle_attitude')
    attitudeTimeLineResponse = np.array(attitudeResponse.data['timestamp'])*1e-6
    rollResponse = np.rad2deg(np.array(attitudeResponse.data['roll']))
    pitchResponse = np.rad2deg(np.array(attitudeResponse.data['pitch']))
    yawResponse = np.rad2deg(np.array(attitudeResponse.data['yaw']))
    # RATES
    ratesSetpoint = ulog.get_dataset('vehicle_rates_setpoint')
    ratesTimeLineSetpoint = np.array(ratesSetpoint.data['timestamp'])*1e-6
    rollRateSetpoint = np.rad2deg(np.array(ratesSetpoint.data['roll']))
    pitchRateSetpoint = np.rad2deg(np.array(ratesSetpoint.data['pitch']))
    yawRateSetpoint = np.rad2deg(np.array(ratesSetpoint.data['yaw']))
    
    ratesResponse = ulog.get_dataset('vehicle_angular_velocity')
    ratesTimeLineResponse = np.array(ratesResponse.data['timestamp'])*1e-6
    rollRateResponse = np.rad2deg(np.array(ratesResponse.data['xyz[0]']))
    pitchRateResponse = np.rad2deg(np.array(ratesResponse.data['xyz[1]']))
    yawRateResponse = np.rad2deg(np.array(ratesResponse.data['xyz[2]']))
    # THRUST
    thrustSetpoint = ulog.get_dataset('vehicle_thrust_setpoint')
    thrustTimeLineSetpoint = np.array(thrustSetpoint.data['timestamp'])*1e-6
    thrustSetpoint = np.array(thrustSetpoint.data['xyz[2]'])
    # POSITION
    positionSetpoint = ulog.get_dataset('vehicle_local_position_setpoint')
    positionTimeLineSetpoint = np.array(positionSetpoint.data['timestamp'])*1e-6
    vxSetpoint = np.array(positionSetpoint.data['vx'])
    vySetpoint = np.array(positionSetpoint.data['vy'])
    vzSetpoint = np.array(positionSetpoint.data['vz'])
    positionResponse = ulog.get_dataset('vehicle_local_position')
    positionTimeLineResponse = np.array(positionResponse.data['timestamp'])*1e-6
    vxResponse = np.array(positionResponse.data['vx'])
    vyResponse = np.array(positionResponse.data['vy'])
    vzResponse = np.array(positionResponse.data['vz'])
    
    # interpolate data to the same time line
    timeLine = np.linspace(startTime, endTime, int((endTime-startTime)*400)+1)
    attCommand = {'roll': np.interp(timeLine, attitudeTimeLineSetpoint, rollSetpoint),
                  'pitch': np.interp(timeLine, attitudeTimeLineSetpoint, pitchSetpoint),
                  'yaw': np.interp(timeLine, attitudeTimeLineSetpoint, yawSetpoint)}
    attResponse = {'roll': np.interp(timeLine, attitudeTimeLineResponse, rollResponse),
                  'pitch': np.interp(timeLine, attitudeTimeLineResponse, pitchResponse),
                  'yaw': np.interp(timeLine, attitudeTimeLineResponse, yawResponse)}
    ratesCommand = {'roll': np.interp(timeLine, ratesTimeLineSetpoint, rollRateSetpoint),
                    'pitch': np.interp(timeLine, ratesTimeLineSetpoint, pitchRateSetpoint),
                    'yaw': np.interp(timeLine, ratesTimeLineSetpoint, yawRateSetpoint)}
    ratesResponse = {'roll': np.interp(timeLine, ratesTimeLineResponse, rollRateResponse),
                    'pitch': np.interp(timeLine, ratesTimeLineResponse, pitchRateResponse),
                    'yaw': np.interp(timeLine, ratesTimeLineResponse, yawRateResponse)}
    thrustCommand = {'thrust': np.interp(timeLine, thrustTimeLineSetpoint, thrustSetpoint)}
    positionCommand = {'vx': np.interp(timeLine, positionTimeLineSetpoint, vxSetpoint),
                       'vy': np.interp(timeLine, positionTimeLineSetpoint, vySetpoint),
                       'vz': np.interp(timeLine, positionTimeLineSetpoint, vzSetpoint)}
    positionResponse = {'vx': np.interp(timeLine, positionTimeLineResponse, vxResponse),
                        'vy': np.interp(timeLine, positionTimeLineResponse, vyResponse),
                        'vz': np.interp(timeLine, positionTimeLineResponse, vzResponse)}
    
    return timeLine, attCommand, attResponse, ratesCommand, ratesResponse, thrustCommand, positionCommand, positionResponse

####################################################################################################################
####################################################################################################################
####################################################################################################################
if __name__ == '__main__':
    
    if 0:
        binFile = '/home/valentin/Projects/Triangle/Records/11-6-25ein_yahav.BIN'
        timeLine, attCommand, attResponse, ratesCommand, ratesResponse, thrustCommand, positionCommand, positionResponse = \
            getDataFromBINFile(binFile, startTime=2500, endTime=3000)

        samplingRate = int(1/np.mean(np.diff(timeLine)))

        Ycorrection = 1
        stepRespDuration = 10
        segmentLength = stepRespDuration

        trace = Trace(name='pitchRate', time=timeLine, result=ratesResponse['pitch'], setpoint=ratesCommand['pitch'], throttle=thrustCommand['thrust'], resplen=3)
        plot_pid_response(trace, timeLine, ratesCommand['pitch'], ratesResponse['pitch'], fignum=10, label='Pitch Rate')

        trace = Trace(name='rollRate', time=timeLine, result=ratesResponse['roll'], setpoint=ratesCommand['roll'], throttle=thrustCommand['thrust'], resplen=3)
        plot_pid_response(trace, timeLine, ratesCommand['roll'], ratesResponse['roll'], fignum=11, label='Roll Rate')

        plt.show()
        pass

    # matFile2 = '/home/valentin/Projects/Triangle/Records/t/00000015.BIN-322292.mat'
    # timeLine, pitchRateCommand, pitchRateResponse, rollRateCommand, rollRateResponse, throttle = \
    #        getDataFromMatFile(matFile2, startTime=510, endTime=710)
    else:
        ulogFileName = '/home/valentin/Projects/GambitonBiut/Recordings/log_7_UnknownDate.ulg'
        timeLine, attCommand, attResponse, ratesCommand, ratesResponse, thrustCommand, positionCommand, positionResponse = getDataFromUlog(ulogFileName, startTime=70, endTime=170)

        samplingRate = int(1/np.mean(np.diff(timeLine)))

        Ycorrection = 1
        stepRespDuration = 10
        segmentLength = stepRespDuration

        traceVelX = Trace(name='velX', time=timeLine, result=positionResponse['vx'], setpoint=positionCommand['vx'], throttle=thrustCommand['thrust'], resplen=3, step_analysis=StepAnalysisType.VELOCITY)
        plot_pid_response(traceVelX, timeLine, positionCommand['vx'], positionResponse['vx'], fignum=14, label='Velocity X')

        traceVelY = Trace(name='velY', time=timeLine, result=positionResponse['vy'], setpoint=positionCommand['vy'], throttle=thrustCommand['thrust'], resplen=3, step_analysis=StepAnalysisType.VELOCITY)
        plot_pid_response(traceVelY, timeLine, positionCommand['vy'], positionResponse['vy'], fignum=15, label='Velocity Y')

        traceVelZ = Trace(name='velZ', time=timeLine, result=positionResponse['vz'], setpoint=positionCommand['vz'], throttle=thrustCommand['thrust'], resplen=3, step_analysis=StepAnalysisType.VELOCITY)
        plot_pid_response(traceVelZ, timeLine, positionCommand['vz'], positionResponse['vz'], fignum=16, label='Velocity Z')

        tracePitchRate = Trace(name='pitchRate', time=timeLine, result=ratesResponse['pitch'], setpoint=ratesCommand['pitch'], throttle=thrustCommand['thrust'], resplen=3, step_analysis=StepAnalysisType.ANGLE_RATE)
        plot_pid_response(tracePitchRate, timeLine, ratesCommand['pitch'], ratesResponse['pitch'], fignum=10, label='Pitch Rate')

        traceRollRate = Trace(name='rollRate', time=timeLine, result=ratesResponse['roll'], setpoint=ratesCommand['roll'], throttle=thrustCommand['thrust'], resplen=3, step_analysis=StepAnalysisType.ANGLE_RATE)
        plot_pid_response(traceRollRate, timeLine, ratesCommand['roll'], ratesResponse['roll'], fignum=11, label='Roll Rate')

        tracePitchAtt = Trace(name='pitchAtt', time=timeLine, result=attResponse['pitch'], setpoint=attCommand['pitch'], throttle=thrustCommand['thrust'], resplen=3, step_analysis=StepAnalysisType.ANGLE)
        plot_pid_response(tracePitchAtt, timeLine, attCommand['pitch'], attResponse['pitch'], fignum=12, label='Pitch Attitude')

        traceRollAtt = Trace(name='rollAtt', time=timeLine, result=attResponse['roll'], setpoint=attCommand['roll'], throttle=thrustCommand['thrust'], resplen=3, step_analysis=StepAnalysisType.ANGLE)
        plot_pid_response(traceRollAtt, timeLine, attCommand['roll'], attResponse['roll'], fignum=13, label='Roll Attitude')

        plt.show()
        pass



