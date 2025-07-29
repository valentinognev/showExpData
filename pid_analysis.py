""" PID response analysis """

import colorsys

import numpy as np

from bokeh.models import Range1d, Span, LinearColorMapper, ColumnDataSource, LabelSet
from scipy.interpolate import interp1d
from scipy.ndimage.filters import gaussian_filter1d

# from config import colors3
# from plotting import DataPlot

# keep the same formatting as the original code
# pylint: skip-file

# Source: https://github.com/Plasmatree/PID-Analyzer
# "THE BEER-WARE LICENSE" (Revision 42):
# <florian.melsheimer@gmx.de> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return. Florian Melsheimer

#enum class for step analysis
class StepAnalysisType:
    ANGLE_RATE = 0
    ANGLE = 1
    VELOCITY = 2

class Trace:
    """ PID response analysis based on a deconvolution using a
    setpoint and the measured process variable as inputs.
    It computes an average, stdev and a 2D histogram.
    """
    
    cutfreq = 25.           # cutfreqency of what is considered as input
    tuk_alpha = 1.0         # alpha of tukey window, if used
    superpos = 16           # sub windowing (superpos windows in framelen)
    threshold = 500.        # threshold for 'high input rate'
    low_threshold = 20      # threshold for 'low input rate'
    noise_framelen = 0.3    # window width for noise analysis
    noise_superpos = 16     # subsampling for noise analysis windows
    vert_range = [-1.5,3.5]

    def __init__(self, name, time, result, setpoint, throttle, resplen = 0.5,
                 d_err=None, debug=None, step_analysis:StepAnalysisType=StepAnalysisType.ANGLE_RATE):
        """Initialize a Trace object, that does the analysis for a single axis.

        Note: all data arrays must have the same length as time

        :param name: axis name (e.g. roll)
        :param time: np array with sampling times [s]
        :param result: np array with the result rates [deg/s]
        :param throttle: np array with the throttle setpoint [0, 100]

        :param d_err: np array with D term error (optional)
        :param debug: TODO
        """
        
        self.resplen = resplen           # length of respose window
        self.framelen = resplen*2        # length of each single frame over which to compute response
        
        # equally space samples in time
        data = {
            'result': result,
            'setpoint': setpoint,
            'throttle': throttle
            }
        if d_err is not None: data['d_err'] = d_err
        if debug is not None: data['debug'] = debug
        self.time, self.data = self.equalize_data(time, data)
        self.result = self.data['result']
        self.setpoint = self.data['setpoint']
        self.throttle = self.data['throttle']
        self.dt = self.time[0]-self.time[1]

        self.data['time'] = self.time

        self.name = name

        #enable this to generate artifical result trace with known system response
        #self.result=self.toy_out(self.setpoint, delay=0.01, mode='normal')####
        if step_analysis == StepAnalysisType.VELOCITY:
            Trace.framelen = 6
            Trace.resplen = 3
            Trace.threshold = 3
            Trace.low_threshold = 1
        elif step_analysis == StepAnalysisType.ANGLE:
            Trace.framelen = 2
            Trace.resplen = 1
            Trace.threshold = 40
            Trace.low_threshold = 1

        self.flen = self.stepcalc(self.time, self.framelen)        # array len corresponding to framelen in s
        self.rlen = self.stepcalc(self.time, self.resplen)         # array len corresponding to resplen in s
        self.time_resp = self.time[0:self.rlen]-self.time[0]

        self.stacks = self.winstacker({'time':[],'setpoint':[],'result':[], 'throttle':[]}, self.flen, Trace.superpos)                                  # [[time, setpoint, output],]
        self.window = np.hanning(self.flen)                                     #self.tukeywin(self.flen, self.tuk_alpha)
        self.spec_sm, self.avr_t, self.avr_in, self.max_in, self.max_thr = self.stack_response(self.stacks, self.window)
        self.low_mask, self.high_mask = self.low_high_mask(self.max_in, self.threshold)       #calcs masks for high and low setpoints according to threshold
        self.toolow_mask = self.low_high_mask(self.max_in, Trace.low_threshold)[1]          #mask for ignoring noisy low setpoint

# commented, because it's unused
#        self.resp_sm = self.weighted_mode_avr(self.spec_sm, self.toolow_mask, [-1.5,3.5], 1000)
#        self.resp_quality = -self.to_mask((np.abs(self.spec_sm -self.resp_sm[0]).mean(axis=1)).clip(0.5-1e-9,0.5))+1.
#        # masking by setting trottle of unwanted traces to neg
#        self.thr_response = self.hist2d(self.max_thr * (2. * (self.toolow_mask*self.resp_quality) - 1.), self.time_resp,
#                                        (self.spec_sm.transpose() * self.toolow_mask).transpose(), [101, self.rlen])

        self.resp_low = self.weighted_mode_avr(values=self.spec_sm, weights=self.low_mask*self.toolow_mask, vertrange=Trace.vert_range, vertbins=1000)
        if self.high_mask.sum()>0:
            self.resp_high = self.weighted_mode_avr(values=self.spec_sm, weights=self.high_mask*self.toolow_mask, vertrange=Trace.vert_range, vertbins=1000)

        if 'd_err' in self.data:
            self.noise_winlen = self.stepcalc(self.time, Trace.noise_framelen)
            self.noise_stack = self.winstacker({'time':[], 'result':[], 'throttle':[], 'd_err':[], 'debug':[]},
                                               self.noise_winlen, Trace.noise_superpos)
            self.noise_win = np.hanning(self.noise_winlen)

            self.noise_result = self.stackspectrum(self.noise_stack['time'],self.noise_stack['throttle'],self.noise_stack['result'], self.noise_win)
            self.noise_d = self.stackspectrum(self.noise_stack['time'], self.noise_stack['throttle'], self.noise_stack['d_err'], self.noise_win)
            self.noise_debug = self.stackspectrum(self.noise_stack['time'], self.noise_stack['throttle'], self.noise_stack['debug'], self.noise_win)
            if self.noise_debug['hist2d'].sum()>0:
                ## mask 0 entries
                thr_mask = self.noise_result['throt_hist_avr'].clip(0,1)
                self.filter_trans = np.average(self.noise_result['hist2d'], axis=1, weights=thr_mask)/\
                                    np.average(self.noise_debug['hist2d'], axis=1, weights=thr_mask)
            else:
                self.filter_trans = self.noise_result['hist2d'].mean(axis=1)*0.

    @staticmethod
    def low_high_mask(signal, threshold):
        low = np.copy(signal)

        low[low <=threshold] = 1.
        low[low > threshold] = 0.
        high = -low+1.

        if high.sum() < 10:     # ignore high pinput that is too short
            high *= 0.

        return low, high

    def to_mask(self, clipped):
        clipped-=clipped.min()
        clipped_max = clipped.max()
        if clipped_max > 1e-10: # avoid division by zero
            clipped/=clipped_max
        return clipped


    def rate_curve(self, rcin, inmax=500., outmax=800., rate=160.):
        ### an estimated rate curve. not used.
        expoin = (np.exp((rcin - inmax) / rate) - np.exp((-rcin - inmax) / rate)) * outmax
        return expoin


    def tukeywin(self, len, alpha=0.5):
        ### makes tukey widow for envelopig
        M = len
        n = np.arange(M - 1.)  #
        if alpha <= 0:
            return np.ones(M)  # rectangular window
        elif alpha >= 1:
            return np.hanning(M)

        # Normal case
        x = np.linspace(0, 1, M, dtype=np.float64)
        w = np.ones(x.shape)

        # first condition 0 <= x < alpha/2
        first_condition = x < alpha / 2
        w[first_condition] = 0.5 * (1 + np.cos(2 * np.pi / alpha * (x[first_condition] - alpha / 2)))

        # second condition already taken care of

        # third condition 1 - alpha / 2 <= x <= 1
        third_condition = x >= (1 - alpha / 2)
        w[third_condition] = 0.5 * (1 + np.cos(2 * np.pi / alpha * (x[third_condition] - 1 + alpha / 2)))

        return w

    def toy_out(self, inp, delay=0.01, length=0.01, noise=5., mode='normal', sinfreq=100.):
        # generates artificial output for benchmarking
        freq= 1./(self.time[1]-self.time[0])
        toyresp = np.zeros(int((delay+length)*freq))
        toyresp[int((delay)*freq):]=1.
        toyresp/=toyresp.sum()
        toyout = np.convolve(inp, toyresp, mode='full')[:len(inp)]#*0.9
        if mode=='normal':
            noise_sig = (np.random.random_sample(len(toyout))-0.5)*noise
        elif mode=='sin':
            noise_sig = (np.sin(2.*np.pi*self.time*sinfreq)) * noise
        else:
            noise_sig=0.
        return toyout+noise_sig


    @staticmethod
    def equalize_data(time, data):
        """Resample & interpolate all dict elements in data for equal sampling in time

        :return: tuple of (time, data)
        """
        newtime = np.linspace(time[0], time[-1], len(time), dtype=np.float64)
        output = {}
        for key in data:
            output[key] = interp1d(time, data[key])(newtime)
        return (newtime, output)


    def stepcalc(self, time, duration):
        ### calculates frequency and resulting windowlength
        tstep = (time[-1]-time[0])/len(time)
        freq = 1./tstep
        arr_len = duration * freq
        return int(arr_len)

    def winstacker(self, stackdict, flen, superpos):
        ### makes stack of windows for deconvolution
        tlen = len(self.time)
        shift = int(flen/superpos)
        wins = int((tlen-flen)/shift)
        for i in np.arange(wins):
            for key in stackdict.keys():
                stackdict[key].append(self.data[key][i * shift:i * shift + flen])
        for k in stackdict.keys():
            #print('key',k)
            #print(len(stackdict[k]))
            stackdict[k]=np.array(stackdict[k], dtype=np.float64)
        return stackdict

    def wiener_deconvolution(self, setpoint, output, cutfreq):      # setpoint/output are two-dimensional
        pad = 1024 - (len(setpoint[0]) % 1024)                     # padding to power of 2, increases transform speed
        setpoint = np.pad(setpoint, [[0,0],[0,pad]], mode='constant')
        output = np.pad(output, [[0, 0], [0, pad]], mode='constant')
        H = np.fft.fft(setpoint, axis=-1)
        G = np.fft.fft(output,axis=-1)
        freq = np.abs(np.fft.fftfreq(len(setpoint[0]), self.dt))
        sn = self.to_mask(np.clip(np.abs(freq), cutfreq-1e-9, cutfreq))
        len_lpf=np.sum(np.ones_like(sn)-sn)
        sn=self.to_mask(gaussian_filter1d(sn,len_lpf/6.))
        sn= 10.*(-sn+1.+1e-9)       # +1e-9 to prohibit 0/0 situations
        Hcon = np.conj(H)
        deconvolved_sm = np.real(np.fft.ifft(G * Hcon / (H * Hcon + 1./sn),axis=-1))
        return deconvolved_sm

    def stack_response(self, stacks, window):
        inp = stacks['setpoint'] * window
        outp = stacks['result'] * window
        thr = stacks['throttle'] * window

        deconvolved_sm = self.wiener_deconvolution(inp, outp, self.cutfreq)[:, :self.rlen]
        delta_resp = deconvolved_sm.cumsum(axis=1)

        max_thr = np.abs(np.abs(thr)).max(axis=1)
        avr_in = np.abs(np.abs(inp)).mean(axis=1)
        max_in = np.max(np.abs(inp), axis=1)
        avr_t = stacks['time'].mean(axis=1)

        return delta_resp, avr_t, avr_in, max_in, max_thr

    def spectrum(self, time, traces):
        ### fouriertransform for noise analysis. returns frequencies and spectrum.
        pad = 1024 - (len(traces[0]) % 1024)  # padding to power of 2, increases transform speed
        traces = np.pad(traces, [[0, 0], [0, pad]], mode='constant')
        trspec = np.fft.rfft(traces, axis=-1, norm='ortho')
        trfreq = np.fft.rfftfreq(len(traces[0]), time[1] - time[0])
        return trfreq, trspec

    def stackfilter(self, time, trace_ref, trace_filt, window):
        ### calculates filter transmission and phaseshift from stack of windows. Not in use, maybe later.
        # slicing off last 2s to get rid of landing
        #maybe pass throttle for further analysis...
        filt = trace_filt[:-int(Trace.noise_superpos * 2. / Trace.noise_framelen), :] * window
        ref = trace_ref[:-int(Trace.noise_superpos * 2. / Trace.noise_framelen), :] * window
        time = time[:-int(Trace.noise_superpos * 2. / Trace.noise_framelen), :]

        full_freq_f, full_spec_f = self.spectrum(self.time, [self.data['result']])
        full_freq_r, full_spec_r = self.spectrum(self.time, [self.data['debug']])

        f_amp_freq, f_amp_hist =np.histogram(full_freq_f, weights=np.abs(full_spec_f.real).flatten(), bins=int(full_freq_f[-1]))
        r_amp_freq, r_amp_hist = np.histogram(full_freq_r, weights=np.abs(full_spec_r.real).flatten(), bins=int(full_freq_r[-1]))

    def hist2d(self, x, y, weights, bins):   #bins[nx,ny]
        ### generates a 2d hist from setpoint 1d axis for x,y. repeats them to match shape of weights X*Y (data points)
        ### x will be 0-100%
        freqs = np.repeat(np.array([y], dtype=np.float64), len(x), axis=0)
        throts = np.repeat(np.array([x], dtype=np.float64), len(y), axis=0).transpose()
        throt_hist_avr, throt_scale_avr = np.histogram(x, 101, [0, 100])

        hist2d = np.histogram2d(throts.flatten(), freqs.flatten(),
                                range=[[0, 100], [y[0], y[-1]]],
                                bins=bins, weights=weights.flatten(), normed=False)[0].transpose()

        hist2d = np.array(abs(hist2d), dtype=np.float64)
        hist2d_norm = np.copy(hist2d)
        hist2d_norm /=  (throt_hist_avr + 1e-9)

        return {'hist2d_norm':hist2d_norm, 'hist2d':hist2d, 'throt_hist':throt_hist_avr,'throt_scale':throt_scale_avr}


    def stackspectrum(self, time, throttle, trace, window):
        ### calculates spectrogram from stack of windows against throttle.
        # slicing off last 2s to get rid of landing
        result = trace[:-int(Trace.noise_superpos*2./Trace.noise_framelen),:] * window
        thr = throttle[:-int(Trace.noise_superpos*2./Trace.noise_framelen),:] * window
        time = time[:-int(Trace.noise_superpos*2./Trace.noise_framelen),:]

        freq, spec = self.spectrum(time[0], result)

        weights = abs(spec.real)
        avr_thr = np.abs(thr).max(axis=1)

        hist2d=self.hist2d(avr_thr, freq,weights,[101,len(freq)/4])

        filt_width = 3  # width of gaussian smoothing for hist data
        hist2d_sm = gaussian_filter1d(hist2d['hist2d_norm'], filt_width, axis=1, mode='constant')

        # get max value in histogram >100hz
        thresh = 100.
        mask = self.to_mask(freq[:-1:4].clip(thresh-1e-9,thresh))
        maxval = np.max(hist2d_sm.transpose()*mask)

        return {'throt_hist_avr':hist2d['throt_hist'],'throt_axis':hist2d['throt_scale'],'freq_axis':freq[::4],
                'hist2d_norm':hist2d['hist2d_norm'], 'hist2d_sm':hist2d_sm, 'hist2d':hist2d['hist2d'], 'max':maxval}

    def weighted_mode_avr(self, values, weights, vertrange, vertbins):
        ### finds the most common trace and std
        threshold = 0.5  # threshold for std calculation
        filt_width = 7  # width of gaussian smoothing for hist data

        resp_y = np.linspace(vertrange[0], vertrange[-1], vertbins, dtype=np.float64)
        times = np.repeat(np.array([self.time_resp],dtype=np.float64), len(values), axis=0)
        weights = np.repeat(weights, len(values[0]))

        hist2d = np.histogram2d(times.flatten(), values.flatten(),
                                range=[[self.time_resp[0], self.time_resp[-1]], vertrange],
                                bins=[len(times[0]), vertbins], weights=weights.flatten())[0].transpose()
        ### shift outer edges by +-1e-5 (10us) bacause of dtype32. Otherwise different precisions lead to artefacting.
        ### solution to this --> somethings strage here. In outer most edges some bins are doubled, some are empty.
        ### Hence sometimes produces "divide by 0 error" in "/=" operation.

        if hist2d.sum():
            hist2d_sm = gaussian_filter1d(hist2d, filt_width, axis=0, mode='constant')
            hist2d_sm /= np.max(hist2d_sm, 0)


            pixelpos = np.repeat(resp_y.reshape(len(resp_y), 1), len(times[0]), axis=1)
            avr = np.average(pixelpos, 0, weights=hist2d_sm * hist2d_sm)
        else:
            hist2d_sm = hist2d
            avr = np.zeros_like(self.time_resp)
        # only used for monochrome error width
        hist2d[hist2d <= threshold] = 0.
        hist2d[hist2d > threshold] = 0.5 / (vertbins / (vertrange[-1] - vertrange[0]))

        std = np.sum(hist2d, 0)

        return avr, std, [self.time_resp, resp_y, hist2d_sm]

    ### calculates weighted avverage and resulting errors
    def weighted_avg_and_std(self, values, weights):
        average = np.average(values, axis=0, weights=weights)
        variance = np.average((values - average) ** 2, axis=0, weights=weights)
        return (average, np.sqrt(variance))

