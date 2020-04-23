from sympy.solvers import solve
from sympy import Symbol
from scipy import signal 
import numpy as np 
import math 
import itertools

class tool_box_tdoa(object):
    def __init__(self):
        self.tmp = 0
    """
                    degree = 0
                        ^   North
                        |   
                        |
                        |
    degree = 270 --------------- degree = 90
                        |
                        |
                        |
                    degree = 180
    """
    def tdoaWithThreeMics(self, x0, y0, x1, y1, t1, x2, y2, t2, x3, y3, t3):
        x, y = symbols('x, y')
        eq1 = ((x-x1)**2.0+(y-y1)**2.0)**0.5-((x-x3)**2.0+(y-y3)**2.0)**0.5-self.c*(t3-t1)
        eq2 = ((x-x1)**2.0+(y-y1)**2.0)**0.5-((x-x2)**2.0+(y-y2)**2.0)**0.5-self.c*(t2-t1)
        eq3 = ((x-x2)**2.0+(y-y2)**2.0)**0.5-((x-x3)**2.0+(y-y3)**2.0)**0.5-self.c*(t3-t2)
        roots = solve([eq1, eq2, eq3], [x, y])
        return self.seekRoots(roots)

    def seekRoots(self, roots):
        if roots:
            if roots[0][0].is_Float: 
                if len(positions) > 1:
                   angle_one = self.angleP2P(positions[0], x0, y0)
                   angle_two = self.angleP2P(positions[1], x0, y0)
                   return "real", [angle_one, angle_two]
                else:
                   angle_one = self.angleP2P(positions[0], x0, y0)
                   return "real", [angle_one]
            else:
                return ("complex", [None])
        else:
            #rospy.loginfo("roots is empty.")
            return ("empty", [None])

    def angleP2P(self, pos, x0, y0):
        x, y = pos
        a = 90-math.degrees(math.atan2(y-y0, x-x0))
        #if a < 0:
        #    a = a+360
        return round(a, 1)

    def twoMicsAngle(self, mic_dl, t1, t2, c, heading=0):
        dt = t1-t2
        #if(t2>t1):
         #   dt = t2-t1
        #else:
         #   dt = t1-t2
        if (c*dt/mic_dl > 1 or c*dt/mic_dl < -1):
            return ("empty", 400)
        theta = 90-math.acos(c*dt/mic_dl)*180/math.pi
        #if(theta > 90):
         #   theta = theta-90
        #elif(theta < -90):
         #   theta = theta+90
        #if(t2>t1):
         #   theta = -theta
        theta = theta+heading*180/math.pi
        #if theta < 0:
        #    theta = theta+360
        return ("two", round(theta, 1))

    def corrXT(self, a_data, b_data, r, c, fs, buffer_len=1):
        a = a_data.copy()
        b = b_data.copy()
        la = len(a)
        lb = len(b)
        d = int(math.ceil(r/c*fs*buffer_len))
        tmp = np.zeros(d*2-1)
        for i in range(d):
            tmp[d+i-1] = np.sum(a[i:]*b[:lb-i])
            tmp[d-i-1] = np.sum(b[i:]*a[:la-i])
            if i != 0:
                tmp[d+i-1] = tmp[d+i-1]+np.sum(a[:i]*b[lb-i:])
                tmp[d-i-1] = tmp[d-i-1]+np.sum(b[:i]*a[la-i:])
        maxArgu = np.argmax(tmp)
        real_recevie_time = round((maxArgu-float(d)+1)/fs, 16)
        return real_recevie_time

    def corrXTForward(self, a_data, b_data, r, c, fs, buffer_len=1):
        a = a_data.copy()
        b = b_data.copy()
        la = len(a)
        lb = len(b)
        d = int(math.ceil(r/c*fs*buffer_len))
        tmp = np.zeros(d)
        for i in range(d):
            tmp[i] = np.sum(a[i:]*b[:lb-i])
            if i != 0:
                tmp[i] = tmp[i]+np.sum(a[:i]*b[lb-i:])
        maxArgu = np.argmax(tmp)
        real_recevie_time = round((maxArgu+1)/fs, 16)
        return real_recevie_time
    def whistle_detector(self, input_data, fs, window, N, overlap, SNR_threshold, lowCutOff, highCutOff):
        # Fourier transform 
        t, f, tmp = self.STFT(input_data, fs, window, N, overlap)
        # use median filter to avoid the short-term noise.
        tmp = self.median_filter(tmp)
        # To find the signal is bigger than SNR threshold or not.
        tmp = self.edge_detector(tmp, SNR_threshold)
        # To find the whistle which is between lowcutoff and highcutoff is continue on time and freq domain or not.
        tmp = self.moving_square(tmp, fs, N, overlap, lowCutOff, highCutOff)
        return t, f, tmp

    def STFT(self, input_data, fs, window, N, overlap):
        f, t, Sxx = signal.spectrogram(input_data, fs, window, N, overlap)
        return f, t, Sxx

    def median_filter(self, input_data, median_size=3):
        medfilter_result = input_data.copy()
        median_len = int(median_size/2)
        medfilter_tmp = np.zeros([input_data.shape[0]-median_len*2, input_data.shape[1]-median_len*2, median_size**2])
        i_start, i_end = median_len, int(input_data.shape[0]-median_len)
        j_start, j_end = median_len, int(input_data.shape[1]-median_len)
        xy_list = range(-median_len, median_len+1)
        k = 0
        for xx, yy in itertools.product(xy_list, xy_list):
            medfilter_tmp[i_start:i_end, j_start:j_end, k] = input_data[i_start+xx:i_end+xx-median_len, j_start+yy:j_end+yy-median_len]
            k = k+1
            if k >= median_size**2:
                k = 0
        medfilter_result[i_start:i_end, j_start:j_end] = np.median(medfilter_tmp, axis=2)

        return medfilter_result

    def edge_detector(self, input_data, SNR_threshold, jump_number=3):
        detect_result = np.zeros(input_data.shape)
        tmp = input_data[jump_number:input_data.shape[0]-jump_number, :]
        row_len = tmp.shape[0]
        moving_jump_number = input_data[:row_len, :]*0.5+input_data[jump_number*2:, :]*0.5
        #ignore the error of divid by 0
        with np.errstate(all='ignore'):
            SNR = 10.0*np.log(tmp/(moving_jump_number)) 
            SNR_i, SNR_j  = np.where(SNR > SNR_threshold)
        SNR_i = SNR_i+jump_number
        detect_result[SNR_i, SNR_j] = 1

        return detect_result

    def moving_square(self, input_data, fs, N, overlap, lowCutOff, highCutOff):
        moving_result = np.zeros(input_data.shape)
        time_width = 0.01
        percent_threshold = 0.5
        bandwidth_sample = 4
        time_width_sample = 12
        #the length and width of the square are bandwidth_sample and time_width_sample.

        if(bandwidth_sample%2 == 0):
            bandwidth_sample = bandwidth_sample-1
        if(time_width_sample%2 == 0):
            time_width_sample = time_width_sample+1

        real_number_threshold = percent_threshold* bandwidth_sample* time_width_sample
        
        i_start, i_end = int(N* lowCutOff/fs), int(N* highCutOff/fs-(bandwidth_sample-1)/2)
        j_start, j_end = int((time_width_sample-1)/2), int(input_data.shape[1]-((time_width_sample-1)/2))
        xx_list = range(-(bandwidth_sample-1)/2, ((bandwidth_sample-1)/2)+1)
        yy_list = range(-(time_width_sample-1)/2, ((time_width_sample-1)/2)+1)

        #the sum of square must be bigger than real_number_threshold.
        #if bigger than real_number_threshold than the data in square will be remained
        #else the data in square will be zero.
        threshold_i, threshold_j = self.pos_of_sum_of_each_square_bigger_than_thershold(input_data, i_start, i_end, j_start, j_end, xx_list, yy_list, real_number_threshold)

        for xx, yy in itertools.product(xx_list, yy_list):
            moving_result[threshold_i+xx, threshold_j+yy] = input_data[threshold_i+xx, threshold_j+yy]

        return moving_result 

    def pos_of_sum_of_each_square_bigger_than_thershold(self, input_data, i_start, i_end, j_start, j_end, xx_list, yy_list, real_number_threshold):
        square_area = np.zeros([i_end-i_start, j_end-j_start])

        for xx, yy in itertools.product(xx_list, yy_list):
            square_area = square_area+input_data[i_start+xx:i_end+xx, j_start+yy:j_end+yy]

        threshold_i, threshold_j = np.where(square_area >= real_number_threshold)
        threshold_i = threshold_i+i_start
        threshold_j = threshold_j+j_start
        return threshold_i, threshold_j

    def exist_whistle_or_not(self, input_data, duration_threshold, dt):
        whistle_duration = self.find_the_whistle_duration(input_data)
        if whistle_duration.any() == -1:
            return False, 0
        duration_not_zero = whistle_duration[np.where(whistle_duration != 0)].astype(float)
        max_duration = np.max(duration_not_zero)
        if max_duration*dt >= duration_threshold:
            return True, round(max_duration*dt, 8)
        else:
            return False, round(max_duration*dt, 8)
        '''
        for duration in duration_not_zero:
            if duration*dt >= duration_threshold:
                return True, round(duration*dt, 8)
            else:
                return False, round(duration*dt, 8)
        '''

    def find_the_whistle_duration(self, input_data):
        whistle_pos_i, whistle_pos_j = np.where(input_data == 1)
        whistle_pos_j = np.unique(whistle_pos_j)
        whistle_len = len(whistle_pos_j)
        if whistle_len == 0:
            return np.array([-1])
        whistle_start = [0]
        whistle_end = []
        for i in xrange(whistle_len):
            if i+1 < whistle_len and whistle_pos_j[i+1] != whistle_pos_j[i]+1:
                whistle_start.append(i+1)
                whistle_end.append(i)
        whistle_end.append(whistle_len-1)
        whistle_duration = whistle_pos_j[whistle_end]-whistle_pos_j[whistle_start]
        return whistle_duration
    
    def butter_bandpass_filter(self, data, fs, highPassCutoff, lowPassCutoff, order=5):
        b, a = self.butter_bandpass(highPassCutoff, lowPassCutoff, fs, order)
        y = signal.filtfilt(b, a, data)
        return y

    def butter_bandpass(self, highPassCutoff, lowPassCutoff, fs, order):
        nyq = 0.5 * fs
        nyq_highPassCutoff = highPassCutoff/nyq
        nyq_lowPassCutoff = lowPassCutoff/nyq
        if highPassCutoff == 0:
            b, a = signal.butter(order, nyq_lowPassCutoff, btype='lowpass', analog=False)
        elif lowPassCutoff == 0:
            b, a = signal.butter(order, nyq_highPassCutoff , btype='highpass', analog=False)
        else:
            b, a = signal.butter(order, [nyq_highPassCutoff ,nyq_lowPassCutoff], btype='bandpass', analog=False)
        return b, a
