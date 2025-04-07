import math
import numpy as np
from scipy import signal as signal
import scipy.stats as stats

# Global variables

# Debug flag
stochastic_debug = False

# Modeling range
# Note: The modeling range must be symmetric around zero to make FFT convolve work accurately
min_dur = -2800
max_dur = 2800

#  Modeling resolution
delta = 5e-1
pmf_size = round((max_dur + delta - min_dur) / delta)

# Accuracy of Min/Max estimates
minmax_perc = 0.01

# Create an array with values from min to max with a given delta offset between values
# Note:
# To make Big grid symmetric, e.g., {-1000.0, -999.9, ..., 999.9, 1000}
# which is very important for the accuracy of FFT convolve,
# the stop argument of np.arange must be higher than 1000.0, otherwise 999.9 becomes the last data point.
big_grid = np.arange(min_dur, max_dur + delta, delta)

# Check symmetry of big grid
if not(abs(big_grid[-1] - max_dur) < delta*1e-2):
    raise Exception("Big grid not symmetric around zero: ", big_grid[-1], max_dur - delta)

# Customize model size
def SetModelSize(new_max_dur, new_delta):
    global min_dur, max_dur, delta, pmf_size, big_grid
    min_dur = -new_max_dur
    max_dur = new_max_dur
    delta = new_delta
    pmf_size = round((max_dur + delta - min_dur) / delta)
    big_grid = np.arange(min_dur, max_dur + delta, delta)
    if not(abs(big_grid[-1] - max_dur) < delta*1e-2):
        raise Exception("Big grid not symmetric around zero: ", big_grid[-1], max_dur - delta)

class StochasticDuration(object):
    def __init__(self):
        # Allow the distribution to be changed
        self.locked = False

        # Default distribution is zero
        self.SetZero()

        if stochastic_debug:
            self.average = 0.0
            self.median = 0.0
            self.min_estimate = 0.0
            self.max_estimate = 0.0

    def Clone(self):
        this = StochasticDuration()
        if not self.zero:
            this.pmf = np.copy(self.pmf)
        this.zero = self.zero
        if stochastic_debug:
            this.average = self.average
            this.median = self.median
            this.min_estimate = self.min_estimate
            this.max_estimate = self.max_estimate
        return this

    def Lock(self):
        self.locked = True

    def UnLock(self):
        self.locked = False

    # '+=' operator
    def __iadd__(self, other):
        self.Conv(other)
        return self

    def SetUniform(self, start_time, duration):
        if not self.locked:
            self.zero = False
            # Save parameters for MC simulations with uniform distributions
            self.start_time = start_time
            self.duration = duration

            distr = stats.uniform(loc= start_time, scale=duration)

            # Probability mass function (PMF), this is the discrete variant of the continuous Probability Density Function (PDF)
            # pmf = pdf*delta
            if duration > 0:
                self.pmf = distr.pdf(big_grid)*delta

            if not self.Normalize():
                # Duration is too short
                # Set a fixed value with probability 1
                self.duration = 0
                idx = (int)(pmf_size/2 * (1 + self.start_time/max_dur))
                if idx == pmf_size // 2:
                    self.SetZero()
                else:
                    self.pmf[idx] = 1

            # Always record median and min/max for modelled distributions
            self.median = self.Median()
            self.min_estimate, self.max_estimate = self.MinMaxEstimate()
        else:
            raise Exception("Can not modify a locked stochastic duration")

    def SetGeneralDistribution(self, pmf_list):
        if not self.locked:
            self.zero = False
            extended_list = [0.0] * (pmf_size - len(pmf_list))
            extended_list.extend(pmf_list)
            self.pmf = np.array(extended_list)
            self.Normalize()
            # Always record median and min/max for modelled distributions
            self.median = self.Median()
            self.min_estimate, self.max_estimate = self.MinMaxEstimate()
        else:
            raise Exception("Can not modify a locked stochastic duration")

    def SetRandomCDF(self, perc_0, perc_100, no_of_segments):
        if not self.locked:
            self.zero = False
            start_index = int((perc_0 - min_dur)/(max_dur - min_dur)*pmf_size)
            end_index = int((perc_100 - min_dur)/(max_dur - min_dur)*pmf_size)
            segment_len = int((end_index - start_index) / no_of_segments)
            average_slope = 1.5 / (no_of_segments * segment_len)
            average_jump = 2.0 / no_of_segments

            cdf = np.empty(pmf_size)
            cdf[0:start_index] = 0.0
            prev_val = 0.0
            for i in range(0, no_of_segments):
                # Random jump
                if np.random.randint(0,10) == 0:
                    prev_val += average_jump

                # Random slope
                random_slope = np.random.uniform(0.0, 2*average_slope)
                for j in range(0, segment_len):
                    next_val = prev_val + random_slope
                    if next_val > 1.0:
                        next_val = 1.0
                    next_index = start_index + i*segment_len + j
                    if next_index >= pmf_size:
                        break
                    cdf[next_index] = next_val
                    prev_val = next_val
                if i == no_of_segments - 1:
                    cdf[start_index+(i+1)*segment_len:-1] = 1.0
                    cdf[-1] = 1.0
            self.pmf = np.gradient(cdf)

            # Always record median and min/max for modelled distributions
            self.median = self.Median()
            self.min_estimate, self.max_estimate = self.MinMaxEstimate()
        else:
            raise Exception("Can not modify a locked stochastic duration")

    def SetNormal(self, mean, std):
        if not self.locked:
            self.zero = False
            distr = stats.norm(loc=mean, scale=std)

            # Probability mass function (PMF), this is the discrete variant of the continuous Probability Density Function (PDF)
            # pmf = pdf*delta
            self.pmf = distr.pdf(big_grid)*delta

            # Always record median and min/max for modelled distributions
            self.median = self.Median()
            self.min_estimate, self.max_estimate = self.MinMaxEstimate()
        else:
            raise Exception("Can not modify a locked stochastic duration")

    def SetZero(self):
        if not self.locked:
            self.zero = True
            # Save parameters for MC simulations
            self.start_time = 0
            self.duration = 0

            # Always record median and min/max for modelled distributions
            self.median = 0.0
            self.min_estimate = 0.0
            self.max_estimate = 0.0
        else:
            raise Exception("Can not modify a locked stochastic duration")
    
    # A general but not efficient method
    # to generate a random number from any distribution
    def RandomObservation(self):
        obs = np.random.choice(big_grid, p=self.pmf)
        return obs

    def Conv(self, sd):
        if not self.locked:
            if not sd.zero:
                if self.zero:
                    # Copy the other distribution
                    self.pmf = np.copy(sd.pmf)
                    self.zero = sd.zero
                    if stochastic_debug:
                        self.average = sd.average
                        self.median = sd.median
                        self.min_estimate = sd.min_estimate
                        self.max_estimate = sd.max_estimate
                else:
                    self.pmf = signal.fftconvolve(self.pmf, sd.pmf,'same')
                    self.zero = False
                    if stochastic_debug:
                        self.average = self.Average()
                        self.median = self.Median()
                        self.min_estimate, self.max_estimate = self.MinMaxEstimate()
        else:
            raise Exception("Can not modify a locked stochastic duration")
        
    def GetNegative(self):
        # Utilize the symmetry
        neg = self.Clone()
        if not self.zero:
            for i in range(0, pmf_size):
                neg.pmf[i] = self.pmf[pmf_size-1-i]
            if stochastic_debug:
                neg.average = neg.Average()
                neg.median = neg.Median()
                neg.min_estimate, neg.max_estimate = self.MinMaxEstimate()
        return neg

    def SetNegative(self):
        if not self.zero:
            # Utilize the symmetry
            self.pmf = np.flip(self.pmf)
            if stochastic_debug:
                self.average = -self.average
                self.median = -self.median
                self.min_estimate = -self.min_estimate
                self.max_estimate = -self.max_estimate

    def Max(self, sd):
        if not self.locked:
            if self.zero:
                if not sd.zero:
                    # Copy the other distribution
                    self.pmf = np.copy(sd.pmf)
                    self.zero = sd.zero
                    # Max with zero
                    self.MaxZeroFast()
            elif sd.zero:
                # Max with zero
                self.MaxZeroFast()
            else:
                start_index = min(np.argmax(self.pmf > 0.000001), np.argmax(sd.pmf > 0.000001))
                fr1 = 0.0
                fr2 = 0.0
                pmf = np.empty(pmf_size)
                pmf[0:start_index] = 0.0
                for i in range(start_index, pmf_size):
                    fr1 += self.pmf[i]
                    fr2 += sd.pmf[i]
                    pmf[i] = fr1*fr2
                    if fr1 > 0.9999 and fr2 > 0.9999:
                        # In practice no further change possible from here.
                        pmf[i+1:pmf_size] = 1.0
                        break
                    elif fr2 > 0.9999 and fr1 < 0.0001:
                        # In practice no change. Keep the current distribution
                        return
                    elif fr1 > 0.9999 and fr2 < 0.0001:
                        # In practice the same distribution as the operand. Copy the operand
                        self.pmf = np.copy(sd.pmf)
                        self.zero = sd.zero
                        if stochastic_debug:
                            self.average = sd.average
                            self.median = sd.median
                            self.min_estimate = sd.min_estimate
                            self.max_estimate = sd.max_estimate
                        return
                self.pmf = np.gradient(pmf)
                self.zero = False
            if stochastic_debug:
                self.average = self.Average()
                self.median = self.Median()
                self.min_estimate, self.max_estimate = self.MinMaxEstimate()
        else:
            raise Exception("Can not modify a locked stochastic duration")

    def MaxZeroFast(self):
        if not self.locked:
            if not self.zero:
                zero_idx = pmf_size // 2
                start_index = min(zero_idx, np.argmax(self.pmf > 0.000001))
                fr = np.sum(self.pmf[start_index:zero_idx])
                self.pmf[start_index:zero_idx] = 0.0
                if fr > 0.999:
                    self.zero = True
                self.pmf[zero_idx] += fr
            if stochastic_debug:
                self.average = self.Average()
                self.median = self.Median()
                self.min_estimate, self.max_estimate = self.MinMaxEstimate()
        else:
            raise Exception("Can not modify a locked stochastic duration")

    def MaxZeroSlow(self):
        if not self.locked:
            if not self.zero:
                sd = StochasticDuration()
                sd.SetZero()
                fr1 = 0.0
                fr2 = 0.0
                pmf = self.pmf.copy()
                for i in range(0, pmf_size):
                    fr1 += self.pmf[i]
                    fr2 += sd.pmf[i]
                    pmf[i] = fr1*fr2
                self.pmf = np.gradient(pmf)
                self.zero = False
                if stochastic_debug:
                    self.average = self.Average()
                    self.median = self.Median()
                    self.min_estimate, self.max_estimate = self.MinMaxEstimate()
        else:
            raise Exception("Can not modify a locked stochastic duration")

    def KullbackLeiblerDivergence(self, other):
        start = np.argmax(other.pmf > 0.000001)
        end = start + np.argmax(other.pmf[start:] < 0.000001)
        div = 0.0
        for i in range(start, end):
            div += self.pmf[i] * math.log(self.pmf[i] / other.pmf[i])
        return div

    def StatDist(self, other):
        dist = 0.0
        for i in range(0, pmf_size):
            dist += abs(self.pmf[i] - other.pmf[i])
        dist = dist / 2
        return dist

    def MultiMax(self, sds):
        if not self.locked:
            non_empty = []
            if not self.zero:
                non_empty.append(self)
            for sd in sds:
                if not sd.zero:
                    non_empty.append(sd)
            if len(non_empty) == 1:
                self.pmf = np.copy(non_empty[0].pmf)
                self.zero = non_empty[0].zero
            elif len(non_empty) > 1:
                start_index = 0
                for sd in non_empty:
                    start_index = min(np.argmax(sd.pmf > 0.000001), start_index)
                pmf = np.empty(pmf_size)
                first = non_empty[0]
                fr1 = 0.0
                fr2 = [0.0] * (len(non_empty)-1)
                pmf[0:start_index] = 0.0
                for i in range(start_index, pmf_size):
                    fr1 += first.pmf[i]
                    pmf[i] = fr1
                    for j in range(1, len(non_empty)):
                        fr2[j-1] += non_empty[j].pmf[i]
                        pmf[i] = pmf[i]*fr2[j-1]
                    if pmf[i] > 0.9999:
                        # In practice no further change possible from here.
                        pmf[i+1:pmf_size] = 1.0
                        break
                self.pmf = np.gradient(pmf)
                self.zero = False
            if stochastic_debug:
                self.average = self.Average()
                self.median = self.Median()
                self.min_estimate, self.max_estimate = self.MinMaxEstimate()
        else:
            raise Exception("Can not modify a locked stochastic duration")

    def MultiMaxLow(self, sds):
        if not self.locked:
            non_empty = []
            if not self.zero:
                non_empty.append(self)
            for sd in sds:
                if not sd.zero:
                    non_empty.append(sd)
            if len(non_empty) == 1:
                self.pmf = np.copy(non_empty[0].pmf)
                self.zero = non_empty[0].zero
            elif len(non_empty) > 1:
                start_index = 0
                for sd in non_empty:
                    start_index = min(np.argmax(sd.pmf > 0.000001), start_index)
                pmf = np.empty(pmf_size)
                first = non_empty[0]
                fr1 = 0.0
                fr2 = [0.0] * (len(non_empty)-1)
                pmf[0:start_index] = 0.0
                for i in range(start_index, pmf_size):
                    fr1 += first.pmf[i]
                    pmf[i] = fr1
                    for j in range(1, len(non_empty)):
                        fr2[j-1] += non_empty[j].pmf[i]
                        pmf[i] = min(pmf[i],fr2[j-1])
                    if pmf[i] > 0.9999:
                        # In practice no further change possible from here.
                        pmf[i+1:pmf_size] = 1.0
                        break
                self.pmf = np.gradient(pmf)
                self.zero = False
            if stochastic_debug:
                self.average = self.Average()
                self.median = self.Median()
                self.min_estimate, self.max_estimate = self.MinMaxEstimate()
        else:
            raise Exception("Can not modify a locked stochastic duration")

    def Dominator(self, sd):
        fr1 = 0.0
        fr2 = 0.0
        dominator = None
        for i in range(0, pmf_size):
            fr1 += self.pmf[i]
            fr2 += sd.pmf[i]
            # eps = 1e-6
            eps = 1e-3
            if (fr1 > eps or fr2 > eps):
                if (fr1 < 1-eps or fr2 < 1-eps):
                    # Check the first order domination criteria
                    # For domination, the cumulative distribution should be less for all values
                    if fr1 <= fr2:
                        if dominator != sd:
                            # This distribution dominates the other (so far)
                            dominator = self
                        else:
                            # Domination criteria does not hold for all values
                            dominator = None
                            break
                    else:
                        if dominator != self:
                            # The other distribution dominates this distribution (so far)
                            dominator = sd
                        else:
                            # Domination criteria does not hold for all values
                            dominator = None
                            break
                else:
                    break

        return dominator

    def AtPercentile(self, perc):
        if self.zero:
            return 0.0
        fr = 0.0
        interpolated_fr_id = -1
        if perc != None:
            if perc < 0.0 or perc > 1.0:
                print("WARNING: percentile outside range")
        for i in range(0, pmf_size):
            fr += self.pmf[i]
            if fr >= perc:
                # Linear interpolation to get a more accurate estimation of the percentile
                interpolated_fr_id = i - (fr - perc)/self.pmf[i]
                break
        if interpolated_fr_id != -1:
            return min_dur + delta*interpolated_fr_id
        else:
            raise Exception("Not possible to compute percentile")

    def Median(self):
        return self.AtPercentile(0.5)

    def AtPercentile50(self):
        return self.AtPercentile(0.5)

    def Stddev(self):
        sd = 0.0
        mu_sq = self.Average()**2
        for i in range(0, pmf_size):
            sd += (min_dur + delta*i)**2*self.pmf[i]
        sd = math.sqrt(sd - mu_sq)
        return sd

    def Average(self):
        av = 0.0
        if not self.zero:
            for i in range(0, pmf_size):
                av += (min_dur + delta*i)*self.pmf[i]
        self.average = av
        return av

    def MaxEstimate(self):
        if self.zero:
            return 0.0
        else:
            start_index = np.argmax(self.pmf > 0.000001)
            fr = 0.0
            fr_id = -1
            for i in range(start_index, pmf_size):
                fr += self.pmf[i]
                if fr > 0.99:
                    fr_id = i
                    break
            if fr_id != -1:
                return min_dur + delta*fr_id
            else:
                raise Exception("Not possible to calculate max estimate, perc = "+str(fr))

    def MinMaxEstimate(self, dummy = None):
        if self.zero:
            return 0.0, 0.0
        else:
            pmf_bool = self.pmf > 0.000001
            start_id = np.argmax(pmf_bool)
            stop_id = len(self.pmf) - 1 - np.argmax(np.flip(pmf_bool))
            start_dur = min_dur + delta * start_id
            cum_sum = np.cumsum(self.pmf[start_id:stop_id+1])
            min_id = np.argmax(cum_sum > minmax_perc)
            max_id = np.argmax(cum_sum > 1 - minmax_perc)
            if max_id < min_id:
                raise Exception("MinMaxEstimate failed. The stochastic variable range may be too small")
            return start_dur + delta * min_id, start_dur + delta * max_id

    def MinEstimate(self):
        if self.zero:
            return 0.0
        else:
            start_index = np.argmax(self.pmf > 0.000001)
            fr = 0.0
            fr_id = -1
            for i in range(start_index, pmf_size):
                fr += self.pmf[i]
                if fr > 0.01:
                    fr_id = i
                    break
            if fr_id != -1:
                return min_dur + delta*fr_id
            else:
                raise Exception("Not possible to calculate max estimate")

    def GetCumulative(self):
        fr = 0.0
        cumul = []
        for i in range(0, pmf_size):
            fr += self.pmf[i]
            cumul.append(fr)
        return cumul

    def Normalize(self):
        ok = True
        if not self.zero:
            if hasattr(self, "pmf"):
                sum = np.sum(self.pmf)
                if sum > 0:
                    self.pmf /= sum
                else:
                    ok = False
            else:
                ok = False
        return ok

