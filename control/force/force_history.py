"""Bounded, unfiltered sensor history; display reduction preserves force spikes."""
from collections import deque

import numpy as np


CHANNELS = ('compensated_force_N', 'applied_command_force_N', 'force_target_N',
            'Fx_N', 'Fy_N', 'Fz_N', 'Tx_Nm', 'Ty_Nm', 'Tz_Nm')


class ForceHistory:
    def __init__(self, window_s=10., sample_rate_hz=1000.):
        if not np.isfinite([window_s, sample_rate_hz]).all() or min(window_s, sample_rate_hz) <= 0:
            raise ValueError('Positive finite history window and sample rate required')
        self.window_s = float(window_s)
        self.samples = deque(maxlen=int(np.ceil(window_s*sample_rate_hz))+2)

    def append(self, row):
        sample = [float(row['time_s']), *[float(row[key]) for key in CHANNELS]]
        if not np.isfinite(sample).all():
            raise ValueError('Nonfinite display sample')
        if self.samples and sample[0] < self.samples[-1][0]:
            raise ValueError('Display time must be monotonic')
        if self.samples and sample[0] == self.samples[-1][0]:
            self.samples[-1] = sample
        else:
            self.samples.append(sample)
        while self.samples and self.samples[0][0] < sample[0]-self.window_s-1e-9:
            self.samples.popleft()

    def arrays(self):
        if not self.samples:
            return np.empty((0, len(CHANNELS)+1))
        return np.asarray(self.samples)


def peak_envelope(times, values, start, end, pixels):
    """Min/max per screen column, retaining single-sample peaks and dropouts."""
    columns = np.floor((np.asarray(times)-start)/max(end-start, 1e-12)*(pixels-1)).astype(int)
    columns = np.clip(columns, 0, pixels-1)
    low, high = np.full(pixels, np.inf), np.full(pixels, -np.inf)
    np.minimum.at(low, columns, values)
    np.maximum.at(high, columns, values)
    occupied = np.flatnonzero(np.isfinite(low))
    return occupied, low[occupied], high[occupied]
