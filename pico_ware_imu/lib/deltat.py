import time

class DeltaT:
    def __init__(self, timediff=None):
        if timediff is None:
            self.expect_ts = False
            self.timediff = lambda start, end: end - start
        else:
            self.expect_ts = True
            self.timediff = timediff
        self.start_time = None

    def __call__(self, ts=None):
        if self.expect_ts:
            if ts is None:
                raise ValueError('Timestamp expected but not supplied.')
        else:
            ts = time.monotonic()
        
        # ts is now valid
        if self.start_time is None:  # 1st call: self.start_time is invalid
            self.start_time = ts
            return 0.0001  # 100μs notional delay. 1st reading is invalid in any case

        dt = self.timediff(ts, self.start_time)
        self.start_time = ts
        return dt

# Example usage
delta_t = DeltaT()
print(delta_t())  # First call, returns 0.0001
time.sleep(1)
print(delta_t())  # Subsequent call, returns time difference in seconds