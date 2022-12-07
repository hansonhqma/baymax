from collections import deque
import math

class controller:
    def __init__(self, p, i, d, target, epsilon):
        self._pcoeff = p
        self._icoeff = i
        self._dcoeff = d
        self._mem = deque(maxlen=50)
        self._target = target
        self._prev_error = None
        self._converged = False
        self._eps = epsilon

    def update_target(self, newtarget):
        self._target = newtarget
    
    def gain(self, pos):
        error = self._target - pos
        if not self._prev_error == None and math.fabs(error) < self._eps and sum(self._mem) < 1e-5 and self._prev_error < 1e-5:
            self._converged = True
            return 0
        if self._prev_error == None:
            delta_e = 0
            self._prev_error = error
        else:
            delta_e = error-self._prev_error
            self._prev_error = error

        pgain = self._pcoeff*error
        igain = self._icoeff*sum(self._mem)
        dgain = self._dcoeff*delta_e

        self._mem.append(error)
        
        return pgain+igain+dgain
        