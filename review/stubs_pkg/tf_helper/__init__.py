# Minimal stand-in for the (now unavailable) ROS1 tf_helper package.
# Provides only the names fs_planning-2022 pulls in via `from tf_helper import *`.
import numpy as np
from functools import wraps

def mutex_lock(lock):
    def deco(fn):
        @wraps(fn)
        def wrapper(*a, **k):
            with lock:
                return fn(*a, **k)
        return wrapper
    return deco
