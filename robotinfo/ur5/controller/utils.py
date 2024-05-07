import copy
import ctypes
import math
import multiprocessing as mp
from klampt.math import vectorops
from klampt.robotsim import equilibrium_torques
import ur5_constants
from typing import Union, Dict, Tuple, Any
import warnings

def clamp_limits(q, min_limits, max_limits):
    """Clamps a configuration to limits."""
    return vectorops.minimum(vectorops.maximum(q,min_limits),max_limits)



def in_limits(q, min_limits, max_limits):
    """
    Check if a configuration is within limits.
    Parameters:
        q:          Configuration to check.
        min_limits: Minimum joint limits. Default of None will auto-fail.
        max_limits: Maximum joint limits. Default of None will auto-fail.
    Return:
        True if every number `q[i]` is between `min_limits[i]` and `max_limits[i]`, inclusive; else False.
    """
    for qi, a, b in zip(q, min_limits, max_limits):
        if qi < a or qi > b:
            return False
    return True

class _EmptyContext:
    def __enter__(self):
        return self
    def __exit__(self,type,value,tb):
        pass

class SharedMap:
    """
    A dict-like object that copies to/from a shared memory buffer. Used for
    real-time communication between processes.  Allows copying ints, floats,
    and lists of ints or floats.
    
    In particular, very useful for UR5 RTDE communication.
    """

    def __init__(self, bindings : Union[Dict[str,Tuple], 'SharedMap', Any], lock : bool = True):
        """
        Create a shared memory (cross process usable) buffer.

        Parameters:
            - bindings (dict, object, or SharedMap): A dictionary mapping from name
                str -> (class,size).
                class is either int or float, size is 0 (scalar) or
                >= 1 (array)
                Or, it can be an object containing floats, ints, or lists
                of floats or ints.  In this case, it will be compatible
                with copy_to_object / copy_from_object.
            - lock (bool): if true, will automatically lock internal buffers on any access.  
                Otherwise, user will need to use ``with sm.lock():`` to protect access
        """
        self._lock = mp.Lock()
        self._auto_lock = lock
        self.locked = False
        int_idx = 0
        float_idx = 0
        # Mapping from keynames to triples (array, start, size).
        self.names_to_starts = {}
        int_items = []
        float_items = []
        if isinstance(bindings,dict):
            for k, v in bindings.items():
                class_, size_ = v
                if class_ == float:
                    float_items.append((k, float_idx, size_))
                    float_idx += size_ if size_ else 1
                elif class_ == int:
                    int_items.append((k, int_idx, size_))
                    int_idx += size_ if size_ else 1
                else:
                    print("Unrecognized type {} for name {}, skipping".format(str(class_), k))
                    continue
        elif isinstance(bindings,SharedMap):
            for k,v in bindings.names_to_starts.items():
                dat, start, size = v
                if dat is bindings.ints:
                    int_items.append((k, start, size))
                    int_idx = max(int_idx,start + size)
                else:
                    assert dat is bindings.floats
                    float_items.append((k, start, size))
                    float_idx = max(float_idx,start + size)
        else:
            for k,v in bindings.__dict__.items():
                if isinstance(v,float):
                    float_items.append((k, float_idx, 0))
                    float_idx += 1
                elif isinstance(v,int):
                    int_items.append((k, int_idx, 0))
                    int_idx += 1
                elif isinstance(v,list) and len(v) > 0:
                    if isinstance(v[0],float):
                        float_items.append((k, float_idx, len(v)))
                        float_idx += len(v)
                    elif isinstance(v[0],int):
                        int_items.append((k, int_idx, len(v)))
                        int_idx += len(v)
                
        self.ints = mp.Array(ctypes.c_int, int_idx, lock=False)
        self.floats = mp.Array(ctypes.c_double, float_idx, lock=False)

        for k, idx, size in int_items:
            self.names_to_starts[k] = (self.ints, idx, size)
        for k, idx, size in float_items:
            self.names_to_starts[k] = (self.floats, idx, size)
        
        if isinstance(bindings,SharedMap):
            self.ints[:] = bindings.ints[:]
            self.floats[:] = bindings.floats[:]
        elif not isinstance(bindings,dict):
            self.copy_from_object(bindings)

    def lock(self):
        """
        Request for the lock for this shared memory.  Use the syntax

            with sm.lock():
                item = sm[key]
        
        to avoid deadlocks on exceptions.
        """
        if self._auto_lock:
            warnings.warn("SharedMap is auto-locking, but lock() is requested")
        return self._lock
    
    def _lock_context(self):
        if self._auto_lock:
            return self._lock
        else:
            return _EmptyContext()
        
    def lock_acquire(self):
        if self.locked:
            warnings.warn("Attempting to lock an already locked process")
        self.locked = True
        self._lock.acquire()

    def lock_release(self):
        if not self.locked:
            warnings.warn("No lock to release")
        else:
            self.locked = False
            self._lock.release()
            
    def keys(self):
        return self.names_to_starts.keys()

    def __contains__(self,key):
        return key in self.names_to_starts

    def get(self, key, defaultValue=None):
        try:
            return self[key]
        except KeyError:
            return defaultValue

    def __getitem__(self, key):
        """
        Get an entry in this shared memory.

        Parameters:
            - key:  name of entry to get.
        Return:
            Single value in the case of size=0 entries, list of values otherwise.
        """
        dat, start, size = self.names_to_starts[key]
        with self._lock_context():
            if size == 0:
                return dat[start]
            return dat[start : start + size]
        

    def __setitem__(self, key, val):
        """
        Set an entry in this shared memory.
        Cannot create new entries. Entries must be created when the buffer is initialized

        Parameters:
            - key:  name of entry to set.
            - val:  int, double for size=0 entries. Iterable for larger entries.
                    NOTE: No bounds checks are performed.
        Return:
            None
        """
        dat, start, size = self.names_to_starts[key]
        with self._lock_context():
            if size == 0:
                dat[start] = val
            else:
                dat[start : start + size] = val
    
    def copy_to_dict(self):
        """Produces a standard dict containing all the objects."""
        with self._lock_context():
            res = dict()
            for k, v in self.names_to_starts.items():
                dat, start, size = v
                if size == 0:
                    res[k] = dat[start]
                else:
                    res[k] = dat[start:start+size]
            return res
    
    def copy_from_dict(self,obj,strict=True):
        """Copies items from obj.  If strict=True, all the keys must be in dict."""
        if strict and len(obj) != len(self.names_to_starts):
            raise ValueError("Invalid dict, doesn't match keys")
        with self._lock_context():
            for k, v in self.names_to_starts.items():
                if k not in obj:
                    if strict:
                        raise ValueError("Invalid dict, doesn't have key {}".format(k))
                    continue
                dat, start, size = v
                if size == 0:
                    dat[start] = obj[k]
                else:
                    dat[start:start+size] = obj[k]

    def copy_from_object(self,obj):
        """
        Copy attributes from obj into this SharedMap assuming each key corresponds
        to an attribute name.
        """
        with self._lock_context():
            for k, v in self.names_to_starts.items():
                if k in obj.__dict__:
                    dat, start, size = v
                    if size == 0:
                        dat[start] = obj.__dict__[k]
                    else:
                        dat[start:start+size] = obj.__dict__[k]
    
    def copy_to_object(self,obj):
        """
        Copy attributes from this SharedMap to obj, assuming each key corresponds
        to an attribute name.
        """
        with self._lock_context():
            for k, v in self.names_to_starts.items():
                if k in obj.__dict__:
                    dat, start, size = v
                    if size == 0:
                        obj.__dict__[k] = dat[start]
                    else:
                        obj.__dict__[k] = dat[start:start+size]

class _Test:
    def __init__(self):
        self.a = 4
        self.b = 3.4
        self.c = [1,2,3]
        self.d = [0.1]
        self.e = "hello"

def self_test():
    """Tests SharedMap"""
    sm = SharedMap(_Test())
    print(sm.keys())
    foo = _Test()
    sm.ints[0] = 1
    sm.copy_to_object(foo)
    foo.b = 5.6
    sm.copy_from_object(foo)
    print(sm.ints[:])
    print(sm.floats[:])