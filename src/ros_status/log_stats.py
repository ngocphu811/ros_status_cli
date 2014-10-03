#!/usr/bin/env python

from enum import Enum
from datetime import *

class LogType(Enum):
    DEBUG = 1
    INFO  = 2
    WARN  = 4
    ERR   = 8
    FATAL = 16

def get_now():
    return datetime.now()

class LogStat(object):
    def __init__(self, logtype):
        self.lastupdate = get_now()
        self._count = 0
        self.logtype = logtype

    @property
    def count(self):
        return self._count
    
    def inc_count(self):
        self.lastupdate = get_now()
        self._count = self._count + 1

class LogStats(object):

    def __init__(self):
        self._debug = LogStat(LogType.DEBUG)
        self._info  = LogStat(LogType.INFO)
        self._warn  = LogStat(LogType.WARN)
        self._err   = LogStat(LogType.ERR)
        self._fatal = LogStat(LogType.FATAL)

    def __getitem__(self, log_type):
        if not isinstance(log_type, LogType):
            raise KeyError("the key is not an object of " + LogType.__class__)
        if log_type == LogType.DEBUG:
            return self._debug
        elif log_type == LogType.INFO:
            return self._info
        elif log_type == LogType.WARN:
            return self._warn
        elif log_type == LogType.ERR:
            return self._err
        elif log_type == LogType.FATAL:
            return self._fatal
        else: 
            raise KeyError("wrong key: " + str(log_type))

    @property
    def debug(self): 
        return self._debug

    @property
    def info(self): 
        return self._info

    @property
    def warn(self): 
        return self._warn

    @property
    def err(self): 
        return self._err

    @property
    def fatal(self): 
        return self._fatal

