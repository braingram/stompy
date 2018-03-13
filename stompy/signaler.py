#!/usr/bin/env python


class Signaler(object):
    def __init__(self):
        self._callbacks = {}

    def on(self, name, func):
        self._callbacks[name] = self._callbacks.get(name, []) + [func, ]

    def remove_on(self, name, func):
        if name not in self._callbacks:
            return
        if func not in self._callbacks[name]:
            return
        self._callbacks[name].remove(func)

    def trigger(self, name, *args, **kwargs):
        for cbf in self._callbacks.get(name, []):
            cbf(*args, **kwargs)
