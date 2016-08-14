#!/usr/bin/env python

import inspect


class Callbacker(object):
    def __init__(self):
        self._next_id = 0
        self._callbacks = {}

    def register(self, function):
        cbid = self._next_id
        self._callbacks[cbid] = function
        self._next_id += 1
        return cbid

    def unregister(self, cbid):
        if cbid not in self._callbacks:
            pass
        del self._callbacks[cbid]

    def __call__(self, *args, **kwargs):
        for cbid in self._callbacks:
            self._callbacks[cbid](*args, **kwargs)


class KWargCallbacker(Callbacker):
    def register(self, function, *args, **kwargs):
        cbid = self._next_id
        # check if function accepts varkwargs
        argspec = inspect.getargspec(function)
        if argspec.keywords is None:
            # if not, don't pass through keyword arguments
            kwargs = None
        self._callbacks[cbid] = (function, args, kwargs)
        self._next_id += 1
        return cbid

    def __call__(self, *args, **kwargs):
        for cbid in self._callbacks:
            cb, cbargs, cbkwargs = self._callbacks[cbid]
            if cbkwargs is not None:
                kw = {}
                kw.update(cbkwargs)
                kw.update(kwargs)
                cb(*(args + cbargs), **kw)
            else:
                cb(*(args + cbargs))
