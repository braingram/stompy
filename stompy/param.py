#!/usr/bin/env python

import cPickle as pickle

from . import signaler


class Meta(object):
    """Meta information about a parameter (range, values, etc)"""
    def __init__(self, **kwargs):
        """kwargs are used for is_valid check and for QInputDialog"""
        self.kwargs = kwargs

    def __getitem__(self, name):
        return self.kwargs[name]

    def is_valid(self, value):
        if isinstance(value, int):
            # for int look for min, max, step
            if value < self.kwargs.get('min', value):
                return False
            if value > self.kwargs.get('max', value):
                return False
            # TODO step?
            return True
        elif isinstance(value, float):
            # for float look for min, max, decimals
            if value < self.kwargs.get('min', value):
                return False
            if value > self.kwargs.get('max', value):
                return False
            # TODO decimal
            return True
        else:
            pass
        # for item?
        # for string?
        return True


class Param(signaler.Signaler):
    def __init__(self, filename=None):
        super(Param, self).__init__()
        self._params = {}
        self._meta = {}
        if filename is not None:
            self.load(filename)

    def set_param(self, name, value):
        ov = self.get_param(name)
        self._params[name] = value
        if value != ov:
            self.trigger(name, value)

    def set_param_from_dictionary(self, name, dictionary):
        for k in dictionary:
            n = '%s.%s' % (name, k)
            if isinstance(dictionary[k], dict):
                self.set_param_from_dictionary(n, dictionary[k])
            else:
                self.set_param(n, dictionary[k])

    def get_param(self, name, default=None):
        return self._params.get(name, default)

    def __getitem__(self, name):
        return self.get_param(name)

    def __setitem__(self, name, value):
        self.set_param(name, value)

    # TODO check type? range checking?

    def set_meta(self, name, meta=None, **kwargs):
        if meta is None:
            meta = Meta(**kwargs)
        self._meta[name] = meta

    def get_meta(self, name, default=None):
        return self._meta.get(name, default)

    def list_params(self, namespace=None):
        names = list(self._params.keys())
        if namespace is None:
            return names
        return [n for n in names if n.find(namespace) == 0]

    def load(self, filename):
        with open(filename, 'rb') as f:
            d = pickle.load(f)
            for m in d['meta']:
                self.set_meta(m, d['meta'][m])
            for p in d['params']:
                self.set_param(p, d['params'][p])

    def save(self, filename):
        with open(filename, 'rb') as f:
            d = {'params': self._params, 'meta': self._meta}
            pickle.dump(d, f)
