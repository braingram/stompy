#!/usr/bin/env python

import json

from .. import signaler


dkeys = {}.keys().__class__


class RemoteMessageEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, dkeys):
            return list(obj)
        return json.JSONEncoder.default(self, obj)


dumps = lambda i: json.dumps(i, cls=RemoteMessageEncoder)


def resolve_obj(obj, name):
    pi = name.find('.')
    bi = name.find('[')
    if pi != -1 and (pi < bi or bi == -1):  # period present and first
        # split by '.'
        # use get
        h, _, t = name.partition('.')
        return resolve_obj(getattr(obj, h), t)
    elif bi != -1:  # bracket present
        if bi == 0:
            # get substring in bracket
            # use getitem
            h, _, t = name[1:].partition(']')
            # convert h type
            if h[0] not in ('"', "'"):
                if '.' in h:
                    h = float(h)
                else:
                    h = int(h)
            else:
                # strip quotes
                h = h[1:-1]
            if len(t) == 0:
                return obj, h, False
            if t[0] == '.':
                t = t[1:]
            return resolve_obj(obj[h], t)
        else:
            # get substring before bracket
            # use get
            h, _, t = name.partition('[')
            return resolve_obj(getattr(obj, h), '[' + t)
    else:
        # neither, done
        # use get
        return obj, name, True


class RPCAgent(signaler.Signaler):
    def __init__(self, obj):
        super(RPCAgent, self).__init__()
        self.obj = obj

    def on(self, obj, key, function):
        if obj is None:
            obj = ''
        obj, sub_obj, is_attr = resolve_obj(self.obj, obj)
        if sub_obj != '':
            if is_attr:
                obj = getattr(obj, sub_obj)
            else:
                obj = obj[sub_obj]
        obj.on(key, function)

    def remove_on(self, obj, key, function):
        if obj is None:
            obj = ''
        obj, sub_obj, is_attr = resolve_obj(self.obj, obj)
        if sub_obj != '':
            if is_attr:
                obj = getattr(obj, sub_obj)
            else:
                obj = obj[sub_obj]
        obj.remove_on(key, function)

    def trigger(self, obj, key, *args, **kwargs):
        # trigger
        if obj is None or obj == '':
            function = 'trigger'
        else:
            function = obj + '.trigger'
        self.call(function, key, *args, **kwargs)

    def get(self, name):
        obj, key, is_attr = resolve_obj(self.obj, name)
        if is_attr:
            return getattr(obj, key)
        return obj[key]

    def set(self, name, value):
        obj, key, is_attr = resolve_obj(self.obj, name)
        if is_attr:
            setattr(obj, key, value)
        else:
            obj[key] = value

    def call(self, name, *args, **kwargs):
        obj, key, is_attr = resolve_obj(self.obj, name)
        if is_attr:
            f = getattr(obj, key)
        else:
            f = obj[key]
        return f(*args, **kwargs)

    def no_return_call(self, name, *args, **kwargs):
        self.call(name, *args, **kwargs)
