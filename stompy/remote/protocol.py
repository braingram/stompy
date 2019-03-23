#!/usr/bin/env python
"""
{'type': 'call/get/set/getitem/setitem/inspect/signal',
'id': <unique message id>}
call: {'name': 'name [can use dot notation]',
'args': [], 'kwargs': {}, 'nonblock': True}
get: {'name': '...'}
set: {'name': '...', 'value': ...}
getitem {'name': '...', 'key': 'key [can use dot notation]'}
setitem {'name': '...', 'key': '...', 'value': ...}
inspect {'method': 'builtin[type,etc] or inspect.something',
'args': [], 'kwargs': {}}
signal: {'name': 'optional object name, if none use base object',
'op': 'on/remove_on default on', 'key': 'signal key'}

send and receive

# all should have ids?
call: has name, possible args, kwargs, nonblock
get: has name
set: has name, has value
getitem: has name, has key
setitem: has name, has key, has value
signal: has name, has method, has key [trigger handled by call]

inspect: has name, has method, possible args, possible kwargs

result message is:
    'result': (always a list for signals)
    'id': (message id)
    return message contents?

combine get/getitem set/setitem by looking for '['
be careful about '.' inside []
"""

#TYPES = ['call', 'get', 'set', 'getitem', 'setitem', 'signal']
TYPES = ['call', 'get', 'set', 'signal']


class RPCError(Exception):
    pass


def _has_key(message, key):
    if key not in message:
        raise RPCError("Missing %s: %s" % (key, message))


def validate_message(message):
    _has_key(message, 'id')
    _has_key(message, 'type')
    if message['type'] not in TYPES:
        raise RPCError("Unkonwn type: %s not in %s" % (message['type'], TYPES))
    _has_key(message, 'name')
    if message['type'] in ('set', 'setitem'):
        _has_key(message, 'value')
    #if message['type'] in ('getitem', 'setitem', 'signal'):
    if message['type'] in 'signal':
        _has_key(message, 'key')
    if message['type'] == 'signal':
        _has_key(message, 'method')
        if message['method'] not in ('on', 'remove_on'):
            raise RPCError("Unknown signal method %s" % (message['method'], ))
