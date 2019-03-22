#!/usr/bin/env python

import json
import select
import socket

import websocket

from . import protocol
from .. import signaler


class RPCClient(signaler.Signaler):
    def __init__(
            self, name='controller', addr=None, port=5000,
            receive_timeout=0.01):
        super(RPCClient, self).__init__()
        self._receive_timeout = receive_timeout

        if addr is None:
            addr = socket.gethostbyname(
                socket.gethostname() + '.local')
            addr = 'ws://%s:%s/%s' % (
                socket.gethostbyname(
                    socket.gethostname() + '.local'),
                port, name)
        self._ws = websocket.WebSocket()
        self._ws.connect(addr)
        self._message_id = 0

    def send(self, **message):
        if 'id' not in message:
            message['id'] = self._message_id
            self._message_id += 1
        # validate message
        protocol.validate_message(message)
        if message['type'] == 'signal':
            if message['method'] == 'on':
                # register callback
                super(RPCClient, self).on(message['id'], message['function'])
            elif message['method'] == 'remove_on':
                # unregister callback
                super(RPCClient, self).remove_on(
                    message['id'], message['function'])
            del message['function']
        self._ws.send(json.dumps(message))
        if (
                message['type'] in ('get', 'getitem', 'call') and
                not message.get('nonblock', False)):
            return self._recv_result(message['id'])

    def _in_waiting(self):
        return len(select.select(
            [self._ws.sock, ], [], [], self._receive_timeout)[0])

    def _recv_result(self, wait_for_id=None):
        if self._in_waiting():
            msg = json.loads(self._ws.recv())
            # call any callbacks for this msg
            if msg['id'] in self._callbacks:
                super(RPCClient, self).trigger(msg['id'], *msg['result'])
        else:
            msg = None
        if wait_for_id is not None:
            if msg is None or not 'id' in msg:
                return self._recv_result()
            return msg['result']

    def update(self):
        self._recv_result()

    def on(self, key, function, obj=None):
        if obj is None:
            obj = ''
        self.send(
            type='signal', key=key, name=obj,
            function=function, method='on')

    def remove_on(self, key, function, obj=None):
        if obj is None:
            obj = ''
        self.send(
            type='signal', key=key, name=obj,
            function=function, method='remove_on')

    def trigger(self, key, obj, *args, **kwargs):
        if obj is None:
            function = 'trigger'
        else:
            function = obj + '.trigger'
        self.send(
            type='call', name=function,
            args=[key, ] + list(args), kwargs=kwargs)

    def get(self, name):
        return self.send(type='get', name=name)

    def set(self, name, value):
        return self.send(type='set', name=name, value=value)

    def getitem(self, key, obj=None):
        if obj is None:
            obj = ''
        return self.send(type='getitem', name=obj, key=key)

    def setitem(self, key, value, obj=None):
        if obj is None:
            obj = ''
        return self.send(type='setitem', name=obj, key=key, value=value)

    def call(self, name, *args, **kwargs):
        kw = {'type': 'call', 'name': name}
        if len(args):
            kw['args'] = args
        if len(kwargs):
            kw['kwargs'] = kwargs
        return self.send(**kw)
