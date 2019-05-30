#!/usr/bin/env python

import json
import select
import socket
import time

import websocket

from . import agent
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
                # lookup correct id for this function
                mid = None
                for mid in self._callbacks:
                    if (
                            len(self._callbacks[mid]) and
                            self._callbacks[mid][0] == message['function']):
                        break
                if mid is None:
                    # non-existant callback, do nothing
                    return
                message['id'] = mid
                # unregister callback
                super(RPCClient, self).remove_on(
                    message['id'], message['function'])
            del message['function']
        #print("send:", message)
        self._ws.send(agent.dumps(message))
        if (
                message['type'] in ('get', 'getitem', 'call') and
                not message.get('nonblock', False)):
            return self._recv_result(message['id'])

    def _in_waiting(self):
        return len(select.select(
            [self._ws.sock, ], [], [], self._receive_timeout)[0])

    def _read_next_message(self):
        if self._in_waiting():
            msg = json.loads(self._ws.recv())
            #print("receive_result:", msg)
            # call any callbacks for this msg
            if msg['id'] in self._callbacks:
                super(RPCClient, self).trigger(msg['id'], *msg['result'])
        else:
            msg = None
        return msg

    def _recv_result(self, wait_for_id):
        msg = None
        t0 = time.time()
        #print("Start waiting[", t0, "]:", wait_for_id)
        while msg is None or msg['id'] != wait_for_id:
            msg = self._read_next_message()
        #print("Done waiting[", time.time() - t0, "]:", msg)
        return msg['result']

    def update(self, max_time=0.1):
        t0 = time.time()
        while time.time() - t0 < max_time and self._in_waiting():
            self._read_next_message()

    def on(self, obj, key, function):
        if obj is None:
            obj = ''
        self.send(
            type='signal', key=key, name=obj,
            function=function, method='on')

    def remove_on(self, obj, key, function):
        if obj is None:
            obj = ''
        # lookup callback id for this guy
        self.send(
            type='signal', key=key, name=obj,
            function=function, method='remove_on')

    def trigger(self, obj, key, *args, **kwargs):
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

    def call(self, name, *args, **kwargs):
        kw = {'type': 'call', 'name': name}
        if len(args):
            kw['args'] = args
        if len(kwargs):
            kw['kwargs'] = kwargs
        return self.send(**kw)

    def no_return_call(self, name, *args, **kwargs):
        kw = {'type': 'call', 'name': name, 'nonblock': True}
        if len(args):
            kw['args'] = args
        if len(kwargs):
            kw['kwargs'] = kwargs
        return self.send(**kw)
