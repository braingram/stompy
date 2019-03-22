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
                self.on(message['id'], message['function'])
            elif message['method'] == 'remove_on':
                # unregister callback
                self.remove_on(message['id'], message['function'])
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
            self.trigger(msg['id'], *msg['result'])
        else:
            msg = None
        if wait_for_id is not None:
            if msg is None or not 'id' in msg:
                return self._recv_result()
            return msg['result']

    def update(self):
        self._recv_result()


class RPC(signaler.Signaler):
    def __init__(self, name, addr=None, port=5000, receive_timeout=0.01):
        super(RPC, self).__init__()
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

        # get spec
        #self._ws.send(
        #    json.dumps({'jsonrpc': '2.0', 'method': '__wsrpc__', 'id': 0}))
        #self.__wsrpc__ = json.loads(self._ws.recv())['result']
        self._message_id = 1

        # build fake object
        #while in_waiting(ws):
        #    msg = ws.recv()
        #    print("Received: %s" % (msg, ))

    def send_request(self, name, *args):
        msg = {'jsonrpc': '2.0', 'method': name, 'params': args}
        msg['id'] = self._message_id
        self._message_id += 1
        self._ws.send(json.dumps(msg))
        return msg['id']

    def _in_waiting(self):
        return len(select.select(
            [self._ws.sock, ], [], [], self._receive_timeout)[0])

    def _process_result(self, msg):
        # call any callbacks for this msg
        if 'error' in msg:
            raise Exception(msg['error']['message'])
        if 'id' not in msg:
            return
        self.trigger(msg['id'], msg['result'])

    def _recv_result(self, wait_for_id=None):
        if self._in_waiting():
            msg = json.loads(self._ws.recv())
            self._process_result(msg)
        else:
            msg = None
        if wait_for_id is not None:
            if msg is None or not 'id' in msg:
                return self._recv_result()
            return msg['result']

    def call_method(self, name, *args):
        if '.' in name:
            t = name.rpartition('.')[-1]
        else:
            t = name
        if t in ('connect', 'disconnect'):
            mid = self.send_request(name, *args[:-1])
            if t == 'connect':
                # this is a signal connection
                return self.on(mid, args[-1])
            elif t == 'disconnect':
                # this is a signal disconnection
                return self.remove_on(mid, args[-1])
        else:
            mid = self.send_request(name, *args)
        return self._recv_result(mid)

    def get_attrribute(self, name):
        return self.call_method(name)

    # TODO no current way to set an attribute?

    def __update_ws__(self):
        # process any new responses
        self._recv_result()


class Proxy(object):
    def __init__(self, name, addr=None, port=5000, receive_timeout=0.01):
        self._receive_timeout = receive_timeout

        self._cbs = {}

        if addr is None:
            addr = socket.gethostbyname(
                socket.gethostname() + '.local')
            addr = 'ws://%s:%s/%s' % (
                socket.gethostbyname(
                    socket.gethostname() + '.local'),
                port, name)
        self._ws = websocket.WebSocket()
        self._ws.connect(addr)

        # get spec
        self._ws.send(
            json.dumps({'jsonrpc': '2.0', 'method': '__wsrpc__', 'id': 0}))
        self.__wsrpc__ = json.loads(self._ws.recv())['result']
        self._message_id = 1

        # build fake object
        #while in_waiting(ws):
        #    msg = ws.recv()
        #    print("Received: %s" % (msg, ))
    def _send_request(self, method, params, use_id=True):
        msg = {'jsonrpc': '2.0', 'method': method, 'params': params}
        if use_id:
            msg['id'] = self._message_id
            self._message_id += 1
        self._ws.send(json.dumps(msg))
        if use_id:
            return msg['id']

    def _in_waiting(self):
        return len(select.select(
            [self._ws.sock, ], [], [], self._receive_timeout)[0])

    def _process_result(self, msg):
        # call any callbacks for this msg
        if 'error' in msg:
            raise Exception(msg['error']['message'])
        if 'id' not in msg:
            return
        if msg['id'] not in self._cbs:
            return
        for cb in self._cbs[msg['id']]:
            cb(msg['result'])

    def _recv_result(self, wait_for_id=None):
        if self._in_waiting():
            msg = json.loads(self._ws.recv())
            self._process_result(msg)
        else:
            msg = None
        if wait_for_id is not None:
            if msg is None or not 'id' in msg:
                return self._recv_result()
            return msg['result']

    def __getattr__(self, name):
        if len(name) and name[0] == '_':
            return super(Proxy, self).__getattribute__(name)
        # is this a function in the spec?
        if name in self.__wsrpc__:
            # if so, return a proxy function
            def w(n=name, o=self):
                def f(*args):
                    mid = o._send_request(n, args)
                    return self._recv_result(mid)
                return f
            # TODO watch for signals
            return w()
        # send request
        rid = self._send_request(name, [])
        # wait for response
        return self._recv_result(rid)

    def __setattr__(self, name, value):
        if len(name) and name[0] == '_':
            return super(Proxy, self).__setattr__(name, value)
        # send request
        # watch for signals
        pass

    def __update_ws__(self):
        # process any new responses
        self._recv_result()
