#!/usr/bin/env python

import json
import socket

import tornado.ioloop
import tornado.web
from tornado.websocket import WebSocketHandler

from .. import controllers
from . import protocol


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        # self.render("index.html")
        return "hi"


class ObjectHandler(WebSocketHandler):
    def initialize(self, obj=None, **kwargs):
        self.obj = obj
        self.loop = tornado.ioloop.IOLoop.instance()
        self._cbs = {}

    def open(self):
        pass

    def on_close(self):
        pass

    def on_message(self, message):
        # decode message, handle response
        msg = json.loads(message)
        protocol.validate_message(msg)
        if msg['type'] == 'get':
            result = reduce(getattr, msg['name'].split('.'), self.obj)
            return self.make_result(result, msg)
        elif msg['type'] == 'getitem':
            attr = reduce(getattr, msg['name'].split('.'), self.obj)
            return self.make_result(attr[msg['key']], msg)
        elif msg['type'] == 'call':
            f = reduce(getattr, msg['name'].split('.'), self.obj)
            return self.make_result(
                f(*msg.get('args', []), **msg.get('kwargs', {})), msg)
        elif msg['type'] == 'set':
            if '.' in msg['name']:
                obj_name, _, attr_name = msg['name'].rpartition('.')
                obj = reduce(getattr, obj_name.split('.'), self.obj)
            else:
                obj = self.obj
                attr_name = msg['name']
            setattr(obj, attr_name, msg['value'])
            return
        elif msg['type'] == 'setitem':
            obj = reduce(getattr, msg['name'].split('.'), self.obj)
            obj[msg['key']] = msg['value']
            return
        elif msg['type'] == 'signal':
            if msg['name'] == '':
                obj = self.obj
            else:
                obj = reduce(getattr, msg['name'].split('.'), self.obj)
            # 'method': 'on/remove_on'
            if msg['method'] == 'on':
                # attach callback using id
                def w(m=msg, s=self):
                    def cb(*args):
                        return s.make_result(args, m)
                    return cb
                f = w()
                self._cbs[msg['id']] = f
                obj.on(msg['key'], f)
            elif msg['method'] == 'remove_on':
                # detach callback using remove_on
                # ignore missing callback removals?
                #if msg['id'] not in self._cbs:
                #    return
                f = self._cbs[msg['id']]
                obj.remove_on(msg['key'], f)

    def make_result(self, result, message):
        rmsg = {
            'result': result,
            'id': message['id']
        }
        self.send(json.dumps(rmsg))

    def send(self, message):
        # do actual writing in the ioloop
        self.loop.add_callback(self.write_message, message)


def serve(addr=None, port=5000):
    c = controllers.multileg.build()

    # setup periodic update
    cb = tornado.ioloop.PeriodicCallback(c.update, 10.0)
    cb.start()
    app = tornado.web.Application([
        (r"/", MainHandler),
        (r"/controller", ObjectHandler, {'obj': c}),
    ])

    if addr is None:
        addr = socket.gethostbyname(
            socket.gethostname() + '.local')
    print("Serving controller on %s:%s" % (addr, port))
    app.listen(port, address=addr)
    tornado.ioloop.IOLoop.current().start()
