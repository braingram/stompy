#!/usr/bin/env python

import json
import os
import socket

import tornado.ioloop
import tornado.web
from tornado.websocket import WebSocketHandler

from . import agent
from .. import controller
from . import protocol


template_path = os.path.join(os.path.dirname(__file__), 'templates')


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        return self.render(os.path.join(template_path, "index.html"))


class ObjectHandler(WebSocketHandler):
    def initialize(self, obj=None, **kwargs):
        self.obj = obj
        self.agent = agent.RPCAgent(self.obj)
        self.loop = tornado.ioloop.IOLoop.instance()
        self._cbs = {}

    def open(self):
        pass

    def on_close(self):
        # discconnect all callbacks for this websocket
        for mid in self._cbs:
            #f, obj, n = self._cbs[mid]
            #obj.remove_on(n, f)
            f, r = self._cbs[mid]
            r(f)
        self._cbs = {}

    def on_message(self, message):
        # decode message, handle response
        msg = json.loads(message)
        #print("received:", msg)
        protocol.validate_message(msg)
        if msg['type'] == 'get':
            #print("get:", msg)
            return self.make_result(self.agent.get(msg['name']), msg)
        elif msg['type'] == 'set':
            self.agent.set(msg['name'], msg['value'])
        elif msg['type'] == 'call':
            r = self.make_result(
                self.agent.call(
                    msg['name'],
                    *msg.get('args', []),
                    **msg.get('kwargs', {})),
                msg)
            return r
        elif msg['type'] == 'signal':
            if msg['method'] == 'on':
                def w(m=msg, s=self, a=self.agent):
                    def cb(*args):
                        return s.make_result(args, m)

                    def rcb(f):
                        a.remove_on(m['name'], m['key'], f)
                    return cb, rcb
                f, r = w()
                self._cbs[msg['id']] = (f, r)
                self.agent.on(msg['name'], msg['key'], f)
            elif msg['method'] == 'remove_on':
                # removal is only by message id
                if msg['id'] not in self._cbs:
                    return
                (f, r) = self._cbs[msg['id']]
                r(f)
                del self._cbs[msg['id']]
                #self.agent.remove_on(msg['name'], msg['key'], f)

            """
            if msg['name'] == '':
                obj = self.obj
            else:
                obj, key, is_attr = agent.resolve_obj(self.obj, msg['name'])
                if is_attr:
                    obj = getattr(obj, key)
                else:
                    obj = obj[key]
            # 'method': 'on/remove_on'
            if msg['method'] == 'on':
                # attach callback using id
                def w(m=msg, s=self):
                    def cb(*args):
                        return s.make_result(args, m)
                    return cb
                f = w()
                self._cbs[msg['id']] = (f, obj, msg['key'])
                obj.on(msg['key'], f)
            elif msg['method'] == 'remove_on':
                # detach callback using remove_on
                # ignore missing callback removals?
                (f, _, _) = self._cbs[msg['id']]
                obj.remove_on(msg['key'], f)
            """

    def make_result(self, result, message):
        rmsg = {
            'result': result,
            'id': message['id']
        }
        #print("result:", rmsg)
        self.send(agent.dumps(rmsg))

    def send(self, message):
        # do actual writing in the ioloop
        self.loop.add_callback(self.write_message, message)


def serve(addr=None, port=5000):
    c = controller.build()

    # setup periodic update
    cb = tornado.ioloop.PeriodicCallback(c.update, 10.0)
    cb.start()
    static_path = os.path.join(os.path.dirname(__file__), 'static')
    app = tornado.web.Application([
        (r"/", MainHandler),
        (r"/controller", ObjectHandler, {'obj': c}),
    ], static_path=static_path)

    if addr is None:
        addr = socket.gethostbyname(
            socket.gethostname() + '.local')
    print("Serving controller on %s:%s" % (addr, port))
    app.listen(port, address=addr)
    tornado.ioloop.IOLoop.current().start()
