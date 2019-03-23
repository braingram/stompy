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


class ObjectHandler(WebSocketHandler):
    def initialize(self, obj=None, **kwargs):
        self.obj = obj
        self.loop = tornado.ioloop.IOLoop.instance()
        self._cbs = {}

    def open(self):
        pass

    def on_close(self):
        # discconnect all callbacks for this websocket
        for mid in self._cbs:
            f, obj, n = self._cbs[mid]
            obj.remove_on(n, f)

    def on_message(self, message):
        # decode message, handle response
        msg = json.loads(message)
        protocol.validate_message(msg)
        if msg['type'] == 'get':
            obj, key, is_attr = resolve_obj(self.obj, msg['name'])
            if is_attr:
                result = getattr(obj, key)
            else:
                result = obj[key]
            return self.make_result(result, msg)
        elif msg['type'] == 'set':
            obj, key, is_attr = resolve_obj(self.obj, msg['name'])
            if is_attr:
                setattr(obj, key, msg['value'])
            else:
                obj[key] = msg['value']
        elif msg['type'] == 'call':
            obj, key, is_attr = resolve_obj(self.obj, msg['name'])
            if is_attr:
                f = getattr(obj, key)
            else:
                f = obj[key]
            return self.make_result(
                f(*msg.get('args', []), **msg.get('kwargs', {})), msg)
        elif msg['type'] == 'signal':
            if msg['name'] == '':
                obj = self.obj
            else:
                obj, key, is_attr = resolve_obj(self.obj, msg['name'])
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
