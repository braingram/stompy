Stompy = function(ws_url) {
    var self = this;
    var instance = {};
    instance._ws_url = ws_url;
    instance._mid = 0;
    instance._callbacks = {};
    instance._signal_callbacks = {};

    instance.onopen = function(event) {
        // web socket connected
    };
  
    instance.onmessage = function(event) {
        // web socket message received
        // parse message, call callbacks, etc
        message = JSON.parse(event.data);
        if (message['id'] in instance._callbacks) {
            instance._callbacks[message['id']](message['result']);
            delete instance._callbacks[message['id']];
        };
        if (message['id'] in instance._signal_callbacks) {
            instance._signal_callbacks[message['id']](...message['result']);
        };
    };

    instance._connect = function() {
        if (instance._ws_url == undefined) {
            url = 'ws://' + window.location.host + '/controller';
        } else {
            url = 'ws://' + instance._ws_url;
        };
        instance._socket = new WebSocket(url);
        instance._socket.onmessage = instance.onmessage;
        instance._socket.onopen = instance.onopen;
        instance._socket.onmessage = instance.onmessage;
    };

    instance.send_message = function(message, callback) {
        if (!('id' in message)) {
            message['id'] = instance._mid;
            instance._mid += 1;
        };
        // handle signal, if on/remove_on handle callbacks
        if (message['type'] == 'signal') {
            if (message['method'] == 'on') {
                // register callback
                instance._signal_callbacks[message['id']] =
                    message['function'];
            } else if (message['method'] == 'remove_on') {
                mid = undefined;
                for (var cbid in instance._signal_callbacks) {
                    if (
                            instance._signal_callbacks[cbid] ==
                            message['function']) {
                        mid = cbid;
                        break;
                    };
                };
                if (mid == undefined) return;
                message['id'] = mid;
                delete instance._signal_callbacks[mid];
            delete message['function'];
            };
        };
        s = JSON.stringify(message);
        instance._socket.send(s);
        if (callback != undefined) {
            instance._callbacks[message['id']] = callback;
        };
    };

    instance.call = function(name, args, kwargs, callback) {
        instance.send_message({
            type: 'call',
            name: name,
            args: args,
            kwargs: kwargs,
        }, callback);
    };

    instance.get = function(name, callback) {
        instance.send_message({
            type: 'get',
            name: name,
        }, callback);
    };

    instance.set = function(name, value) {
        instance.send_message({
            type: 'set',
            name: name,
            value: value,
        });
    };

    instance.on = function(obj, key, func) {
        instance.send_message({
            type: 'signal',
            name: obj,
            key: key,
            method: 'on',
            function: func,
        })
    };

    instance.remove_on = function(obj, key, func) {
        instance.send_message({
            type: 'signal',
            name: obj,
            key: key,
            method: 'remove_on',
            function: func,
        })
    };

    instance.trigger = function(obj, key, args, kwargs) { 
        msg = {
            type: 'call',
            kwargs: kwargs,
            args: [key, ].concat(args),
        }
        if (obj == '') {
            msg['function'] = 'trigger';
        } else {
            msg['function'] = obj + '.trigger';
        };
        instance.send_message(msg);
    };

    instance._connect();
    return instance;
}
