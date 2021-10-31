import logging
import os
from aiohttp import web, WSMsgType

from car import CarContext
from utils import to_bool

async def start_server(car: CarContext):
    app = web.Application()
    app.add_routes([web.get('/ws', websocket_handler),
        web.static('/', os.path.dirname(__file__) + '/public/')])
    app.car = car

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()

async def websocket_handler(req):
    ws = web.WebSocketResponse()
    await ws.prepare(req)

    async for msg in ws:
        if msg.type == WSMsgType.TEXT:
            args = msg.data.split()
            result = process_cmd(req.app.car, args)
            if isinstance(result, tuple):
                write_response(ws, result)
            elif isinstance(result, list):
                for r in result:
                    write_response(ws, r)
            elif result is None:
                write_response(ws, (204, "Empty"))
            else:
                logging.error("Unknown command result type: %s", type(result))

    return ws

async def write_response(ws, result):
    await ws.send_str("{} {}", result[0], result[1])

def process_cmd(car: CarContext, args):
    if len(args) == 0:
        return
    cmd = args[0]

    try:
        if cmd == "move":
            return cmd_move(car, args)
        elif cmd == "get":
            return cmd_get(car, args)
        elif cmd == "set":
            return cmd_set(car, args)
        elif cmd == "stop":
            return cmd_stop(car, args)
        elif cmd == "shutdown":
            return cmd_shutdown(car, args)
        else:
            return (400, "Unknown command")
    except ValueError:
        return (401, "Invalid argument")
    except Exception as e:
        logging.error("Exception occurred while processing command", exc_info=e)
        return (500, "Server Error")

    return (200, "OK")

def cmd_move(car: CarContext, args):
    if len(args) != 3:
        return (402, "Invalid argument count")
    distance = float(args[1])
    angual = float(args[2])

    car.move_target(distance, angual)
    return (200, "OK")

def cmd_get(car: CarContext, args):
    return (502, "Unimplemented")

def cmd_set(car: CarContext, args):
    if len(args) != 3:
        return (402, "Invalid argument count")

    name = args[1]
    value = args[2]
    if name == "accel":
        val = float(value)
        pass
    elif name == "speed":
        val = float(value)
        pass
    elif name == "motor":
        val = to_bool(value)
        pass
    elif name == "avoid":
        val = to_bool(value)
        pass
    else:
        return (400, "Unknown field name")
    return (200, "OK")

def cmd_stop(car: CarContext, args):
    if len(args) != 1:
        return (402, "Invalid argument count")

    car.stop()
    return (200, "OK")

def cmd_shutdown(car: CarContext, args):
    if len(args) != 1:
        return (402, "Invalid argument count")

    car.shutdown = True
    return (200, "OK")
