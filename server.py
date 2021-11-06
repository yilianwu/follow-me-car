import logging
import os
import weakref
from aiohttp import web, WSMsgType, WSCloseCode

from car import CarContext
from constant import *
from utils import to_bool

async def start_server(car: CarContext):
    app = web.Application()
    app.add_routes([web.get('/ws', websocket_handler),
        web.get('/', index_handler),
        web.static('/assets', os.path.dirname(__file__) + '/public/assets')])
    app['car'] = car
    app['websockets'] = weakref.WeakSet()
    app.on_shutdown.append(on_shutdown)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()
    return (site, runner)

async def index_handler(req):
    return web.FileResponse(os.path.dirname(__file__) + '/public/index.html')

async def websocket_handler(req):
    ws = web.WebSocketResponse()
    await ws.prepare(req)
    logging.info(f"New client connected: {req.remote}")
    req.app['websockets'].add(ws)

    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                args = msg.data.split()
                logging.debug(args)
                result = process_cmd(req.app['car'], args)
                if isinstance(result, tuple):
                    await write_response(ws, result)
                elif isinstance(result, list):
                    for r in result:
                        await write_response(ws, r)
                elif result is None:
                    await write_response(ws, (204, "Empty"))
                else:
                    logging.error("Unknown command result type: %s", type(result))
    finally:
        req.app['websockets'].discard(ws)

    return ws

async def on_shutdown(app):
    logging.info("Closing all websockets...")
    for ws in set(app['websockets']):
        await ws.close(code=WSCloseCode.GOING_AWAY, message='Server shutdown')

async def write_response(ws, result):
    await ws.send_str(f"{result[0]} {result[1]}")

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
    if len(args) != 2:
        return (402, "Invalid argument count")

    name = args[1]
    result = []
    if name == "accel":
        result.append((100, f"{name} {car.max_acceler}"))
    elif name == "speed":
        val = car.max_speed * 60 / PPR;
        result.append((100, f"{name} {val}"))
    elif name == "motor":
        result.append((100, f"{name} {car.motor_state}"))
    elif name == "avoid":
        result.append((100, f"{name} {car.avoid_state}"))
    elif name == "status":
        result.append((100, f"{name} {car.status}"))
    else:
        return (400, "Unknown field name")

    result.append((200, "OK"))
    return result

def cmd_set(car: CarContext, args):
    if len(args) != 3:
        return (402, "Invalid argument count")

    name = args[1]
    value = args[2]
    if name == "accel":
        val = float(value)
        car.max_acceler = val
    elif name == "speed":
        val = float(value)
        car.max_speed = val * PPR / 60
    elif name == "motor":
        val = to_bool(value)
        car.motor_state = val
    elif name == "avoid":
        val = to_bool(value)
        car.avoid_state = val
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
