from aiohttp import web, WSMsgType
from asyncio import Queue
import os

async def start_server(self, cmd_queue: Queue):
    app = web.Application()
    app.add_routes([web.get('/ws', websocket_handler),
        web.static('/', os.path.dirname(__file__) + '/public/')])
    app.cmd_queue = cmd_queue

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
            await req.app.cmd_queue.put(args)

    return ws

