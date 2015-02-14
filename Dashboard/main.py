import asyncio
from os.path import abspath, dirname, join
from aiohttp import web

import networktables_controller

@asyncio.coroutine
def forward_request(request):
    return web.HTTPFound("/index.html")

def run_server(port):
    print("Starting Dashboard Server.")
    file_root = join(abspath(dirname(__file__)), "resources")
    networktables_controller.setup_networktables()
    app = web.Application()
    app.router.add_route("GET", "/networktables", networktables_controller.networktables_websocket)
    app.router.add_route("GET", "/netinfo", networktables_controller.networktables_websocket)
    app.router.add_route("GET", "/", forward_request)
    app.router.add_static("/", file_root)
    loop = asyncio.get_event_loop()
    f = loop.create_server(app.make_handler(), '0.0.0.0', port)
    srv = loop.run_until_complete(f)
    print("4819 Dashboard listening on", srv.sockets[0].getsockname())
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    run_server(5802)

