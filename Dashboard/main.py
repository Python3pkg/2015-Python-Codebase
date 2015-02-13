#/usr/bin/python3
import asyncio
import sys
from os import symlink, remove
from os.path import abspath, dirname, join, exists

from aiohttp import web

import networktables_controller

@asyncio.coroutine
def forward_request(request):
    return web.HTTPFound("/index.html")

INIT_FILE = "Webdash_init.sh"
INSTALL_LOCATIONS = "/etc/init.d/" + INIT_FILE, "/etc/rc5.d/S99" + INIT_FILE

def run_server(port):
    print("Starting Webdash Server.")
    file_root = join(abspath(dirname(__file__)), "resources")
    networktables_controller.setup_networktables()
    app = web.Application()
    app.router.add_route("GET", "/networktables", networktables_controller.networktables_websocket)
    app.router.add_route("GET", "/", forward_request)
    app.router.add_static("/", file_root)
    loop = asyncio.get_event_loop()
    f = loop.create_server(app.make_handler(), '0.0.0.0', port)
    srv = loop.run_until_complete(f)
    print("RoboRIO Webdash listening on", srv.sockets[0].getsockname())
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    run_server(8300)