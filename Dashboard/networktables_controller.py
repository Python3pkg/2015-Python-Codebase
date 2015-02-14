from networktables import NetworkTable
import asyncio
import json
from aiohttp import web, errors as weberrors
from threading import RLock
from copy import deepcopy

server_address = "0.0.0.0"

initialized_networktables = False

table_data = dict()
table_data_lock = RLock()
root_table = None

connections = list()
tagged_tables = list()

class ConnectionListener:
    def connected(self, table):
        set_local_value("/", "~CONNECTED~", True)

    def disconnected(self, table):
        set_local_value("/", "~CONNECTED~", False)

def subtable_listener(source, key, value, isNew):
    with table_data_lock:
        if source.containsSubTable(key):
            watch_table(value.path)
        else:
            val_listener(source, key, value, isNew)

def val_listener(source, key, value, isNew):
    set_local_value(source.path, key, source.getValue(key))

def set_local_value(path, key, value):

    # Get the nested dictionary at path
    target_ref = table_data
    for s in path.split(NetworkTable.PATH_SEPARATOR):
        if s == "":
            continue
        elif s not in target_ref:
            target_ref[s] = dict()
        target_ref = target_ref[s]

    # Save the value if it is new
    if key not in target_ref or target_ref[key] != value:
        target_ref[key] = value
        for con in connections:
            con["updated_data"] = True

def watch_table(key):
    print("Watching Table " + key)
    with table_data_lock:
        if key in tagged_tables:
            return
        new_table = root_table.getTable(key)
        new_table.addSubTableListener(subtable_listener)
        new_table.addTableListener(val_listener, True)

def setup_networktables(server=server_address):
    global root_table, table_data, initialized_networktables
    if initialized_networktables:
        return
    NetworkTable.setIPAddress(server)
    NetworkTable.setClientMode()
    NetworkTable.initialize()
    root_table = NetworkTable.getTable("")
    c_listener = ConnectionListener()
    root_table.addConnectionListener(c_listener)
    root_table.addSubTableListener(subtable_listener)
    initialized_networktables = True

@asyncio.coroutine
def networktables_websocket(request):
    #Setup websocket
    ws = web.WebSocketResponse()
    ws.start(request)

    #Setup connection dict
    con_id = len(connections)
    with table_data_lock:
        connection = {"socket": ws, "updated_data": True}
        connections.append(connection)
    print("NT Websocket {} Connected".format(con_id))

    #Start listener coroutine
    asyncio.async(networktables_websocket_listener(ws))

    #Set IP status data
    ip = request.transport.get_extra_info("sockname")[0]
    set_local_value("", "~SERVER_IP~", ip)

    last_data = dict()

    #Update periodically until the websocket is closed.
    try:

        while True:
            if connection["updated_data"]:
                connection["updated_data"] = False
                updates = dict_delta(last_data, table_data)
                string_data = json.dumps(updates)
                print("Sending " + string_data)
                ws.send_str(string_data)
                last_data = deepcopy(table_data)
            if ws.closing:
                break
            yield from asyncio.sleep(1)
    except weberrors.ClientDisconnectedError or weberrors.WSClientDisconnectedError:
        print("Client Disconnected")
    finally:
        print("NT Websocket {} Disconnected".format(con_id))
        with table_data_lock:
            connections.remove(connection)
        return ws

def dict_delta(dict_a, dict_b):
    """
    recursively compares two dictionaries, returns the dictionary of differences.
    aka retval = dict_b - dict_a
    """
    result = dict()
    for k in dict_b:
        if k in dict_a:
            if isinstance(dict_a[k], dict) and isinstance(dict_b[k], dict):
                comp_res = dict_delta(dict_a[k], dict_b[k])
                if len(comp_res) > 0:
                    result[k] = comp_res
            elif dict_a[k] != dict_b[k]:
                result[k] = dict_b[k]
        else:
            result[k] = dict_b[k]
    return result


@asyncio.coroutine
def networktables_websocket_listener(ws):
    while True:
        try:
            jdata = yield from ws.receive_str()
        except Exception:
            return
        data = json.loads(jdata)
        root_table.pushData(data["name"], data["value"])

def networktables_websocket_writer(ws):
    while True:
        try:
            jdata = yield from ws.write_str()
        except Exception:
            return
        data = json.loads(jdata)
        root_table.pushData(data["name"], data["value"])