var networktables_websocket
var networktables_wd_connected = false
var networktables_nt_connected = false
var root_table = ""
var networktables_data = []

function start_networktables(){
    networktables_websocket = new WebSocket("ws:" + window.location.host + '/networktables')
    networktables_websocket.onmessage = networktables_message
    networktables_websocket.onclose = networktables_close
    networktables_websocket.onerror = networktables_error
    networktables_set_table()
}

function networktables_connect(e){
    setBadge("#networktables-wd-status", true, "Connected")
    networktables_data = []
    networktables_wd_connected = true
}

function networktables_error(e){
    console.log("networktables_error")
}

function networktables_close(e){
    networktables_wd_connected = false
    networktables_nt_connected = false
    setBadge("#networktables-wd-status", false, "Disconnected")
    setBadge("#networktables-nt-status", false, "Disconnected")
    $("#server_ip").text("Disconnected")
    setTimeout(start_networktables, 1000)
}

function networktables_message(e){
    if (!networktables_wd_connected)
        networktables_connect()
    obj = JSON.parse(e.data)
    pydict_update(networktables_data, obj)
    update_networktables_ui()
}

function networktables_set_table(){
    root_table = $("#networktables-select").val()
    update_networktables_ui()
}

function networktables_get_value(key, def){
    if (!def){
        def = false
    }
    components = key.split("/")
    target = networktables_data
    for (var i = 0; i < components.length; i++){
        subkey = components[i]
        if (subkey == ""){
            continue
        }
        if (target.hasOwnProperty(subkey)){
            target = target[subkey]
        }
        else{
            return def
        }
    }
    return target
}

function update_networktables_ui(){
    //Update NT connection badge
    if (networktables_get_value("/~CONNECTED~")){
        setBadge("#networktables-nt-status", true, "Connected")
        networktables_nt_connected = true
    }
    else{
        setBadge("#networktables-nt-status", false, "Disconnected")
        networktables_nt_connected = false
    }

    if (networktables_wd_connected){
        $("#server_ip").text(networktables_get_value("/~SERVER_IP~"))
    }

    //Clear UI
    $("#networktables-table tbody tr").remove()

    //Flatten data
    flattened_data = flatten_object(networktables_data)

    //Filter table
    filtered_data = []
    for (i in flattened_data){
        if (i.lastIndexOf(root_table, 0) == 0){
            newname = i.replace(root_table, "")
            filtered_data[newname] = flattened_data[i]
        }
    }

    //Sort table
    table_order = []
    for (i in filtered_data){
        table_order.push(i)
    }
    table_order.sort()

    //Generate HTML
    html = ""
    for (i=0; i < table_order.length; i++){
        name = table_order[i]
        html += '<tr><td>' + name +
                '</td><td>' + filtered_data[name] + '</td></tr>';
    }
    $("#networktables-table tbody").html(html)
}

$(document).ready(function(){
   start_networktables()
})