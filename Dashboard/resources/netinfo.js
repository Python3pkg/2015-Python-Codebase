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
