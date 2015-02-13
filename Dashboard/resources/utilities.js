function setBadge(identifier, status, text){
    badge = $(identifier)
    badge.text(text)
    badge.removeClass("label-danger")
    badge.removeClass("label-success")
    if(status){
        badge.addClass("label-success")
    }
    else{
        badge.addClass("label-danger")
    }
}


function pydict_update(tgt, src){
    for (var key in src){
        if (typeof obj[key] == "object"){
            if (!tgt.hasOwnProperty(key)){
                tgt[key] = []
            }
            pydict_update(tgt[key], src[key])
        }
        else{
            tgt[key] = src[key]
        }
    }
}


function flatten_object(obj){
    var result = []
    for (var key in obj){
        if (typeof obj[key] == "object"){
            var subflat = flatten_object(obj[key])
            for (s in subflat){
                result["/" + key + s] = subflat[s]
            }
        }
        else{
            result["/" + key] = obj[key]
        }
    }
    return result
}