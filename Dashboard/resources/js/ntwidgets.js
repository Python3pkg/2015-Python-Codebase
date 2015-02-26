function ntwidgets_update(){
    $("input.ntwidgets-textbox").each(ntwidgets_update_textbox)
}

function ntwidgets_update_textbox(){
    jqobj = $(this)
    if (jqobj.is(":focus")) return
    key = jqobj.data("ntkey")
    current_value = networktables_get_value(key)
    jqobj.val(current_value.toString())
    // Update event listeners
    jqobj.off("focus")
    jqobj.off("focusout")
    jqobj.focus(ntwidgets_focus_textbox)
}

function ntwidgets_focus_textbox(){
    $(this).off("focus")
    $(this).focusout(ntwidgets_save_textbox)
    $(this).keyup(function (e){if(e.keyCode == 13){$(this).blur()}});
}

function ntwidgets_save_textbox(){
    $(this).off("focusout")
    key = $(this).data("ntkey")
    value = $(this).val()
    networktables_set_value(key, value)
}

$(document).ready(function(){
   $(document).on("ntupdate", ntwidgets_update)
})