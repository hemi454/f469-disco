import display
import lvgl as lv

display.init()

scr = lv.scr_act()
lbl = lv.label(scr)
lbl.set_text("Hello world")
display.update(30)
