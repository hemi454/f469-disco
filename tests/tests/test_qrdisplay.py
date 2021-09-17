import display
import lvgl as lv
from lvqr import QRCode
import time

display.init()

scr = lv.scr_act()
qr = QRCode(scr)
time.sleep(0.1)
qr.set_text("Hello world, this is some text")
time.sleep(0.1)
qr.set_size(200)

lbl = lv.label(scr)
lbl.set_text("This text was written and posted on the screen by me")
lbl.set_x(20)
lbl.set_align(lv.label.ALIGN.CENTER)
lbl.set_y(250)

time.sleep(5)
