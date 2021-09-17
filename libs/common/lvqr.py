import lvgl as lv
import qrcode
import math
import gc

class QRCode(lv.img):
    def set_text(self, text="Text"):
        self._text = text

        raw = qrcode.encode(text)
        size = int(math.sqrt(len(raw)*8))
        buf = bytearray(raw)
    
        img = lv.img_dsc_t()
        
        img.header.cf = lv.img.CF.ALPHA_1BIT
        img.header.w = size
        img.header.h = img.header.w

        img.data = buf
        img.data_size = img.header.h * img.header.w

        self.set_src(img)

        # del raw
        gc.collect()

    def get_text(self):
        return self._text

    def set_size(self, size):
        self.set_width(size)
        self.set_height(size)

