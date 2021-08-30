from udisplay import update, on, off, set_rotation

def init(autoupdate=True):
    import lvgl as lv
    import SDL

    HOR_RES = 480
    VER_RES = 800

    """
    GUI initialization function. 
    Should be called once in the very beginning.
    """

    # init the gui library
    lv.init()
    # init the hardware library
    SDL.init()

   # Register SDL display driver.

    disp_buf1 = lv.disp_buf_t()
    buf1_1 = bytes(HOR_RES*10)
    disp_buf1.init(buf1_1, None, len(buf1_1)//4)
    disp_drv = lv.disp_drv_t()
    disp_drv.init()
    disp_drv.buffer = disp_buf1
    disp_drv.flush_cb = SDL.monitor_flush
    disp_drv.hor_res = HOR_RES
    disp_drv.ver_res = VER_RES
    disp_drv.register()

    # Regsiter SDL mouse driver

    indev_drv = lv.indev_drv_t()
    indev_drv.init() 
    indev_drv.type = lv.INDEV_TYPE.POINTER
    indev_drv.read_cb = SDL.mouse_read
    indev_drv.register()

    scr = lv.obj()
    lv.scr_load(scr)
    if autoupdate:
        import SDL
        SDL.enable_autoupdate()
