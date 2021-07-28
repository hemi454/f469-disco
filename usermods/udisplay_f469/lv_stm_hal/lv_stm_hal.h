#ifndef __LV_STM_HAL_H__
#define __LV_STM_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Maximal horizontal and vertical resolution to support by the library.*/
#define LV_HOR_RES_MAX          (480)
#define LV_VER_RES_MAX          (800)

void tft_init();
void touchpad_init();

#ifdef __cplusplus
}
#endif

#endif //__LV_STM_HAL_H__
