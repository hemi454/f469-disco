/**
 * @file px_img.h
 *
 */

#ifndef PX_IMG_H
#define PX_IMG_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef LV_CONF_INCLUDE_SIMPLE
#include "lv_conf.h"
#else
#include "../../lib/lv_bindings/lv_conf.h"
#endif

#if LV_USE_IMG != 0

#include "../../lib/lv_bindings/lvgl/src/lv_core/lv_obj.h"
#include "../../lib/lv_bindings/lvgl/src/lv_misc/lv_fs.h"
#include "../../lib/lv_bindings/lvgl/src/lv_widgets/lv_label.h"
#include "../../lib/lv_bindings/lvgl/src/lv_draw/lv_draw.h"

lv_obj_t * px_img_create(lv_obj_t * par, const lv_obj_t * copy);

#endif /*LV_USE_IMG*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*PX_IMG_H*/
