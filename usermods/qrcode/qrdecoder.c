// Include required definitions first.
#include "py/obj.h"
#include "py/runtime.h"
#include "py/builtin.h"
#include "quirc.h"
#include <math.h>
#include <string.h>

extern uint8_t img_qr_sidorenko_2_map[];

static struct quirc *qr;
static uint8_t *image;
static struct quirc_code code;
static struct quirc_data data;

static struct quirc_data *qrdecoder_quirc_decoder(const uint8_t *img, const uint32_t img_len)
{
    struct quirc_data *result = NULL;

    qr = quirc_new();

    if (!qr)
    {
        mp_raise_ValueError(MP_ERROR_TEXT("QUIRC: Failed to allocate memory"));
    }
    else
    {
        int size = (int)sqrt(img_len);

        if (size > 0)
        {
            if (quirc_resize(qr, size, size) < 0)
            {
                mp_raise_ValueError(MP_ERROR_TEXT("QUIRC: Failed to allocate video memory"));
            }
            else
            {
                int w, h, num_codes;

                image = quirc_begin(qr, &w, &h);

                memcpy(image, img, img_len);

                quirc_end(qr);

                num_codes = quirc_count(qr);

                for (int i = 0; i < num_codes; i++)
                {
                    quirc_extract(qr, i, &code);

                    /* Decoding stage */
                    switch (quirc_decode(&code, &data))
                    {
                    case QUIRC_ERROR_INVALID_GRID_SIZE:
                    {
                        mp_raise_ValueError(MP_ERROR_TEXT("QUIRC: Invalid grid size"));
                        break;
                    }
                    case QUIRC_ERROR_INVALID_VERSION:
                    {
                        mp_raise_ValueError(MP_ERROR_TEXT("QUIRC: Invalid version"));
                        break;
                    }
                    case QUIRC_ERROR_FORMAT_ECC:
                    {
                        mp_raise_ValueError(MP_ERROR_TEXT("QUIRC: Format data ECC failure"));
                        break;
                    }
                    case QUIRC_ERROR_DATA_ECC:
                    {
                        mp_raise_ValueError(MP_ERROR_TEXT("QUIRC: ECC failure"));
                        break;
                    }
                    case QUIRC_ERROR_UNKNOWN_DATA_TYPE:
                    {
                        mp_raise_ValueError(MP_ERROR_TEXT("QUIRC: Unknown data type"));
                        break;
                    }
                    case QUIRC_ERROR_DATA_OVERFLOW:
                    {
                        mp_raise_ValueError(MP_ERROR_TEXT("QUIRC: Data overflow"));
                        break;
                    }
                    case QUIRC_ERROR_DATA_UNDERFLOW:
                    {
                        mp_raise_ValueError(MP_ERROR_TEXT("QUIRC: Data underflow"));
                        break;
                    }
                    default:
                    {
                        result = &data;
                        i = num_codes;
                        break;
                    }
                    }
                }
            }
        }
    }

    quirc_destroy(qr);

    return result;
}

STATIC mp_obj_t qrdecoder_decode(void)
{
    mp_obj_t result = mp_const_none;

    struct quirc_data *decode_data = qrdecoder_quirc_decoder(img_qr_sidorenko_2_map, pow(300, 2));

    if (decode_data != NULL)
    {
        result = mp_obj_new_bytearray(sizeof(uint8_t) * decode_data->payload_len, decode_data->payload);
    }

    return result;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(qrdecoder_decode_obj, qrdecoder_decode);

/****************************** MODULE ******************************/

STATIC const mp_rom_map_elem_t qrdecoder_module_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_qrdecoder)},
    {MP_ROM_QSTR(MP_QSTR_decode), MP_ROM_PTR(&qrdecoder_decode_obj)},
};
STATIC MP_DEFINE_CONST_DICT(qrdecoder_module_globals, qrdecoder_module_globals_table);

// Define module object.
const mp_obj_module_t qrdecoder_user_cmodule = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&qrdecoder_module_globals,
};

// Register the module to make it available in Python
MP_REGISTER_MODULE(MP_QSTR_qrdecoder, qrdecoder_user_cmodule, MODULE_QRDECODER_ENABLED);