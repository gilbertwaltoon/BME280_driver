#include <stdio.h>
#include <inttypes.h>
#include "bme280.h"
#include "bme280_xtras.h"
// #include <string.h>
// #include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

static const char *TAG = "bme280_xtras.c";

esp_err_t bme280_err(int8_t rslt)
{
    switch (rslt)
    {
    case BME280_E_NULL_PTR:
        ESP_RETURN_ON_ERROR(ESP_FAIL, TAG, "BME2_E_NULL_PTR");
        break;
    case BME280_E_COMM_FAIL:
        ESP_RETURN_ON_ERROR(ESP_FAIL, TAG, "BME2_E_COM_FAIL");
        break;
    case BME280_E_INVALID_LEN:
        ESP_RETURN_ON_ERROR(ESP_FAIL, TAG, "BME2_E_INVALID_LEN");
        break;
    case BME280_E_DEV_NOT_FOUND:
        ESP_RETURN_ON_ERROR(ESP_FAIL, TAG, "BME2_E_DEV_NOT_FOUND");
        break;
    case BME280_E_SLEEP_MODE_FAIL:
        ESP_RETURN_ON_ERROR(ESP_FAIL, TAG, "BME2_E_SLEEP_MODE_FAIL");
        break;
    case BME280_E_NVM_COPY_FAILED:
        ESP_RETURN_ON_ERROR(ESP_FAIL, TAG, "BME2_E_NVM_COPY_FAILED");
        break;
    default:
        break;
    }

    return ESP_OK;
}

esp_err_t
mbme280_init(struct bme280_dev *dev)
{
    return bme280_err(bme280_init(dev));
}

esp_err_t
mbme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint32_t len, struct bme280_dev *dev)
{
    return bme280_err(bme280_set_regs(reg_addr, reg_data, len, dev));
}

esp_err_t
mbme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                 struct bme280_dev *dev)
{
    return bme280_err(bme280_get_regs(reg_addr, reg_data, len, dev));
}

esp_err_t
mbme280_set_sensor_settings(uint8_t desired_settings,
                            const struct bme280_settings *settings,
                            struct bme280_dev *dev)
{
    return bme280_err(bme280_set_sensor_settings(desired_settings, settings, dev));
}

esp_err_t
mbme280_get_sensor_settings(struct bme280_settings *settings, struct bme280_dev *dev)
{
    return bme280_err(bme280_get_sensor_settings(settings, dev));
}

esp_err_t
mbme280_set_sensor_mode(uint8_t mode, struct bme280_dev *dev)
{
    return bme280_err(bme280_set_sensor_mode(mode, dev));
}

esp_err_t mbme280_soft_reset(struct bme280_dev *dev)
{
    return bme280_err(bme280_soft_reset(dev));
}

esp_err_t
mbme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data,
                        struct bme280_dev *dev)
{
    return bme280_err(bme280_get_sensor_data(sensor_comp, comp_data, dev));
}

esp_err_t mbme280_compensate_data(uint8_t sensor_comp,
                                  const struct bme280_uncomp_data *uncomp_data,
                                  struct bme280_data *comp_data,
                                  struct bme280_calib_data *calib_data)
{
    return bme280_err(bme280_compensate_data(sensor_comp, uncomp_data,
                                             comp_data, calib_data));
}

esp_err_t
mbme280_cal_meas_delay(uint32_t *max_delay,
                       const struct bme280_settings *settings)
{
    return bme280_err(bme280_cal_meas_delay(max_delay, settings));
}


static esp_err_t _bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev)
{
    return bme280_err(bme280_get_sensor_data(sensor_comp, comp_data, dev));
}

esp_err_t mbme280_get_tph(struct s_bme280 *bme280, struct bme280_data *bme280_data)
{

    esp_err_t e;
    uint8_t status_reg;
 
    // ESP_LOGI(TAG, "Measurement delay : %lu us\n", (long unsigned int)period);
    do
    {
        mbme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &bme280->dev);
        // bmp2_log_error_code(r);
    } while (!(status_reg & BME280_STATUS_MEAS_DONE));

    /* Delay between measurements */
    (&bme280->dev)->delay_us(bme280->sampling_time, (&bme280->dev)->intf_ptr);
    e = _bme280_get_sensor_data(BME280_ALL, bme280_data, &bme280->dev);

#ifdef BME280_64BIT_COMPENSATION
    bme280_data->pressure = (bme280_data->pressure) / 256;
#endif

    return e;
}
