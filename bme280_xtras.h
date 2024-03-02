#ifndef BME280_XTRAS_H
#define BME280_XTRAS_H

#include <inttypes.h>
#include "esp_err.h"
#include "bme280.h"

/*!
 * @details User defineable struct to hold
 * additional info. about the BMP280
 */
struct identify
{
    uint8_t dev_addr; /* BME280's I2C address */
};

struct s_bme280
{
    struct identify id;
    struct bme280_dev dev;
    struct bme280_settings settings;
    uint32_t sampling_time;
};

/*
 * @details A wrapper around Bosch bme280_init() used to
 * return an esp_err_t
 */
esp_err_t
mbme280_init(struct bme280_dev *dev);

/*!
 * @brief A wrapper around Bosch bme280_set_regs() used to
 * return an esp_err_t
 */
esp_err_t
mbme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint32_t len, struct bme280_dev *dev);

/*!
 * @brief A wrapper around Bosch bme280_get_regs() used to
 * return an esp_err_t
 */
esp_err_t
mbme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bme280_dev *dev);

/*!
 * @brief A wrapper around Bosch bme280_set_sensor_settings() used to
 * return an esp_err_t
 */
esp_err_t mbme280_set_sensor_settings(uint8_t desired_settings,
                                      const struct bme280_settings *settings,
                                      struct bme280_dev *dev);
/*!
 * @brief A wrapper around Bosch bme280_get_sensor_settings() used to
 * return an esp_err_t
 */
esp_err_t mbme280_get_sensor_settings(struct bme280_settings *settings, struct bme280_dev *dev);

/*!
 * @brief A wrapper around Bosch bme280_soft_reset() used to
 * return an esp_err_t
 */
esp_err_t mbme280_set_sensor_mode(uint8_t mode, struct bme280_dev *dev);

/*!
 * @brief A wrapper around Bosch bme280_soft_reset() used to
 * return an esp_err_t
 */
esp_err_t mbme280_soft_reset(struct bme280_dev *dev);

/*!
 * @brief A wrapper around Bosch bme280_get_sensor_data() used to
 * return an esp_err_t
 */
esp_err_t mbme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev);

/*!
 * @brief A wrapper around Bosch bme280_compensate_data() used to
 * return an esp_err_t
 */
esp_err_t mbme280_compensate_data(uint8_t sensor_comp,
                                  const struct bme280_uncomp_data *uncomp_data,
                                  struct bme280_data *comp_data,
                                  struct bme280_calib_data *calib_data);

/*!
 * @brief A wrapper around Bosch bme280_cal_meas_delay() used to
 * return an esp_err_t
 */
esp_err_t mbme280_cal_meas_delay(uint32_t *max_delay,
                                 const struct bme280_settings *settings);


/*!
 * @brief A wrapper around Bosch bme280_cal_meas_delay() used to
 * return an esp_err_t
 */
esp_err_t mbme280_get_tph(struct s_bme280 *bme280, struct bme280_data *bme280_data);

#endif