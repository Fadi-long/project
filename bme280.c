#include "bme280.h"

// low-level SPI R/W
static HAL_StatusTypeDef BME280_ReadRegs(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= 0x80;
    BME280_CS_LOW();
    HAL_SPI_Transmit(hspi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, buf, len, HAL_MAX_DELAY);
    BME280_CS_HIGH();
    return HAL_OK;
}
static HAL_StatusTypeDef BME280_WriteReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { reg & 0x7F, val };
    BME280_CS_LOW();
    HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
    BME280_CS_HIGH();
    return HAL_OK;
}

// publieke API
uint8_t BME280_ReadID(SPI_HandleTypeDef *hspi) {
    uint8_t id;
    BME280_ReadRegs(hspi, BME280_REG_ID, &id, 1);
    return id;
}

HAL_StatusTypeDef BME280_Init(SPI_HandleTypeDef *hspi, BME280_Calib *c) {
    uint8_t buf[26];
    // reset
    BME280_WriteReg(hspi, BME280_REG_RESET, 0xB6);
    HAL_Delay(100);
    // lees T/P-calibratie 0x88..0xA1 (26 bytes)
    BME280_ReadRegs(hspi, BME280_REG_CALIB00, buf, 26);
    c->dig_T1 = (buf[1]<<8)|buf[0];
    c->dig_T2 = (buf[3]<<8)|buf[2];
    c->dig_T3 = (buf[5]<<8)|buf[4];
    c->dig_P1 = (buf[7]<<8)|buf[6];
    c->dig_P2 = (buf[9]<<8)|buf[8];
    c->dig_P3 = (buf[11]<<8)|buf[10];
    c->dig_P4 = (buf[13]<<8)|buf[12];
    c->dig_P5 = (buf[15]<<8)|buf[14];
    c->dig_P6 = (buf[17]<<8)|buf[16];
    c->dig_P7 = (buf[19]<<8)|buf[18];
    c->dig_P8 = (buf[21]<<8)|buf[20];
    c->dig_P9 = (buf[23]<<8)|buf[22];
    c->dig_H1 =  buf[25];
    // lees H-calibratie 0xE1..0xE7 (7 bytes)
    BME280_ReadRegs(hspi, BME280_REG_CALIB26, buf, 7);
    c->dig_H2 = (buf[1]<<8)|buf[0];
    c->dig_H3 = buf[2];
    c->dig_H4 = (buf[3]<<4)|(buf[4]&0x0F);
    c->dig_H5 = (buf[5]<<4)|(buf[4]>>4);
    c->dig_H6 = buf[6];



    return HAL_OK;
}

HAL_StatusTypeDef BME280_ReadRawHumidity(SPI_HandleTypeDef *hspi, uint8_t *rawH) {
    // raw H op 0xFD,0xFE = PRESS_MSB+6, +7
    return BME280_ReadRegs(hspi, BME280_REG_PRESS_MSB+6, rawH, 2);
}

HAL_StatusTypeDef BME280_ForceMeasure(SPI_HandleTypeDef *hspi) {
    // oversample hum×1
    BME280_WriteReg(hspi, BME280_REG_CTRL_HUM,  0x01);
    // forced-mode, osrs_t=1, osrs_p=1
    return BME280_WriteReg(hspi, BME280_REG_CTRL_MEAS, 0x25);
}

HAL_StatusTypeDef BME280_WaitForReady(SPI_HandleTypeDef *hspi) {
    uint8_t s;
    do {
        BME280_ReadRegs(hspi, BME280_REG_STATUS, &s, 1);
    } while (s & 0x08);
    return HAL_OK;
}

HAL_StatusTypeDef BME280_ReadAll(SPI_HandleTypeDef *hspi, BME280_Calib *c,
                                 int32_t *t, uint32_t *p, uint32_t *h) {
    uint8_t d[8];
    BME280_ReadRegs(hspi, BME280_REG_PRESS_MSB, d, 8);
    int32_t adc_P = (d[0]<<12)|(d[1]<<4)|(d[2]>>4);
    int32_t adc_T = (d[3]<<12)|(d[4]<<4)|(d[5]>>4);
    int32_t adc_H = (d[6]<<8)|d[7];

    //int32_t adc_P = ( d[0] << 12 | d[1] << 4 | d[2] & 0xF);


    // --- Temperatur comp. ---
    int32_t var1 = ((((adc_T>>3) - ((int32_t)c->dig_T1<<1))) * c->dig_T2) >> 11;
    int32_t var2 = (((((adc_T>>4) - c->dig_T1)*((adc_T>>4) - c->dig_T1))>>12) * c->dig_T3) >> 14;
    int32_t t_fine = var1 + var2;
    *t = (t_fine*5 + 128) >> 8;  // 0.01 °C

    // --- Pressure comp. ---
    int64_t v1 = (int64_t)t_fine - 128000;
    int64_t v2 = v1*v1*c->dig_P6 + ((v1*c->dig_P5)<<17) + (((int64_t)c->dig_P4)<<35);
    v1 = ((v1*v1*c->dig_P3)>>8) + ((v1*c->dig_P2)<<12);
    v1 = (((((int64_t)1)<<47) + v1) * c->dig_P1) >> 33;
    if (v1==0) return HAL_ERROR;
    int64_t p64 = 1048576 - adc_P;
    p64 = (((p64<<31) - v2) * 3125) / v1;
    v1 = (c->dig_P9 * (p64>>13) * (p64>>13)) >> 25;
    v2 = (c->dig_P8 * p64) >> 19;
    *p = (uint32_t)((p64 + v1 + v2) >> 8);

    // --- Humidity comp. (64-bit) ---
    int64_t var_H = (int64_t)t_fine - 76800;
    var_H = (((((int64_t)adc_H)<<14) - (((int64_t)c->dig_H4)<<20) - ((int64_t)c->dig_H5 * var_H) + 16384) >> 15);
    var_H = (var_H * (((((var_H * c->dig_H6)>>10)
                    * (((var_H * c->dig_H3)>>11) + 32768))>>10)
                    + 2097152) + 8192) >> 14;
    var_H = var_H - (((((var_H>>15)*(var_H>>15))>>7) * c->dig_H1) >> 4);
    if (var_H < 0)         var_H = 0;
    if (var_H > 419430400) var_H = 419430400;
    *h = (uint32_t)(var_H >> 12) / 2;  // 0.001 %RH
    if (*h > 100.0f) *h = 100.0f;

    return HAL_OK;
}
