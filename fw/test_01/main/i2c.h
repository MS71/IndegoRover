#ifndef _I2C_H_
#define _I2C_H_

#define I2CIO_PCF8574_WHEEL_L       0x0000
#define I2CIO_PCF8574_WHEEL_R       0x0001
#define I2CIO_PCF8574_STOP          0x0002
#define I2CIO_PCF8574_BEEP          0x0003

#define I2CIO_PCF8574_PON           0x0005
#define I2CIO_PCF8574_CAM_RST       0x0006
#define I2CIO_PCF8574_CAM_PWDN      0x0007

void i2c_init();
void i2c_handle();
void i2c_exit();
void i2c_set_gpio(uint16_t pin,uint8_t value);

#endif
