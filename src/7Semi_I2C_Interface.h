#pragma once
#include "7Semi_Interface.h"
#include <Wire.h>

class I2C_Interface : public Interface_7Semi
{
public:
    TwoWire *i2c = nullptr;
    uint8_t address = 0;

    bool beginI2C(uint8_t addr, TwoWire &wire, uint32_t speed,
                  uint8_t sda = 255, uint8_t scl = 255)
    {
        address = addr;
        i2c = &wire;

#if defined(ESP32)
        if (sda != 255 && scl != 255)
            i2c->begin(sda, scl);
        else
            i2c->begin();
#else
        i2c->begin();
#endif
        i2c->setClock(speed);

        // Probe device
        i2c->beginTransmission(address);
        return (i2c->endTransmission() == 0);
    }

    int8_t read(uint16_t reg, uint8_t *data, uint32_t len) override
{
    if (!i2c || !data)
        return -1;

    i2c->beginTransmission(address);
    i2c->write(reg >> 8);
    i2c->write(reg & 0xFF);

    if (i2c->endTransmission(false) != 0)
        return -1;
    // Serial.print("R-Reg: ");
    // Serial.print(reg, HEX);
    // Serial.print(" |Data: ");
    uint32_t index = 0;
    uint32_t timeout;

    while (len > 0)
    {
        uint8_t request_len = (len > 32) ? 32 : len;

        timeout = millis() + 50;

        uint8_t received = i2c->requestFrom(address, request_len);

        if (received == 0)
            return -2;

        for (uint8_t i = 0; i < received; i++)
        {
            if (millis() > timeout)
                return -5;

            data[index++] = i2c->read();
        }

        len -= received;
        // for (int i = 0; i < received; i++)
        // {
        //     Serial.print(" ");
        //     Serial.print(data[i], HEX);
        // }
    }
        // Serial.println();

    return 0;
}

    bool beginSPI(uint8_t, SPIClass &, uint32_t,
              uint8_t, uint8_t, uint8_t) override
    {
        return false;
    }

    int8_t write(uint16_t reg, const uint8_t *data, uint32_t len) override
    {
        if (!i2c)
            return -1;

        i2c->beginTransmission(address);
        i2c->write(reg >> 8);
        i2c->write(reg & 0xFF);
        for (uint32_t i = 0; i < len; i++)
            i2c->write(data[i]);

        uint8_t status = i2c->endTransmission();
      
        // Serial.print("W-Reg: ");
        // Serial.print(reg, HEX);
        // Serial.print(" |Data: ");
        // for (int i = 0; i < len; i++)
        // {
        //     Serial.print(" ");
        //     Serial.print(data[i], HEX);
        // }
        // Serial.println();

        return (status == 0) ? 0 : -status;
    }
};