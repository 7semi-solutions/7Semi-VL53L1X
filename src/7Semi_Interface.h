#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
/**
 * 7Semi Universal Interface Layer
 *
 * - Abstract communication layer for I2C / SPI
 * - Ensures sensor drivers remain hardware independent
 */
class Interface_7Semi {
public:

    virtual ~Interface_7Semi() {}

    /** 
     * beginI2C()
     *
     * - Initializes I2C peripheral
     * - Configures SDA / SCL pins if supported
     * - Sets communication clock
     *
     * Returns:
     * - true  → Initialization successful
     * - false → Initialization failed
     */
    virtual bool beginI2C(
        uint8_t address,
        TwoWire &wire,
        uint32_t speed,
        uint8_t sda = 255,
        uint8_t scl = 255) = 0;

    virtual bool beginSPI(
        uint8_t cs_pin,
        SPIClass &spi,
        uint32_t speed,
        uint8_t csk = 255,
        uint8_t miso = 255,
        uint8_t mosi = 255) = 0;


    /** 
     * read()
     *
     * - Reads data from device register
     *
     * Parameters:
     * - reg  : Register address
     * - data : Output buffer
     * - len  : Number of bytes
     *
     * Returns:
     * - 0  → Success
     * - <0 → Error
     */
    virtual int8_t read(
        uint16_t reg,
        uint8_t *data,
        uint32_t len) = 0;

    /** 
     * write()
     *
     * - Writes data to device register
     *
     * Returns:
     * - 0  → Success
     * - <0 → Error
     */
    virtual int8_t write(
        uint16_t reg,
        const uint8_t *data,
        uint32_t len) = 0;

protected:
    /** Delay wrapper */
    static void delay_us(uint32_t us)
    {
        delayMicroseconds(us);
    }
};