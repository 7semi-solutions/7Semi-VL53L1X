#pragma once
#include <Arduino.h>

template <typename Bus>
class BusIO_7Semi
{

public:
    /**
     * Constructor
     *
     * - Stores interface reference
     */
    BusIO_7Semi(Bus &busRef) : bus(busRef) {}

    /** read 8-bit */
    inline bool read(uint16_t reg, uint8_t &value)
    {
        return (bus.read(reg, &value, 1) == 0);
    }

    /** read 16-bit */
    inline bool read(uint16_t reg, uint16_t &value)
    {
        uint8_t data[2];
        if (bus.read(reg, data, 2) != 0)
            return false;
        value = (data[0] << 8) | data[1];
        return true;
    }

    /** read burst */
    inline bool read(uint16_t reg, uint8_t *data, uint32_t len)
    {
        return (bus.read(reg, data, len) == 0);
    }

    /** write 8-bit */
    inline bool write(uint16_t reg, uint8_t value)
    {
        return (bus.write(reg, &value, 1) == 0);
    }

    /** write 16-bit */
    inline bool write(uint16_t reg, uint16_t value)
    {
        uint8_t data[2] = {(uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
        return (bus.write(reg, data, 2) == 0);
    }

    /** write burst */
    inline bool write(uint16_t reg, const uint8_t *data, uint32_t len)
    {
        return (bus.write(reg, data, len) == 0);
    }

    /**
     * readBits (8-bit register)
     */
    bool readBits(uint16_t reg, uint8_t pos, uint8_t len, uint8_t &value)
    {
        if (len == 0 || len > 8 || (pos + len) > 8)
            return false;

        uint16_t reg_val;
        if (!read(reg, reg_val))
            return false;

        uint8_t mask = ((uint8_t)1 << len) - 1;

        value = (reg_val >> pos) & mask;
        return true;
    }

    /**
     * readBits (16-bit register)
     */
    bool readBits(uint16_t reg, uint8_t pos, uint8_t len, uint16_t &value)
    {
        if (len == 0 || len > 16 || (pos + len) > 16)
            return false;

        uint16_t reg_val;
        if (!read(reg, reg_val))
            return false;

        uint16_t mask = ((uint16_t)1 << len) - 1;

        value = (reg_val >> pos) & mask;
        return true;
    }

    /**
     * writeBits (8-bit register)
     *
     * - Modifies specific bits in 8-bit register
     */
    bool writeBits(uint16_t reg, uint8_t pos, uint8_t len, uint8_t value)
    {
        if (len == 0 || len > 8 || (pos + len) > 8)
            return false;

        uint16_t reg_val;
        if (!read(reg, reg_val))
            return false;

        uint8_t mask = ((uint8_t)1 << len) - 1;

        reg_val &= ~(mask << pos);
        reg_val |= (value & mask) << pos;

        return write(reg, reg_val);
    }

    /**
     * writeBits (16-bit register)
     *
     * - Modifies specific bits in 16-bit register
     */
    bool writeBits(uint16_t reg, uint8_t pos, uint8_t len, uint16_t value)
    {
        if (len == 0 || len > 16 || (pos + len) > 16)
            return false;

        uint16_t reg_val;
        if (!read(reg, reg_val))
            return false;

        uint16_t mask = ((uint16_t)1 << len) - 1;

        reg_val &= ~(mask << pos);
        reg_val |= (value & mask) << pos;

        return write(reg, reg_val);
    }

    bool readBit(uint16_t reg, uint8_t pos, uint8_t &value)
    {
        return readBits(reg, pos, 1, value);
    }

    bool readBit(uint16_t reg, uint8_t pos, uint16_t &value)
    {
        return readBits(reg, pos, 1, value);
    }

    bool writeBit(uint16_t reg, uint8_t pos, uint8_t value)
    {
        return writeBits(reg, pos, 1, value );
    }
    bool writeBit(uint16_t reg, uint8_t pos, uint16_t value)
    {
        return writeBits(reg, pos, 1, value);
    }

private:
    Bus &bus;
};