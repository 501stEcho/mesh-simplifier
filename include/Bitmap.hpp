#pragma once

#include <vector>
#include <iostream>

class Bitmap
{
    public:
        Bitmap(bool initialValue = false)
        {
            baseValue = initialValue;
        }

        Bitmap(std::size_t size, bool initialValue = false)
        {
            uint8_t value;
            
            if (initialValue)
                value = 0xFF;
            else
                value = 0x00;
            baseValue = initialValue;

            bits = std::vector<uint8_t>(size, value);
        }

        void set(std::size_t index, bool value)
        {
            if (index >= bits.size() * 8)
                resize(index + 1);

            size_t byteIndex = index / 8;
            size_t bitIndex = index % 8;

            if (value)
                bits[byteIndex] |= (1 << bitIndex);
            else
                bits[byteIndex] &= ~(1 << bitIndex);
        }

        void reset(std::size_t index)
        {
            set(index, baseValue);
        }

        bool isActive(std::size_t index)
        {
            if (index >= bits.size() * 8)
                return baseValue;

            size_t byteIndex = index / 8;
            size_t bitIndex = index % 8;
            return bits[byteIndex] & (1 << bitIndex);
        }

    protected:
        void resize(std::size_t newSize)
        {
            uint8_t value;
            if (baseValue)
                value = 0xFF;
            else
                value = 0x00;

            bits.resize((newSize + 7) / 8, value);
        }

        std::vector<uint8_t> bits;
        bool baseValue;
};
