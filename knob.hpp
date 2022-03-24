#ifndef KNOB
#define KNOB

#include <cstdint>

class Knob
{
public:
    Knob()
    {
        knobRotation = 0;
        previousstateBA = 0;
    }

    Knob(uint8_t start_val)
    {
        knobRotation = start_val;
    }

    //variable getter
    //read current knob rotation value
    uint8_t readknob()
    {
        return knobRotation;
    }
    //variable setter
    //update the rotation value
    void writeknob(int8_t rotate)
    {
        __atomic_add_fetch(&knobRotation, rotate, __ATOMIC_RELAXED);
        //increase or decrease volume in one direction
        if (knobRotation > 16)
            knobRotation = 16;
        if (knobRotation < 0)
            knobRotation = 0;
    }

    void restore()
    {
        __atomic_store_n(&knobRotation, 0, __ATOMIC_RELAXED);
    }

    void updateknob(uint8_t currentstateBA)
    {
        if (previousstateBA == 0b00 && currentstateBA == 0b01)
        {
            writeknob(1);
        }
        else if (previousstateBA == 0b00 && currentstateBA == 0b10)
        {
            writeknob(-1);
        }
        else if (previousstateBA == 0b00 && currentstateBA == 0b01)
        {
            writeknob(0);
        }
        else if (previousstateBA == 0b00 && currentstateBA == 0b11)
        {
            writeknob(-2);
        }
        else if (previousstateBA == 0b01 && currentstateBA == 0b00)
        {
            writeknob(-1);
        }
        else if (previousstateBA == 0b01 && currentstateBA == 0b01)
        {
            writeknob(0);
        }
        else if (previousstateBA == 0b01 && currentstateBA == 0b10)
        {
            writeknob(-2);
        }
        else if (previousstateBA == 0b01 && currentstateBA == 0b11)
        {
            writeknob(1);
        }
        else if (previousstateBA == 0b10 && currentstateBA == 0b00)
        {
            writeknob(1);
        }
        else if (previousstateBA == 0b10 && currentstateBA == 0b01)
        {
            writeknob(2);
        }
        else if (previousstateBA == 0b10 && currentstateBA == 0b10)
        {
            writeknob(0);
        }
        else if (previousstateBA == 0b10 && currentstateBA == 0b11)
        {
            writeknob(-1);
        }
        else if (previousstateBA == 0b11 && currentstateBA == 0b00)
        {
            writeknob(-2);
        }
        else if (previousstateBA == 0b11 && currentstateBA == 0b01)
        {
            writeknob(-1);
        }
        else if (previousstateBA == 0b11 && currentstateBA == 0b10)
        {
            writeknob(1);
        }
        else if (previousstateBA == 0b11 && currentstateBA == 0b11)
        {
            writeknob(0);
        }
        __atomic_store_n(&previousstateBA, currentstateBA, __ATOMIC_RELAXED);
        // previousstateBA = currentstateBA;
    }

private:
    //encapsulate rotation variable as private because we
    //want the function be reentrant
    // uint8_t knobRotation;
    int8_t knobRotation;
    uint8_t previousstateBA;
};

#endif
