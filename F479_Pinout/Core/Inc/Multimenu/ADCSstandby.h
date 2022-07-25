/*
  ADCSstandby.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#include "Multimenu.h"

class ADCSstandby : public Multimenu
{
public:
    ADCSstandby(char header[]);
    bool menu_head();
    void print_menu_head();
    void set_param();

private:
};

ADCSstandby::ADCSstandby(char header[]) : ADCSstandby(header)
{
}

bool ADCSstandby::menu_head()
{
    int input = 0;
    /*
        if (this->scan_int(&input)) // check user enter any data
        {
            // SerialCPU.println(input);
            switch (input)
            {
            case 0:
                return false;
                break;
            case 1:
                this->set_param();
                input = 0;
                break;
            default:
                SerialCPU.println("Please try again!");
            }
            this->_is_headed = false;
        }
        */
    return true;
}

void ADCSstandby::print_menu_head()
{
    /*
    SerialCPU.println("#####+++++" + this->_header + "+++++#####");
    SerialCPU.println("______________________________________________\n");
    SerialCPU.println("Enter '1' : Set Bdot Parameter");
    SerialCPU.println("Enter '0' : Back to previous menu");
    SerialCPU.println("______________________________________________\n");
    this->_is_headed = true;
    */
}

void ADCSstandby::set_param()
{
}