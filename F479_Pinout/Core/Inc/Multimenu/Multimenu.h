/*
  Multimenu.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/

#include "string.h"

class Multimenu
{
public:
  Multimenu(char header[]);
  bool scan_int(int *data);
  bool scan_float(float *data);
  virtual bool menu_head() = 0;
  virtual void print_menu_head(void) {};
  void sequence();
  char _header[50];
  bool _is_headed= false;
private:
  
};


Multimenu::Multimenu(char header[]) 
{
    strcpy(_header,header);
}

void Multimenu::sequence()
{
    this->_is_headed = false;
    this->print_menu_head();
    while (this->menu_head() == true) // no back choice
    {
        if (this->_is_headed == false){
            this->print_menu_head();
            this->_is_headed= true;
        }
        //delay(500);
    }
}

bool Multimenu::scan_int(int *data)
{
    return false;
}

bool Multimenu::scan_float(float *data)
{
    return false;
}

bool Multimenu::menu_head()
{
    int input = 0;
    if (this->scan_int(&input)) // check user enter any data
    {
        
        switch (input)
        {
        case 1:
            // run_demon();
            break;
        case 2:
            // run_func_t();
            break;
        case 3:
            // run_setting();
            break;
        default:
            //SerialCPU.println("Please try again!");
            return false;
        }
        return true;
    }
    return false;
}