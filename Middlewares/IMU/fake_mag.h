#ifndef __FAKE_MAG_H
#define __FAKE_MAG_H
#include "stdint.h"
class FakeMag
{
    public:
        FakeMag();
        bool initialize();
        bool testConnection();
        void getHeading(int16_t* mx, int16_t* my, int16_t* mz);
};

#endif