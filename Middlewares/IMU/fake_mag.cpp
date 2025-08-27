#include "fake_mag.h"

FakeMag::FakeMag()
{

}
void FakeMag::getHeading(int16_t* mx, int16_t* my, int16_t* mz)
{
    *mx = 0x1101;
    *my = 0x1101;
    *mz = 0x1101;
}