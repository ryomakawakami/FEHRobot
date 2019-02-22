#ifndef FEHIO_QUAD_H
#define FEHIO_QUAD_H

#include "derivative.h"
#include "MK60DZ10.h"
#include "adc16.h"

class QuadEncoder
{
public:
    QuadEncoder( FEHIO::FEHIOPin pin1, FEHIO::FEHIOPin pin2, FEHIO::FEHIOInterruptTrigger trigger );
    QuadEncoder( FEHIO::FEHIOPin pin1, FEHIO::FEHIOPin pin2);
    int Counts1();
    int Counts2();
    void ResetCounts();

private:
    FEHIO::FEHIOPin _pin1, _pin2;

    QuadEncoder();
    void Initialize( FEHIO::FEHIOPin pin1, FEHIO::FEHIOPin pin2, FEHIO::FEHIOInterruptTrigger trigger );
};

#endif // FEHIO_QUAD_H
