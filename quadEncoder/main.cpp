#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>

#include "derivative.h"
#include "MK60DZ10.h"
#include "adc16.h"

typedef enum
{
    PortA,
    PortB,
    PortC,
    PortD,
    PortE
} GPIOPort;

typedef enum
{
    low,
    high
} GPIOValue;

typedef enum
{
    ADC0,
    ADC1
} ADCNumber;

const GPIOPort GPIOPorts[ 32 ] =
{
    PortB, PortB, PortB, PortB, PortB, PortB, PortB, PortB,
    PortC, PortC, PortC, PortC, PortC, PortC, PortA, PortA,
    PortA, PortA, PortA, PortA, PortA, PortA, PortA, PortA,
    PortA, PortE, PortE, PortE, PortE, PortE, PortD, PortD
};

const ADCNumber ADCNumbers[ 33 ] =
{
    ADC1, ADC1, ADC1, ADC1, ADC1, ADC1, ADC1, ADC1,
    ADC0, ADC0, ADC1, ADC1, ADC1, ADC1, ADC1, ADC0,
    ADC0, ADC1, ADC1, ADC0, ADC0, ADC1, ADC1, ADC0,
    ADC0, ADC0, ADC0, ADC0, ADC1, ADC0, ADC0, ADC0,
    ADC0
};

const int GPIOPinNumbers[ 32 ] =
{
    11, 10, 7, 6, 5, 4, 1, 0,
    0, 1, 8, 9, 10, 11, 17, 16,
    15, 14, 13, 12, 11, 10, 9, 8,
    7, 25, 24, 26, 27, 28, 1, 6
};

const int AnalogPinNumbers[ 33 ] =
{
    15, 14, 13, 12, 11, 10, 9, 8,
    14, 15, 4, 5, 6, 7, 17, 1,
    20, 1, 20, 0, 19, 0, 19, 11,
    10, 18, 17, 16, 16, 4, 5, 7,
    6
};

// Function used to enable interrupt register
void enable_irq_q(int irq)
{
    int div;
    div = (irq-16)/32;

    switch(div)
    {
        case 0x0:
            NVICICPR0 |= 1 << ((irq-16)%32);
            NVICISER0 |= 1 << ((irq-16)%32);
            break;
        case 0x1:
            NVICICPR1 |= 1 << ((irq-16)%32);
            NVICISER1 |= 1 << ((irq-16)%32);
            break;
        case 0x2:
            NVICICPR2 |= 1 << ((irq-16)%32);
            NVICISER2 |= 1 << ((irq-16)%32);
            break;
    }
}

//Interrupt port functions
long interrupt_counts_q1[32];
long interrupt_counts_q2[32];
void portB_isr_q()
{
    int pins[8] = { 11, 10, 7, 6, 5, 4, 1, 0 };

    for( int i=0 ; i<8 ; i++)
    {

        if( (PORTB_ISFR & (1<<pins[i])) != 0)
        {
            interrupt_counts_q1[i]++;
            interrupt_counts_q2[i]++;
            PORTB_ISFR &= (1<<pins[i]);
        }
    }
}
void portC_isr_q()
{
    int pins[6] = { 0, 1, 8, 9, 10, 11 };

    for( int i=0 ; i<6 ; i++)
    {
        if( (PORTC_ISFR & (1<<pins[i])) != 0)
        {
            interrupt_counts_q1[i+8]++;
            interrupt_counts_q2[i+8]++;
            PORTC_ISFR &= (1<<pins[i]);
        }
    }
}
void portA_isr_q()
{
    int pins[11] = { 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7 };

    for( int i=0 ; i<11 ; i++)
    {
        if( (PORTA_ISFR & (1<<pins[i])) != 0)
        {
            interrupt_counts_q1[i+14]++;
            interrupt_counts_q2[i+14]++;
            PORTA_ISFR &= (1<<pins[i]);
        }
    }
}
void portE_isr_q()
{
    int pins[5] = { 25, 24, 26, 27, 28 };

    for( int i=0 ; i<5 ; i++)
    {
        if( (PORTE_ISFR & (1<<pins[i])) != 0)
        {
            interrupt_counts_q1[i+25]++;
            interrupt_counts_q2[i+25]++;
            PORTE_ISFR &= (1<<pins[i]);
        }
    }
}

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

// Begin Functions for Quad Encoder Pin Type
QuadEncoder::QuadEncoder( FEHIO::FEHIOPin pin1, FEHIO::FEHIOPin pin2, FEHIO::FEHIOInterruptTrigger trigger )
{
    Initialize( pin1, pin2, trigger );
}
QuadEncoder::QuadEncoder( FEHIO::FEHIOPin pin1, FEHIO::FEHIOPin pin2)
{
    FEHIO::FEHIOInterruptTrigger trigger(FEHIO::EitherEdge);
    Initialize( pin1, pin2, trigger );
}

QuadEncoder::QuadEncoder()
{

}

void QuadEncoder::Initialize( FEHIO::FEHIOPin pin1, FEHIO::FEHIOPin pin2, FEHIO::FEHIOInterruptTrigger trigger )
{
    // store selected pin numbers in class
    _pin1 = pin1;
    _pin2 = pin2;
    unsigned char trig = (unsigned char)trigger;
    switch( GPIOPorts[ (int)_pin1 ] )
    {
        case PortA:
        {
            PORT_PCR_REG( PORTA_BASE_PTR, GPIOPinNumbers[ (int)_pin1 ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            GPIOA_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin1 ] ) );
            enable_irq_q(INT_PORTA);
            break;
        }
        case PortB:
        {
            PORT_PCR_REG( PORTB_BASE_PTR, GPIOPinNumbers[ (int)_pin1 ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            GPIOB_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin1 ] ) );
            enable_irq_q(INT_PORTB);
            break;
        }
        case PortC:
        {
            PORT_PCR_REG( PORTC_BASE_PTR, GPIOPinNumbers[ (int)_pin1 ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            GPIOC_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin1 ] ) );
            enable_irq_q(INT_PORTC);
            break;
        }
        case PortD:
        {
            //Port D is already in use for power reset pin. Therefore Digital Encoders cannot be used on P3_6 and P3_7
            //PORT_PCR_REG( PORTD_BASE_PTR, GPIOPinNumbers[ (int)_pin1 ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA) | PORT_PCR_PFE_MASK );
            //GPIOD_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin1 ] ) );
            //enable_irq_q(INT_PORTD);
            break;
        }
        case PortE:
        {
            PORT_PCR_REG( PORTE_BASE_PTR, GPIOPinNumbers[ (int)_pin1 ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            GPIOE_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin1 ] ) );
            enable_irq_q(INT_PORTE);
            break;
        }
    }
    switch( GPIOPorts[ (int)_pin2 ] )
    {
        case PortA:
        {
            PORT_PCR_REG( PORTA_BASE_PTR, GPIOPinNumbers[ (int)_pin2 ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            GPIOA_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin2 ] ) );
            enable_irq_q(INT_PORTA);
            break;
        }
        case PortB:
        {
            PORT_PCR_REG( PORTB_BASE_PTR, GPIOPinNumbers[ (int)_pin2 ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            GPIOB_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin2 ] ) );
            enable_irq_q(INT_PORTB);
            break;
        }
        case PortC:
        {
            PORT_PCR_REG( PORTB_BASE_PTR, GPIOPinNumbers[ (int)_pin2 ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            GPIOB_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin2 ] ) );
            enable_irq_q(INT_PORTC);
            break;
        }
        case PortD:
        {
            //Port D is already in use for power reset pin. Therefore Digital Encoders cannot be used on P3_6 and P3_7
            //PORT_PCR_REG( PORTB_BASE_PTR, GPIOPinNumbers[ (int)_pin2 ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            //GPIOB_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin2 ] ) );
            //enable_irq_q(INT_PORTD);
            break;
        }
        case PortE:
        {
            PORT_PCR_REG( PORTB_BASE_PTR, GPIOPinNumbers[ (int)_pin2 ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            GPIOB_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin2 ] ) );
            enable_irq_q(INT_PORTE);
            break;
        }
    }
}

int QuadEncoder::Counts1()
{
    return interrupt_counts_q1[_pin1];
}

int QuadEncoder::Counts2()
{
    return interrupt_counts_q2[_pin2];
}

void QuadEncoder::ResetCounts()
{
    interrupt_counts_q1[_pin1] = 0;
    interrupt_counts_q2[_pin2] = 0;
}

int main(void)
{
    float x,y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    QuadEncoder enc(FEHIO::P0_0, FEHIO::P0_1);

    while(1) {
        LCD.Write(enc.Counts1());
        LCD.Write(" ");
        LCD.WriteLine(enc.Counts2());

        Sleep(100);
    }

    return 0;
}
