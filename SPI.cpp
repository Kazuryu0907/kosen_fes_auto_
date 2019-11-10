#include "mbed.h"
SPI spi(A3,D5,D3);
DigitalOut cs(D0);
Serial serial(USBTX,USBRX);


int main()
{
    cs = 1;
    while(1)
    {
        spi.write(0x75);
        int val = spi.write(0);
        serial.printf("%d\n",val);
    }
}