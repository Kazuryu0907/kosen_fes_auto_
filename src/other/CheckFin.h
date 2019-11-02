#include <math.h>

class CheckFin
{
    public:
        CheckFin(int arraylengh,int c):TargetcountFin(c),arrayLength(arraylengh){};
        void update(double term);
        bool isEnd();
        void reset();
    private:
        int TargetcountFin;
        int countFin;
        int arrayLength;
        double array[100];
        double round(double,int);
        void shift(double*);
        bool check;
};