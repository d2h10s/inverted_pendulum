#include "mbed.h"
#include "QEI.h"

#define Sampling_Time 1000

#define m_rev 400
#define p_rev 4096

#define Kp 1
#define Kd 2
#define Ki 0.001

enum {CCW=0, CW=1};
// encoder CW가 -방향

BufferedSerial pc(USBTX, USBRX, 9600);
PwmOut pwm(D9);
DigitalOut dir(D8), led(LED1);
QEI motor(D2, D3, NC, 2048);
QEI pendulum(D4, D5, NC, 100);

int main()
{
    int64_t target = 400, p, e, y, integral = 0, prev_e = 0;
    double derivative = 0;
    Timer timer;
    timer.start();
    
    while (true) {
        if(timer.elapsed_time().count() > Sampling_Time){
            led=!led;
            e = target - p;
            integral += e * Sampling_Time;
            derivative = (e - prev_e) / Sampling_Time;
            y = Kp * e + Ki * integral + Kd * derivative;

            prev_e = e;

            printf("pulse:: %5lld   y:: %5lld\n", p, y);
            if(y>10000) y = 10000;
            else if(y<-10000) y = -10000;
            dir = y>0? CW:CCW;
            pwm = abs(y) / 50000.0;

            timer.reset();
        }
        p = motor.getPulses();
    }
}