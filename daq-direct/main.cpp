#include <iostream>
#include <chrono>
#include <thread>
#include <ratio>
#include "826api.h"

using namespace std;



// Helper functions for getPosition & setForce
//==============================================================================
double getMotorAngle(int motor, double cpr) {
    const double pi = 3.14159265359;
    const unsigned int maxdata = 0xFFFFFFFF; // 32 bit

    uint encoderValue;
    S826_CounterSnapshot(0,motor);
    uint ctstamp;
    uint reason;
    S826_CounterSnapshotRead(0,motor,&encoderValue,&ctstamp,&reason,0);

    if(encoderValue >= maxdata/2)
        return -1.0*2.0*pi*(maxdata-encoderValue)/cpr;
    return 2.0*pi*encoderValue/cpr;
}

void setVolt(double v, int motor){
    if(v > 10 || v< -10) { printf("Volt outside +/- 10 Volt\n"); return; }

    // -10V to +10V is mapped from 0x0000 to 0xFFFF
    unsigned int signal = (v+10.0)/20.0 * 0xFFFF;
    S826_DacDataWrite(0,motor,signal,0);
}


int main(){



    int boardflags  = S826_SystemOpen();

    // Initialize counters for channel 0,1,2
    for(unsigned int i=0;i<3;++i){
        S826_CounterFilterWrite(0,i,0);
        S826_CounterModeWrite(0,i,0x70);
        S826_CounterPreloadWrite(0,i,0,0); // Load 0
        S826_CounterPreload(0,i,0,0);
        S826_CounterStateWrite(0,i,1);

        // And range of analoge signal to escon
        S826_DacRangeWrite(0,i,3,0); //-10 to 10 V

        // Enable ESCON driver by a digial signal, which in our case
        // is an analogue signal due to the fact that we only have the
        // analgoue and encoder breakout board of the S826.
        //
        S826_DacDataWrite(0,4+i,0xFFFF,0); // Channel 4,5,6 = Pin 41,43,45
    }


    // Disable power
    S826_DacDataWrite(0,4,0x0,0); // Channel 4 = Pin 41
    S826_DacDataWrite(0,5,0x0,0); // Channel 5 = Pin 43
    S826_DacDataWrite(0,6,0x0,0); // Channel 6 = Pin 45




}



#ifdef PWM_TEST_CODE

int main()
{
    cout << "Hello PWM World!" << endl;

    std::cout << "S826 System Open: " << S826_SystemOpen() << std::endl;


    /* Working PWM Example
    uint data[2]= {1,0}; // DIO 0
    S826_SafeWrenWrite(0,2);
    cout << "\nSet source: " << S826_DioOutputSourceWrite(0,data);
    cout << "\nCounter Preload (register 0): " << S826_CounterPreloadWrite(0,0,0,900); // On time in us (.9ms)
    cout << "\nCounter Preload (register 1): " << S826_CounterPreloadWrite(0,0,1,100); // Off time in us (.1ms)
    cout << "\nCounter Mode: " << S826_CounterModeWrite(0,0,0x01682020);
    cout << "\nForce load: " << S826_CounterPreload(0,0,0,0);
    cout << "\nCounter State (run): " << S826_CounterStateWrite(0,0,1);
    */


    //uint data[2]= {1,0}; // DIO 0
    S826_SafeWrenWrite(0,2);
    //cout << "\nSet source: " << S826_DioOutputSourceWrite(0,data);
    cout << "\nCounter Preload (register 0): " << S826_CounterPreloadWrite(0,0,0,50000); // On time in us (1ms)
    cout << "\nCounter Preload (register 1): " << S826_CounterPreloadWrite(0,0,1,50000); // Off time in us (1ms)
    cout << "\nCounter Mode: " << S826_CounterModeWrite(0,0,0x01682020+16);
    cout << "\nForce load: " << S826_CounterPreload(0,0,0,0);
    cout << "\nCounter State (run): " << S826_CounterStateWrite(0,0,1);

    while(true){
        uint ctstamp;
        uint reason;
        uint counter;
        S826_CounterSnapshot(0,0);
        S826_CounterSnapshotRead(0,0,&counter,&ctstamp,&reason,0);

        uint status;
        S826_CounterStatusRead(0,0,&status);

        std::cout << " Status: " << (status)  << " Counter: " << counter << std::endl;

        // Sleep 100ms
        using namespace std::chrono;
        duration<int, std::micro> d{1};
        std::this_thread::sleep_for(d);
    }


    S826_SystemClose();
    return 0;
}
































/*
S826_DioOutputSourceWrite(0,data2);




S826_CounterStateWrite(0,0,0);
S826_CounterStateWrite(0,1,0);
S826_CounterStateWrite(0,2,0);


S826_CounterPreloadWrite(0,0,0,500);//period*pwm_percent);     // On time in us
S826_CounterPreloadWrite(0,0,1,500);//period*(1-pwm_percent)); // Off time in us
S826_CounterModeWrite(0,0,0x01682020); //+131072 (invert)
S826_CounterStateWrite(0,0,1);
S826_CounterPreloadWrite(0,0,0,500);//period*pwm_percent);     // On time in us
S826_CounterPreloadWrite(0,0,1,500);//period*(1-pwm_percent)); // Off time in us
S826_CounterModeWrite(0,0,0x01682020); //+131072 (invert)
S826_CounterStateWrite(0,0,1);


uint data[2];
data[0] = 8; // DIO3 (bit 4 set = 8)
data[1] = 0;
S826_DioOutputWrite(0,data,1); // Clear DIO3 = pin 41 is set HIGH



std::cin >> s;

S826_DioOutputWrite(0,data,2); // Set DIO3 = pin 41 is set Low
*/



/*
 * Home-made pulse
 *
int a=2;
while(true){
    a = a==2? 1 : 2;

    S826_DioOutputWrite(0,data,a); // Set DIO 0,1,2 = pin 47,45,43 is set Low

    using namespace std::chrono;
    duration<int, std::micro> d{5000};
    std::this_thread::sleep_for(d);
}
*/
#endif

