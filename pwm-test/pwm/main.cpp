#include <iostream>
#include <chrono>
#include <thread>
#include <ratio>
#include "826api.h"

using namespace std;


int main()
{
    cout << "Hello PWM World!" << endl;

    std::cout << "S826 System Open: " << S826_SystemOpen() << std::endl;


    uint data[2]= {1,0}; // DIO 0
    cout << "\nSet source: " << S826_DioOutputSourceWrite(0,data);

    cout << "\nCounter Mode: " << S826_CounterModeWrite(0,0,0x01682020);
    cout << "\nCounter State: " << S826_CounterStateWrite(0,0,1);
    cout << "\nCounter Preload (register 0): " << S826_CounterPreloadWrite(0,0,0,9000000); // On time in us (9s)
    cout << "\nCounter Preload (register 1): " << S826_CounterPreloadWrite(0,0,1,5000000); // Off time in us (5s)


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
        duration<int, std::micro> d{100000};
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

