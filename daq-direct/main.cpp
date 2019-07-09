#include <iostream>
#include <chrono>
#include <thread>
#include <ratio>
#include "826api.h"




// ******************** SELECT PROGRAM TO RUN  ******************
#define JUST_COUNT
int chosen_motor = 0;
// **************************************************************



// ******************** FOR LINUX KEYBOARD LOOP BREAK ***********
#include <stdio.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <stropts.h>

int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
// *************************************************************

#include <deque>

using namespace std;



// Helper functions for getPosition & setForce
//==============================================================================
double getMotorAngle(int motor, double cpr) {
    const double pi = 3.14159265359;
    const unsigned int maxdata = 0xFFFFFFFF; // 32 bit

    uint encoderValue;
    uint ctstamp;
    uint reason;
    S826_CounterSnapshot(0,motor);
    S826_CounterSnapshotRead(0,motor,&encoderValue,&ctstamp,&reason,0);

    uint status;
    S826_CounterStatusRead(0,0,&status);
    //std::cout << "Reading from channel: " << motor << " Status: " << (status)  << " Counter: " << encoderValue <<  std::endl;


    if(encoderValue >= maxdata/2)
        return -1.0*2.0*pi*(maxdata-encoderValue)/cpr;
    return 2.0*pi*encoderValue/cpr;
}

void setVolt(double v, int motor){
    if(v > 10 || v< -10) { printf("Volt outside +/- 10 Volt\n"); return; }

    // -10V to +10V is mapped from 0x0000 to 0xFFFF
    unsigned int signal = (v+10.0)/20.0 * 0xFFFF;
    std::cout << "Sets volt " << v << " with number " << signal << "\n";
    S826_DacDataWrite(0,motor,signal,0);
}


void AdcHandler(void){
    while(1){
        int errcode;
        int slotval[16];
        uint slotlist = 1;
        errcode = S826_AdcRead(0,slotval,0,&slotlist,S826_WAIT_INFINITE);
        if(errcode != S826_ERR_OK)
            break;
        int raw = (slotval[0] & 0xFFFF);         // 16 bit signed integer
        double deg = 90*double(raw)/0x7FFF;
        cout << "raw adc data = " << raw << "\t " << deg << " degrees\n";
    }
}

int CreateDelay(uint board, uint ctr, uint delay)
{
     S826_CounterStateWrite(board, ctr, 0);
// Configure the delay timer:
    S826_CounterModeWrite(board, ctr,                // Configure counter mode:
      S826_CM_K_1MHZ |                                //   clock source = 1 MHz internal
      S826_CM_PX_START | S826_CM_PX_ZERO |            //   preload @start and counts==0
      S826_CM_UD_REVERSE);
S826_CounterPreloadWrite(board, ctr, 0, delay);
S826_CounterSnapshotConfigWrite(board, ctr,
S826_SSRMASK_ZERO,
S826_BITWRITE);
return S826_CounterStateWrite(board, ctr, 1);
}
int WaitForDelay(uint board, uint ctr)
{
return S826_CounterSnapshotRead(board, ctr,
NULL, NULL, NULL,
S826_WAIT_INFINITE);
}



void readSSI(){
    constexpr uint clock_chan = 24;
    constexpr uint data_chan = 25;
    using namespace std::chrono;
    duration<int, std::micro> d{1};
    duration<int, std::micro> d26{26};
    int data[10];

    uint dio_clock[] = {0,1<<(clock_chan-24)}; // DIO0-23,DIO24-47
    uint input[2]; // DIO0-23,DIO24-47

    // make sure clock starts at high
    // set clock channel high (which is logic 0)
    // and data channel high in order to read from channel
    uint all_high[] = {0,0};
    S826_DioOutputWrite(0,all_high,S826_BITWRITE);


    //std::this_thread::sleep_for(d26);
  //  CreateDelay(0, 0, 26);
    WaitForDelay(0, 0);
    WaitForDelay(0, 0);
    WaitForDelay(0, 0);
    WaitForDelay(0, 0);
    WaitForDelay(0, 0);
    WaitForDelay(0, 0);
    WaitForDelay(0, 0);
    WaitForDelay(0, 0);
    WaitForDelay(0, 0);

    // set clock channel low (which is logic 1)
    S826_DioOutputWrite(0,dio_clock,S826_BITSET);
    //std::this_thread::sleep_for(d);
//    CreateDelay(0, 0, 10);
    //WaitForDelay(0, 0);

    for(int i=0;i<10;++i){
        // set clock channel high
        S826_DioOutputWrite(0,dio_clock,S826_BITCLR);
//        std::this_thread::sleep_for(d);
//        CreateDelay(0, 0, 10);
       // WaitForDelay(0, 0);

        // read data
        S826_DioInputRead(0,input);
        data[i] = input[1]; // Includes data & clock channels
        //int bit = (input[1] >> (data_chan-24)) & 1;

        // set clock low
        S826_DioOutputWrite(0,dio_clock,S826_BITSET);
        //std::this_thread::sleep_for(d);
        //WaitForDelay(0, 0);

    }

    // Write result
    return;
    std::cout << "Data: ";
    for(int i=0;i<10;++i){
        std::cout << data[i] << " ";
    }
    std::cout << std::endl;
}


int main(int argc, char** argv) {


    std::cout << "Have " << argc << " arguments:" << std::endl;
    for (int i = 0; i < argc; ++i) {
        std::cout << argv[i] << std::endl;
    }
    if(argc>1) chosen_motor = atoi(argv[1]);


    constexpr int sleep_us = 1000;
    constexpr unsigned int log_entries=10;


    std::cout << "Open: " << S826_SystemOpen();

    //CreateDelay(0, 0, 1);




    // Initialize counters for channel 0,1,2 AND 3,4,5 (gimbal)
    for(unsigned int i=0;i<6;++i){
        S826_CounterStateWrite(0,i,0);

        constexpr uint multiples_of_20ns = 65535; // 65535 = 1.3107 ms (maximum)
        constexpr uint ENABLE_IX_FILTER = (1 << 31);
        constexpr uint ENABLE_CK_FILTER = (1 << 30);

        //S826_CounterFilterWrite(0,i,0);



        //S826_CounterFilterWrite(0,i,multiples_of_20ns | ENABLE_IX_FILTER | ENABLE_CK_FILTER);
        S826_CounterFilterWrite(0,i,multiples_of_20ns | ENABLE_IX_FILTER );

        S826_CounterModeWrite(0,i,0x70);
        S826_CounterPreloadWrite(0,i,0,0); // Load 0
        S826_CounterPreload(0,i,0,0);


        // Configure snapshot to occur when IX signal either rises or falls,
        // this is to get a callback when i.e. a button is pressed that we
        // have wired to IX channel (since we are not using the encoder's IX)
        S826_CounterSnapshotConfigWrite(0, i, S826_SSRMASK_IXRISE |
                                              S826_SSRMASK_IXFALL, S826_BITWRITE);

        // Enable
        S826_CounterStateWrite(0,i,1);

    }

    // ADC
    constexpr uint SAMPLING_PERIOD = 50000;  // 20samples/s
    constexpr uint TSETTLE = SAMPLING_PERIOD-3;
    S826_AdcSlotConfigWrite(0,0,15,TSETTLE,S826_ADC_GAIN_2); // -5v to +5v
    S826_AdcSlotlistWrite(0,1,S826_BITWRITE);
    S826_AdcTrigModeWrite(0,0); // continous
    S826_AdcEnableWrite(0,1);


    uint filter_chans[]={0x00FFFFFF,0x00FFFFFF};
    S826_DioFilterWrite(0,10,filter_chans);




/*
    for(int i=0;i<6;++i){
        S826_CounterStateWrite(0,i,0);
        S826_CounterModeWrite(0,i,0x70);
        S826_CounterPreloadWrite(0,i,0,0); // Load 0
        S826_CounterPreload(0,i,0,0);

    }
*/
    /*
    S826_DacRangeWrite(0,0,3,0); //-10 to 10 V
    S826_DacDataWrite(0,0,uint(0xFFFF/2.0),0); // Channel 4,5,6 = Pin 41,43,45
    S826_DacRangeWrite(0,1,3,0); //-10 to 10 V
    S826_DacDataWrite(0,1,uint(0xFFFF/2.0),0); // Channel 4,5,6 = Pin 41,43,45
    S826_DacRangeWrite(0,2,3,0); //-10 to 10 V
    S826_DacDataWrite(0,2,uint(0xFFFF/2.0),0); // Channel 4,5,6 = Pin 41,43,45

    //S826_DacRangeWrite(0,3,3,0); //-10 to 10 V
    //S826_DacDataWrite(0,3,uint(0xFFFF/2.0),0); // Channel 4,5,6 = Pin 41,43,45
    //setVolt(1.0,2);

    S826_DacRangeWrite(0,3,0,0); //-10 to 10 V
    S826_DacDataWrite(0,3,uint(0xFFFF),0); // Channel 4,5,6 = Pin 41,43,45

    */
/*

    char q;
    cin >> q;
    setVolt(2.0,0);

    cin >> q;


    S826_DacDataWrite(0,3,0x0,0); // Channel 4,5,6 = Pin 41,43,45

}
*/
#define NORMAL

#ifdef NORMAL
    for(unsigned int i=0;i<3;++i){
        // And range of analoge signal to escon
        S826_DacRangeWrite(0,i,3,0); //-10 to 10 V
        S826_DacDataWrite(0,i,0xFFFF/2,0); // Channel 4,5,6 = Pin 41,43,45

        // Enable ESCON driver by a digial signal, which in our case
        // is an analogue signal due to the fact that we only have the
        // analgoue and encoder breakout board of the S826.
        //
        //S826_DacDataWrite(0,4+i,0xFFFF,0); // Channel 4,5,6 = Pin 41,43,45

    }
    S826_DacRangeWrite(0,3,0,0); // 0 to 5 V
//    S826_DacRangeWrite(0,5,0,0); // 0 to 5 V
//    S826_DacRangeWrite(0,6,0,0); // 0 to 5 V
    S826_DacDataWrite(0,3,0xFFFF,0); // Channel 4,5,6 = Pin 41,43,45
//    S826_DacDataWrite(0,5,0x0,0); // Channel 4,5,6 = Pin 41,43,45
//    S826_DacDataWrite(0,6,0x0,0); // Channel 4,5,6 = Pin 41,43,45

//    setVolt(0,0);


    cout << "Ready?\n";

    // Wait for new key hit
    while(!_kbhit());
    char pelle;
    cin >> pelle;



    int printcount=0;
    double prev_deg=0;
    int active_phase = 1;
    double goal_deg=0;

    deque<double> error_history;

    double v=0;


    std::cout << "hej\n";

    int on=0;

    while(active_phase){
        if(_kbhit()){
            cin >> pelle;

            if(pelle=='2')
                goal_deg += 5;
            if(pelle=='1')
                goal_deg -= 5;
            if(pelle=='q') {
                active_phase=0;
                continue;
            }
        }

#ifdef JUST_COUNT
        uint ctstamp;
        uint reason;
        uint counter;
        const uint chan = chosen_motor;

        //readSSI();
        //continue;

/*
        constexpr uint clock_chan = 24;
        constexpr uint data_chan = 25;
        using namespace std::chrono;
        duration<int, std::micro> d{20};
        int data[]={0,0,0,0,0,0,0,0,0,0};

        uint dios_clock[] = {0,1<<(clock_chan-24)}; // DIO0-23,DIO24-47
        uint dios_data[] = {0,1<<(data_chan-24)}; // DIO0-23,DIO24-47
        uint input[2]; // DIO0-23,DIO24-47

        // clear to be able to read input
        S826_DioOutputWrite(0,dios_data,S826_BITCLR);
        S826_DioOutputWrite(0,dios_clock,S826_BITCLR);
*/

        // set clock channel low (which is logic 1)
        /*
        if(on){

            S826_DioOutputWrite(0,dios_clock,S826_BITSET);
            on=0;
        } else {
            S826_DioOutputWrite(0,dios_clock,S826_BITCLR);
            on=1;
        }
        std::this_thread::sleep_for(d);
        */
/*

        S826_DioInputRead(0,input);
        int bit = (input[1] >> (data_chan-24)) & 1;
        data[0] = input[1];//bit;
        std::cout << (data[0]? " low 0v" : "high 5v") << " data " <<  data[0] <<"\n";
*/



/*
        for(int ix=0;ix<1;++ix){
            //int a = S826_CounterSnapshotRead(0,ix,&counter,&ctstamp,&reason,S826_WAIT_INFINITE);
            //S826_CounterSnapshot(0,0);

            int a = S826_CounterSnapshotRead(0,ix,&counter,&ctstamp,&reason,S826_WAIT_INFINITE); // wait 1us
            //std::cout << a << "\n";
            if(a != S826_ERR_NOTREADY){ // nothing in buffer
                // do counter snapshot
                //S826_CounterSnapshot(0,chan);
                //a = S826_CounterSnapshotRead(0,chan,&counter,&ctstamp,&reason,0);
                std::cout << a << " INDEX! Reading from channel: " << ix << " Reason: " << reason << " Counter: " << counter << std::endl;
                //std::cout << "hej\n";
            }
        }

*/
        //uint status;
        //S826_CounterStatusRead(0,0,&status);


        //if(reason!=128)
        //if(reason!=128){
            //while(!_kbhit());
            //char pelle;
            //cin >> pelle;
        //    std::cout << "\n\n";
        //}


        //std::cout << a << " Reading from channel: " << chan << " Reason: " << reason << " Counter: " << counter << std::endl;

        uint counts;

        for(int i=0;i<6;++i){
        S826_CounterRead(0,i,&counts);
        std::cout << "counts: " << counts << " chan: " << i << "\n";
}
        //S826_CounterRead(0,1,&counts);
        //std::cout << "counts: " << counts << " chan: 1\n";

        // Sleep 1ms
        //using namespace std::chrono;
        //duration<int, std::micro> dd{10000};
        //std::this_thread::sleep_for(dd);

v =0;
        //v-=0.01;
        //if(v<-2) v=0;
        //setVolt(v,0);

        //AdcHandler();

/*
        int errcode;
        int slotval[16];
        uint slotlist = 1;
        errcode = S826_AdcRead(0,slotval,0,&slotlist,S826_WAIT_INFINITE);
        if(errcode != S826_ERR_OK)
            break;
        int raw = (slotval[0] & 0xFFFF);         // 16 bit signed integer
        double adcdeg = 90*double(raw)/0x7FFF;
        //cout << "raw adc data = " << raw << "\t " << adcdeg << " degrees\n";
*/

        //setVolt(2,1);
        //setVolt(0.5,2);
        using namespace std::chrono;
        duration<int, std::micro> dd{sleep_us};
        std::this_thread::sleep_for(dd);

        continue;

#endif
        const double theta = getMotorAngle(chosen_motor, 4000);
        //std::cout << theta << "\n";
        const double deg = 360.0*theta/(3.141592*2.0);


        // Motor current
        double current = 0;
        if(deg < 360 && deg > -360){

            //double goal = (active_phase-1)*10;
            //if(active_phase == 1) goal = 20;
            double error = goal_deg-deg;


            double sum=0;
            for(auto e : error_history)
                sum+=e;

            error_history.push_back(error);
            double prev_error = goal_deg-prev_deg;
            if(error_history.size()>log_entries){
                prev_error = error_history.front();
                error_history.pop_front();
            }



            double delta_t = 0.000001*sleep_us*log_entries;
            double error_prim = (error - prev_error)/delta_t;
            double error_integrative = delta_t*(error+prev_error)/2;

            // Exact integrative
            //error_integrative = sum;


            double P = 0.1;
            double I = 0.0;
            double D = 0.0005;

            current = P*error + I*error_integrative + D*error_prim;
            if(!(printcount%100))
                cout << "Goal: " << goal_deg << " Error: " << error << " P*: " << P*error
                     << " Ierror: " << error_integrative << " I*: " << I*error_integrative
                     << " error': " << error_prim << " D*: " << D*error_prim << " \n";

            // max 3 amps
            if(current>3) current=3;
            if(current<-3) current =-3;

        }
        double signal = -10*current/3; // volt signal 10 gives 3 amps.
        setVolt(signal,chosen_motor);

        //cout << 10*i/3 << "\n";
        //setVolt(0.0,0); // 10 = 3A, 1=0.3A

        if(!(printcount++%100))
            cout << "Angle: " << theta << " rad  " << deg << " deg. Current: " << current << " amps (signal: " << signal <<  " v)\n\n";


        prev_deg = deg;

        // Sleep 1ms
        //using namespace std::chrono;
        //duration<int, std::micro> d{sleep_us};
        std::this_thread::sleep_for(dd);
    }



    // Disable power
    S826_DacDataWrite(0,4,0x0,0); // Channel 4 = Pin 41
    S826_DacDataWrite(0,5,0x0,0); // Channel 5 = Pin 43
    S826_DacDataWrite(0,6,0x0,0); // Channel 6 = Pin 45

    cout << "Done.\n";

    // Wait for new key hit
    while(!_kbhit());




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

#endif
