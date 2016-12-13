#include "mbed.h"
#include "USBHID.h"
#include "WoodenDevice.h"
#include "FastPWM.h"


//We declare a USBHID device. Input out output reports have a length of 8 bytes
USBHID hid(8, 8);
 
//This report will contain data to be sent
HID_REPORT send_report;
HID_REPORT recv_report;

DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
DigitalOut myled4(LED4);

DigitalOut enableEscons(p14,0); 
 
InterruptIn  encoder0_A(p5);
InterruptIn  encoder0_B(p7);
InterruptIn  encoder1_A(p8);
InterruptIn  encoder1_B(p10);
InterruptIn  encoder2_A(p11);
InterruptIn  encoder2_B(p13);

Timeout msg_watchdog;




Serial pc(USBTX, USBRX); // tx, rx


int no_enc = 3;

int prev_state[3] = {-1,-1,-1};
bool encoder_raw[3][2] = {{false,false},{false,false},{false,false}};

PwmOut pwm[3]={p21,p22,p23};
PwmOut direction[3]={p24,p25,p26};

int counter[3] = {0,0,0};

// Pre  Cur  Dir  Dec   
// 0 0  0 1  +     1
// 0 0  1 0  -     2
// 0 1  1 1  +     7
// 0 1  0 0  -     4
// 1 1  1 0  +    14
// 1 1  0 1  -    13
// 1 0  0 0  +     8
// 1 0  1 1  -    11
//
//                0 1  2 3  4 5 6 7 8 9 10 11 12 13 14 15
//int stable[16] = {0,1,-1,0,-1,0,0,1,1,0, 0,-1, 0,-1, 1, 0}; //OLD
int stable[16] =   {0,-1,1,0,1,0,0,-1,-1,0, 0,1, 0,1, -1, 0};
 
void encoder_callback(int _encoder,int AB,bool value){
        int cur_state;
        
        encoder_raw[_encoder][AB]=value;
        
        cur_state = encoder_raw[_encoder][0] << 1 | encoder_raw[_encoder][1];

        if(prev_state[_encoder] < 0) prev_state[_encoder] = cur_state;        
        counter[_encoder] += stable[prev_state[_encoder] << 2 | cur_state];
        prev_state[_encoder]=cur_state;
}

// "callback stubs"
void callback_0_A_rise(void) { encoder_callback(0,0,true);}
void callback_0_A_fall(void) { encoder_callback(0,0,false);}
void callback_0_B_rise(void) { encoder_callback(0,1,true);}
void callback_0_B_fall(void) { encoder_callback(0,1,false);}

void callback_1_A_rise(void) { encoder_callback(1,0,true);}
void callback_1_A_fall(void) { encoder_callback(1,0,false);}
void callback_1_B_rise(void) { encoder_callback(1,1,true);}
void callback_1_B_fall(void) { encoder_callback(1,1,false);}

void callback_2_A_rise(void) { encoder_callback(2,0,true);}
void callback_2_A_fall(void) { encoder_callback(2,0,false);}
void callback_2_B_rise(void) { encoder_callback(2,1,true);}
void callback_2_B_fall(void) { encoder_callback(2,1,false);}
 
// Our 12*4=48 byte message (used both up and down)
struct woodenhaptics_message {
    float position_x;
    float position_y;
    float position_z;
    float command_force_x;
    float command_force_y;
    float command_force_z;
    float actual_current_0;
    float actual_current_1;
    float actual_current_2;
    float temperature_0;
    float temperature_1;
    float temperature_2;

    woodenhaptics_message():position_x(0),position_y(0),position_z(0),
                            command_force_x(0),command_force_y(0),command_force_z(0),
                            actual_current_0(0),actual_current_1(0),actual_current_2(0),
                            temperature_0(0),temperature_1(0),temperature_2(0){}
};

struct hid_to_pc_message { // 4*2 = 8 bytes
    short encoder_a;
    short encoder_b;
    short encoder_c;
    unsigned short debug;
};

struct pc_to_hid_message {  // 4*2 = 8 bytes
    short current_motor_a_mA;
    short current_motor_b_mA;
    short current_motor_c_mA;
    unsigned int debug;
};

void escon_timeout() {
    enableEscons = 0;
    myled2 = 0;
    
    myled3 = 0;
    myled4 = 0;
        
    for(int i=0;i<3;i++){
        pwm[i].write(0);  
        direction[i].write(0);
    }        
}
 
int main(void) {
    myled1 = 1;   // SETUP
    myled2 = 1;
    myled3 = 1;
    myled4 = 1;  
    
    wait_ms(500); 
    myled1 = 0;   // SETUP
    myled2 = 0;
    myled3 = 0;
    myled4 = 0;  

    
    encoder0_A.rise(&callback_0_A_rise);
    encoder0_A.fall(&callback_0_A_fall);
    encoder0_B.rise(&callback_0_B_rise);
    encoder0_B.fall(&callback_0_B_fall);
    
    encoder1_A.rise(&callback_1_A_rise);
    encoder1_A.fall(&callback_1_A_fall);
    encoder1_B.rise(&callback_1_B_rise);
    encoder1_B.fall(&callback_1_B_fall);

    encoder2_A.rise(&callback_2_A_rise);
    encoder2_A.fall(&callback_2_A_fall);
    encoder2_B.rise(&callback_2_B_rise);
    encoder2_B.fall(&callback_2_B_fall);
    
    enableEscons = 0;
    
    configuration config = default_woody();  
    
    for(int i=0;i<3;i++){
        pwm[i].period_us(500);
        pwm[i].write(0);  
        direction[i].period_us(500);
        direction[i].write(0);
        }      
    
    
    pc.baud(115200);
    
    send_report.length = 8; 
    //woodenhaptics_message msg;
    
    hid_to_pc_message hid_to_pc;
    hid_to_pc.debug = 0;
    pc_to_hid_message pc_to_hid;
    
//    Timer t;
//    t.start();
    
    Timer usb_timer;
    usb_timer.start();

    Timer debug_t;
    debug_t.start();
    
    Timer message_timeout;
    message_timeout.start();


    pc.printf("Hello World Debug!\n\r");
    
    
    while (1) {
        myled1 = 1;
        


        //try to read a msg
        if(hid.readNB(&recv_report)) {
            // TODO: Make sure we read the latest message, not the oldest.
            myled3 = !myled3; // We got data
            

            if(recv_report.length == 8){ // It should always be!                
            
            
                //message_timeout.reset();
                //enableEscons = 1;
                //myled2 = 1;
                msg_watchdog.detach();
                msg_watchdog.attach(&escon_timeout,0.5);
                enableEscons = 1;
                myled2 = 1;

            
                pc_to_hid = *reinterpret_cast<pc_to_hid_message*>(recv_report.data);
                
                //if(debug_t.read() > 0.5){
                //    pc.printf("Force: %f\n\r",pc_to_hid.current_motor_a_mA);
                //    debug_t.reset();
                //}
                
                //float f[3] = {msg.command_force_x, msg.command_force_y, msg.command_force_z};
                
                //float f[3] = {msg.command_force_x, msg.command_force_y, msg.command_force_z};
                float f[3] = {pc_to_hid.current_motor_a_mA*0.001, pc_to_hid.current_motor_b_mA*0.001,pc_to_hid.current_motor_c_mA*0.001};
                for(int i=0;i<3;i++){
                    int dir = f[i] > 0 ? 1 : 0;
                    direction[i].write(dir);
//                    if(i==1)
//                        pc.printf("Direction: %d (direction %d) \n\r", dir, direction[1].read());
                    float abs_val = std::abs(f[i]);
                    if(abs_val > 3.0)
                        pwm[i].write(0.9);
                    else
                        pwm[i].write(0.8*abs_val/3.0+0.1);
                }
            }
                
            myled1 = 1;
        }

        hid_to_pc.encoder_a = counter[0];
        hid_to_pc.encoder_b = counter[1];
        hid_to_pc.encoder_c = counter[2];
        
        if(usb_timer.read() > 0.00001) {
            pc.printf("%f\n",usb_timer.read());
            usb_timer.reset();

            hid_to_pc.debug++;
            unsigned char* out_buf = reinterpret_cast<unsigned char*>(&hid_to_pc);
    
            //Fill the report
            for (int i = 0; i < send_report.length; i++) {
                send_report.data[i] = out_buf[i];
            }
                
            //Send the report
            hid.send(&send_report);      
            myled4 = !myled4;
        }
    }
}