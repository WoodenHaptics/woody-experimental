//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2014, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Your name, institution, or company name.
    \version   3.0.0 $Rev: 1242 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CWoodenDeviceH
#define CWoodenDeviceH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_WOODEN_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
#include "hidapi.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CWoodenDevice.h

    \brief
    <b> Devices </b> \n 
    Custom Haptic Device (Template).
*/
//==============================================================================

//------------------------------------------------------------------------------
class cWoodenDevice;
typedef std::shared_ptr<cWoodenDevice> cWoodenDevicePtr;


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

//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cWoodenDevice
    \ingroup    devices  

    \brief
    Interface to custom haptic devices (template).

    \details
    cWoodenDevice provides a basic template which allows to very easily
    interface CHAI3D to your own custom haptic device. \n\n

    Simply follow the 11 commented step in file CWoodenDevice.cpp 
    and complete the code accordingly.
    Depending of the numbers of degrees of freedom of your device, not
    all methods need to be implemented. For instance, if your device
    does not provide any rotation degrees of freedom, simply ignore
    the getRotation() method. Default values will be returned correctly
    if these are not implemented on your device. In the case of rotations
    for instance, the identity matrix is returned.\n\n

    You may also rename this class in which case you will also want to
    customize the haptic device handler to automatically detect your device.
    Please consult method update() of the cHapticDeviceHandler class
    that is located in file CHapticDeviceHandler.cpp .
    Simply see how the haptic device handler already looks for
    device of type cWoodenDevice.\n\n

    If you are encountering any problems with your implementation, check 
    for instance file cDeltaDevices.cpp which implement supports for the 
    Force Dimension series of haptic devices. In order to verify the implementation
    use the 01-device example to get started. Example 11-effects is a great
    demo to verify how basic haptic effects may behave with you haptic
    devices. If you do encounter vibrations or instabilities, try reducing
    the maximum stiffness and/or damping values supported by your device. 
    (see STEP-1 in file CWoodenDevice.cpp).\n
    
    Make  sure that your device is also communicating fast enough with 
    your computer. Ideally the communication period should take less 
    than 1 millisecond in order to reach a desired update rate of at least 1000Hz.
    Problems can typically occur when using a slow serial port (RS232) for
    instance.\n
*/
//==============================================================================
class cWoodenDevice : public cGenericHapticDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cWoodenDevice.
    cWoodenDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cWoodenDevice.
    virtual ~cWoodenDevice();

    //! Shared cWoodenDevice allocator.
    static cWoodenDevicePtr create(unsigned int a_deviceNumber = 0) { return (std::make_shared<cWoodenDevice>(a_deviceNumber)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! Open connection to haptic device.
    virtual bool open();

    //! Close connection to haptic device.
    virtual bool close();

    //! Calibrate haptic device.
    virtual bool calibrate(bool a_forceCalibration = false);

    //! Read the position of the device. Units are meters [m].
    virtual bool getPosition(cVector3d& a_position);

    //! Read the orientation frame of the device handle.
    virtual bool getRotation(cMatrix3d& a_rotation);

    //! Read the gripper angle in radian [rad].
    virtual bool getGripperAngleRad(double& a_angle);

    //! Send a force [N] and a torque [N*m] and gripper force [N] to haptic device.
    virtual bool setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce);

    //! Read status of the switch [__true__ = __ON__ / __false__ = __OFF__].
    virtual bool getUserSwitch(int a_switchIndex, bool& a_status);


    //! Public methods to read special info from the wooden device (for developers)
    cVector3d getTorqueSignals() { return torqueSignals; }
    cVector3d getEncoders() { return cVector3d(incoming_msg.temperature_0,
                                               incoming_msg.temperature_1,
                                               incoming_msg.temperature_2);}


    //--------------------------------------------------------------------------
    // PUBLIC STATIC METHODS:
    //--------------------------------------------------------------------------

public: 

    //! Returns the number of devices available from this class of device.
    static unsigned int getNumDevices();


    //! A collection of variables that can be set in ~/wooden_haptics.json 
    struct configuration {
        double diameter_capstan_a;      // m
        double diameter_capstan_b;      // m
        double diameter_capstan_c;      // m
        double length_body_a;           // m
        double length_body_b;           // m
        double length_body_c;           // m
        double diameter_body_a;         // m
        double diameter_body_b;         // m
        double diameter_body_c;         // m
        double workspace_origin_x;      // m
        double workspace_origin_y;      // m
        double workspace_origin_z;      // m
        double workspace_radius;        // m (for application information)
        double torque_constant_motor_a; // Nm/A
        double torque_constant_motor_b; // Nm/A
        double torque_constant_motor_c; // Nm/A
        double current_for_10_v_signal; // A
        double cpr_encoder_a;           // quadrupled counts per revolution
        double cpr_encoder_b;           // quadrupled counts per revolution
        double cpr_encoder_c;           // quadrupled counts per revolution
        double max_linear_force;        // N
        double max_linear_stiffness;    // N/m
        double max_linear_damping;      // N/(m/s)
        double mass_body_b;             // Kg
        double mass_body_c;             // Kg
        double length_cm_body_b;        // m     distance to center of mass  
        double length_cm_body_c;        // m     from previous body
        double g_constant;              // m/s^2 usually 9.81 or 0 to 
                                              //       disable gravity compensation

        // Set values
        configuration(const double* k):
          diameter_capstan_a(k[0]), diameter_capstan_b(k[1]), diameter_capstan_c(k[2]),
          length_body_a(k[3]), length_body_b(k[4]), length_body_c(k[5]),
          diameter_body_a(k[6]), diameter_body_b(k[7]), diameter_body_c(k[8]), 
          workspace_origin_x(k[9]), workspace_origin_y(k[10]), workspace_origin_z(k[11]), 
          workspace_radius(k[12]), torque_constant_motor_a(k[13]), 
          torque_constant_motor_b(k[14]), torque_constant_motor_c(k[15]), 
          current_for_10_v_signal(k[16]), cpr_encoder_a(k[17]), cpr_encoder_b(k[18]), 
          cpr_encoder_c(k[19]), max_linear_force(k[20]), max_linear_stiffness(k[21]), 
          max_linear_damping(k[22]), mass_body_b(k[23]), mass_body_c(k[24]), 
          length_cm_body_b(k[25]), length_cm_body_c(k[26]), g_constant(k[27]){}
    };


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////
    /*
        INTERNAL VARIABLES:

        If you need to declare any local variables or methods for your device,
        you may do it here bellow. 
    */
    ////////////////////////////////////////////////////////////////////////////

    int lost_messages;

protected:

    const configuration m_config;

    cVector3d torqueSignals;

    woodenhaptics_message incoming_msg;
    woodenhaptics_message outgoing_msg;

    hid_to_pc_message hid_to_pc;
    pc_to_hid_message pc_to_hid;



    int res;
    unsigned char buf[9];// 1 extra byte for the report ID
    #define MAX_STR 255
    wchar_t wstr[MAX_STR];
    hid_device *handle;
    int i;
    struct hid_device_info *devs, *cur_dev;
};

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_WOODEN_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
