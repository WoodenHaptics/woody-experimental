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


configuration default_woody(){
    double data[] = { 0.010, 0.010, 0.010, 
                      0.080, 0.205, 0.200, 
                      0.160, 0.120, 0.120,
                      0.220, 0.000, 0.080, 0.100, 
                      0.0603, 0.0603, 0.0603, 3.0, 4096, 2000, 2000, 
                      12.0, 5000.0, 8.0,
                      0.170, 0.110, 0.051, 0.091, 9.81};
    return configuration(data); 
}

//==============================================================================
// Helper functions for getPosition & setForce
//==============================================================================
double getMotorAngle(int motor, const double cpr, const int encoderValue) {
    const double pi = 3.14159265359;
    return 2.0*pi*encoderValue/cpr;
}
/*
void setVolt(double v, int motor){
    if(v > 10 || v< -10) { printf("Volt outside +/- 10 Volt\n"); return; }

    // -10V to +10V is mapped from 0x0000 to 0xFFFF
    unsigned int signal = (v+10.0)/20.0 * 0xFFFF;
    S826_DacDataWrite(0,motor,signal,0);
}
*/

struct pose {
    double Ln;
    double Lb;
    double Lc;
    double tA;  // angle of body A (theta_A)
    double tB;  // angle of body B (theta_B)
    double tC;  // angle of body C (theta_C)
};

pose calculate_pose(const configuration& c, int* encoderValues) {
    pose p;

    double cpr[] = { c.cpr_encoder_a, c.cpr_encoder_b, c.cpr_encoder_c };
    double gearRatio[] = { -c.diameter_body_a / c.diameter_capstan_a,
                   -c.diameter_body_b / c.diameter_capstan_b,
                    c.diameter_body_c / c.diameter_capstan_c };

    double dofAngle[3];
    for(int i=0;i<3;i++)
        dofAngle[i] = getMotorAngle(i,cpr[i],encoderValues[i]) / gearRatio[i];

    // Calculate dof angles (theta) for each body
    p.Ln = c.length_body_a; 
    p.Lb = c.length_body_b; 
    p.Lc = c.length_body_c; 
    p.tA = dofAngle[0];
    p.tB = dofAngle[1];
    p.tC = dofAngle[2] - dofAngle[1];

    return p;
}

struct vec {
    double x;
    double y;
    double z;
    
    vec(double x, double y, double z):x(x),y(y),z(z) {}
};

vec getPosition(const configuration& m_config, int* encoder_values){
    double x,y,z;

    const pose p = calculate_pose(m_config, encoder_values);
    const double& Ln = p.Ln;
    const double& Lb = p.Lb; 
    const double& Lc = p.Lc; 
    const double& tA = p.tA; 
    const double& tB = p.tB; 
    const double& tC = p.tC; 

    // Do forward kinematics (thetas -> xyz)
    x = cos(tA)*(Lb*sin(tB)+Lc*cos(tB+tC))     - m_config.workspace_origin_x;
    y = sin(tA)*(Lb*sin(tB)+Lc*cos(tB+tC))     - m_config.workspace_origin_y;
    z = Ln + Lb*cos(tB) - Lc*sin(tB+tC)        - m_config.workspace_origin_z;
    
    return vec(x,y,z);
}