const int CHASSIS_STANDARD_SLEW_RATE = 10;
const int FAST_SLEW_RATE             = 50;
const int STANDARD_SLEW_RATE         = 10;

// constants that describe the position and size of the tracking wheels - used for odometry
const double LEFT_TRACKING_WHEEL_CONVERSION  = 1.0000; // To convert radians into in
const double RIGHT_TRACKING_WHEEL_CONVERSION = 1.0000; // To convert radians into in
const double BACK_TRACKING_WHEEL_CONVERSION  = 1.0000; // To convert radians into in
const double ROBOT_WIDTH                     = 1.0000; // To convert radians into in
const double BACK_TRACKING_OFFSET            = 1.0000; // in

const double auton_chassis_straight_P         =   7.00;
const double auton_chassis_straight_I         =   0.00;
const double auton_chassis_straight_I_boundry =      6;
const double auton_chassis_straight_D         =  300.0;
const double auton_chassis_straight_P2        =   4.00;

const double master_puncherResetDeg = 598.3;

const double auton_chassis_turn_P             =                   0.80;
const double auton_chassis_turn_I             =                   0.00;
const double auton_chassis_turn_I_boundry     =                   30.0;
const double auton_chassis_turn_D             =                   55.0;
const double auton_chassis_turn_multiplier    =                12.2319;
const double auton_chassis_straight_tiles     =                16.9492;
const double auton_puncherResetDeg            = master_puncherResetDeg;
const double auton_chassis_park_center_time   =                   4000;
const double auton_chassis_park_alliance_time =                   2500;

const double usercontrol_chassis_fastMultiplier = 1.0;
const double usercontrol_chassis_slowMultiplier = 0.4;
const double usercontrol_puncherResetDeg = master_puncherResetDeg;