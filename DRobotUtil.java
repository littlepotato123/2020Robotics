package org.firstinspires.ftc.teamcode;

public class DRobotUtil {

    public static final String TAG= "DRobotUtil";
    public static final float mmPerInch        = 25.4f;
/*
    public static final double ROBOT_LENGTH = 12;
    public static final double ROBOT_WIDTH = 17;
    public static final double ROBOT_HEIGHT = 13*mmPerInch;
    public static final double ROBOT_WHEEL_RADIUS = 2*mmPerInch;
*/

//Camera on Robot
/*
    public static final double PHONE_CAMERA_REL_HEIGHT = 10*mmPerInch;
    public static final double PHONE_CAMERA_REL_WIDTH = 8.5*mmPerInch;
    public static final double PHONE_CAMERA_REL_LENGTH = 6*mmPerInch;
*/
    public static final float CAMERA_FORWARD_DISPLACEMENT  = 3.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    public static final float CAMERA_VERTICAL_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    public static final float CAMERA_LEFT_DISPLACEMENT     = -7*mmPerInch;     // eg: Camera is ON the robot's center line




    //FTC Field Spec

    public static final float FTC_FIELD_WIDTH  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
    public static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
//Robot Spec
    //static final double  COUNTS_PER_MOTOR_REV = 1440;
    static final double  COUNTS_PER_MOTOR_REV = 1120;
    static final double  COUNTS_PER_MOTOR_REV_LIFT = 1440;
    static final double  WHEELS_DIAMETER_INCHES = 4;
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEELS_DIAMETER_INCHES *3.14154);
    static final double COUNTS_PER_INCH_STRAFE = COUNTS_PER_MOTOR_REV / (WHEELS_DIAMETER_INCHES *3.14154);
    static final double DRIVE_SPEED =  0.8;
    static final double TURN_SPEED =  0.3;
    static final double LIFT_SPEED = 0.2;
    static final double ROBOT_WIDTH_INCHES = 17;
    static final double ROBOT_LENGTH_INCHES = 17.5;
    static final double ROBOT_HEIGHT_INCHES = 15;
    public static final double LIFT_MOTOR_WHEEL_RADIUS = 0.2;


    public static final double AUTONOMOUS_DRIVE_SPEED = 1.0;

    // Constant for Stone Target
    static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    public static final float bridgeZ = 6.42f * mmPerInch;
    public static final float bridgeY = 23 * mmPerInch;
    public static final float bridgeX = 5.18f * mmPerInch;
    public static final float bridgeRotY = 59;                                 // Units are degrees
    public static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField  = 36 * mmPerInch;

    static final String VUFORIA_KEY = "ASK5yBH/////AAABmTU1FcFdrUcoj+ogluo/eWoShGiH8QPFpaGBeLrauvdQuQHSqF4YZEiXCwqFG38x4zKjIztJg2dxwD5lLlnQIs54XbqL01Xp3vzcORMq7pdV6ydGBOsAQlga62JkWlMw4jax3Z23WPQcvUSYeAhUv4CYmNKDMyXlUW2Dx4R4Gvevay5dVVQKvcfBZFcwbPL5uSC7q8MXgvksbsJHLIa+DNvo2QseEwLd0NT9oYA0g6MbBmj8UGneV+z3FNVjrHYv1DJtnFypT/VUIidBiexruourO5x0Ulnah2Hu03PnyDVftuBs6f9NeKQfBmTkGvHFa9r0tPqntoI1ehvdjYmilB4tBveeXvm3fv+lBA2u2Fv0";




}
