
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;
import com.qualcomm.robotcore.hardware.*;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices;
import org.firstinspires.ftc.teamcode.DRobotVisionAnalyze;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.DRobotUtil.AUTONOMOUS_DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.DRobotUtil.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.DRobotUtil.COUNTS_PER_INCH_STRAFE;
import static org.firstinspires.ftc.teamcode.DRobotUtil.COUNTS_PER_MOTOR_REV;
import static org.firstinspires.ftc.teamcode.DRobotUtil.COUNTS_PER_MOTOR_REV_LIFT;
import static org.firstinspires.ftc.teamcode.DRobotUtil.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.DRobotUtil.LIFT_MOTOR_WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.DRobotUtil.LIFT_SPEED;
import static org.firstinspires.ftc.teamcode.DRobotUtil.ROBOT_WIDTH_INCHES;
import static org.firstinspires.ftc.teamcode.DRobotUtil.TURN_SPEED;

import org.firstinspires.ftc.teamcode.DRobotUtil;
/*
import static org.firstinspires.ftc.teamcode.DRobotUtil.AUTONOMOUS_DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.DRobotUtil.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.DRobotUtil.COUNTS_PER_MOTOR_REV;
import static org.firstinspires.ftc.teamcode.DRobotUtil.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.DRobotUtil.LIFT_MOTOR_WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.DRobotUtil.LIFT_SPEED;
import static org.firstinspires.ftc.teamcode.DRobotUtil.ROBOT_WIDTH_INCHES;
import static org.firstinspires.ftc.teamcode.DRobotUtil.mmPerInch;
*/

public class DRobotClass {

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;
    public DcMotor liftmotor = null;
   // public DcMotor motor40=null;
   // public DcMotor pullmotor = null;


    public double leftFrontPower = 0;
    public double rightFrontPower = 0;
    public double leftRearPower = 0;
    public double rightRearPower = 0;




    public Servo positionservo;
    public Servo gripperservo;
    public Servo capstoneservo;
    public Servo pullservoleft;
    public Servo pullservoright;

/*
    static final double  COUNTS_PER_MOTOR_REV = 1440;
    static final double  WHEELS_DIAMETER_INCHES = 4;
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEELS_DIAMETER_INCHES *3.14154);
    static final double DRIVE_SPEED =  0.4;
    static final double TURN_SPEED =  0.4;
    static final double LIFT_SPEED = 0.2;
    static final double ROBOT_WIDTH_INCHES = 17;
    static final double ROBOT_LENGTH_INCHES = 17.5;
    static final double ROBOT_HEIGHT_INCHES = 15;
*/

    public Position robotCurrentPos;

    public Position getCurrentRobotPosition()
    {
        return robotCurrentPos;
    }

    public void setCurrentRobotPosition(Position pos )
    {
         robotCurrentPos = pos;
    }

    public void calculateRobotNewDisplacement(Position pos)
    {

        //Calculate new robot position and then set the new position
        setCurrentRobotPosition(pos);
   }


    public void Initialize_TeleOp_Robot(HardwareMap hardwareMap)
    {

        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront  = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        liftmotor = hardwareMap.get(DcMotor.class, "liftmotor");
        //pullmotor = hardwareMap.get(DcMotor.class, "pullmotor");

        positionservo = hardwareMap.get(Servo.class,"positionservo" );
        gripperservo = hardwareMap.get(Servo.class,"gripperservo" );
        pullservoleft = hardwareMap.get(Servo.class,"pullservoleft" );
        pullservoright = hardwareMap.get(Servo.class,"pullservoright" );
        capstoneservo = hardwareMap.get(Servo.class,"capstoneservo" );

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);



        liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SetPower_TeleOp_Robot(leftFrontPower ,rightFrontPower,leftRearPower,rightRearPower);



    }


    public void Initialize_Autonomous_Robot(HardwareMap hardwareMap)
    {


        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront  = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        liftmotor = hardwareMap.get(DcMotor.class, "liftmotor");
        //motor40 = hardwareMap.get(DcMotor.class, "motor40");
        //pullmotor = hardwareMap.get(DcMotor.class, "pullmotor");

        gripperservo = hardwareMap.get(Servo.class,"gripperservo" );
        positionservo = hardwareMap.get(Servo.class,"positionservo" );
        pullservoleft = hardwareMap.get(Servo.class,"pullservoleft" );
        pullservoright = hardwareMap.get(Servo.class,"pullservoright" );
        capstoneservo = hardwareMap.get(Servo.class,"capstoneservo" );




        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor40.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //pullmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

/*
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pullmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

*/


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        liftmotor.setDirection(DcMotor.Direction.FORWARD);
        //motor40.setDirection(DcMotorSimple.Direction.FORWARD);
        //pullmotor.setDirection(DcMotor.Direction.FORWARD);

}



/*
    public void positionstraight()
    {
        positionservo.setPosition(0.6);
    }

    public void positionright()
    {
        positionservo.setPosition(0.1);
    }

    public void opengripper()
    {
        gripperservo.setPosition(1);
    }

    public void closegripper()
    {
        gripperservo.setPosition(0.45);
    }

    public void resetStartingposition()
    {
        gripperservo.setPosition(0);
        positionservo.setPosition(0);
        lift_Move(0.2);

    }
*/
    public double getgripperServoPosition()
    {
        return (double)(gripperservo.getPosition()  );
    }

    public void setgripperServoPosition(double pos)
    {
        gripperservo.setPosition(pos);
    }



    public void setpullServoPosition(double pos)
    {
        pullservoleft.setPosition(1-pos-0.1);
        pullservoright.setPosition(pos);

    }

    public void setplacerServoPosition(double pos)
    {
        positionservo.setPosition(pos);
    }

    public double getplacerServoPosition()
    {
        return (double)(positionservo.getPosition()  );
    }

    public double getpullServoPosition()
    {
        return (double)(pullservoleft.getPosition()  );
    }



    public void  SetPower_TeleOp_Robot(double leftFrontPower, double rightFrontPower , double leftRearPower,double rightRearPower )
    {
          leftFront.setPower(leftFrontPower  );
          rightFront.setPower(rightFrontPower );
          leftRear.setPower(leftRearPower );
          rightRear.setPower(rightRearPower );

    }

    public void  SetPower_AutonomousOp_Robot(double leftFrontPower, double rightFrontPower , double leftRearPower,double rightRearPower )
    {
        leftFront.setPower(leftFrontPower  );
        rightFront.setPower(rightFrontPower );
        leftRear.setPower(leftRearPower );
        rightRear.setPower(rightRearPower );

    }


    public void stop_TeleOp_Robot()
    {
        SetPower_TeleOp_Robot(0,0,0,0);
    }



    public void lift_Move(double height)
    {
        int currentencoder=0, newencoderposition=0;


        currentencoder = liftmotor.getCurrentPosition();


        double counts_perinch = COUNTS_PER_MOTOR_REV_LIFT/(2*3.14159*LIFT_MOTOR_WHEEL_RADIUS);
        newencoderposition = currentencoder + (int) (counts_perinch *height) ;


        liftmotor.setTargetPosition(newencoderposition );
        liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        liftmotor.setPower(LIFT_SPEED );


        while (liftmotor.isBusy() )
        {
            //hold on guys we are moving
        }


    }

    /*
    public void lift_Move_UP(double height)
    {
        int currentencoder=0, newencoderposition=0;

        currentencoder = liftmotor.getCurrentPosition();


        double counts_perinch = COUNTS_PER_MOTOR_REV/(2*3.14159*LIFT_MOTOR_WHEEL_RADIUS);
        newencoderposition = currentencoder + (int) (counts_perinch *height) ;

        //Range.clip( newencoderposition , , (int)(1440*3*2*3.14159*LIFT_MOTOR_WHEEL_RADIUS) ) ;



        liftmotor.setTargetPosition(newencoderposition );
        liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        liftmotor.setPower(LIFT_SPEED );
        liftmotor.setDirection(DcMotor.Direction.REVERSE);

        while (liftmotor.isBusy() )
        {
            //hold on guys we are moving
        }


    }

    public void lift_Move_DOWN(double height)
    {
        int currentencoder=0, newencoderposition=0;

        currentencoder = liftmotor.getCurrentPosition();


        double counts_perinch = COUNTS_PER_MOTOR_REV/(2*3.14159*LIFT_MOTOR_WHEEL_RADIUS);
        newencoderposition = currentencoder + (int) (counts_perinch *height) ;

        //Range.clip( newencoderposition , , (int)(1440*3*2*3.14159*LIFT_MOTOR_WHEEL_RADIUS) ) ;



        liftmotor.setTargetPosition(newencoderposition );
        liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        liftmotor.setPower(LIFT_SPEED );
        liftmotor.setDirection(DcMotor.Direction.FORWARD);

        while (liftmotor.isBusy() )
        {
            //hold on guys we are moving
        }


    }
*/



    public void moveLeft_Autonomous_Robot(double distance)
    {
        moveLeft_Autonomous_Robot(distance,AUTONOMOUS_DRIVE_SPEED);

    }


    public void moveLeft_Autonomous_Robot(double distance, double motorspeed)
    {

        Position pos=new Position();
        calculateRobotNewDisplacement(pos);

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        newLeftFrontTarget = this.leftFront.getCurrentPosition() - (int)(distance*COUNTS_PER_INCH_STRAFE  );
        newRightFrontTarget = this.rightFront.getCurrentPosition() + (int)(distance*COUNTS_PER_INCH_STRAFE );
        newLeftRearTarget= this.leftRear.getCurrentPosition() + (int )(distance*COUNTS_PER_INCH_STRAFE );
        newRightRearTarget= this.rightRear.getCurrentPosition() - (int) (distance*COUNTS_PER_INCH_STRAFE );

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);


        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION );

        leftFront.setPower(motorspeed );
        rightFront.setPower(motorspeed );
        leftRear.setPower(motorspeed );
        rightRear.setPower(motorspeed);

        while ((leftFront.isBusy() ) || (rightFront.isBusy() )||(leftRear.isBusy() )||(rightRear.isBusy() ))
        {
            //Hold On
        }

        //stop All Motion
        /*
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */


    }

    public void moveRight_Autonomous_Robot(double distance)
    {
        moveRight_Autonomous_Robot(distance,AUTONOMOUS_DRIVE_SPEED);

    }



    public void moveRight_Autonomous_Robot(double distance, double motorspeed )
    {

        Position pos=new Position();
        calculateRobotNewDisplacement(pos);
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;
        int errorcorrect = 0;

        newLeftFrontTarget = this.leftFront.getCurrentPosition() + (int)(distance*COUNTS_PER_INCH_STRAFE );
        newRightFrontTarget = this.rightFront.getCurrentPosition() - (int)(distance*COUNTS_PER_INCH_STRAFE )-errorcorrect ;
        newLeftRearTarget= this.leftRear.getCurrentPosition() - (int )(distance*COUNTS_PER_INCH_STRAFE ) ;
        newRightRearTarget= this.rightRear.getCurrentPosition() + (int) (distance*COUNTS_PER_INCH_STRAFE ) + errorcorrect ;

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);


        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION );

        leftFront.setPower(motorspeed );
        rightRear.setPower(motorspeed);
        rightFront.setPower(motorspeed );
        leftRear.setPower(motorspeed);




        while ((leftFront.isBusy() ) || (rightFront.isBusy() )||(leftRear.isBusy() )||(rightRear.isBusy() ))
        {
            //Hold On
        }

        //stop All Motion
        /*
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */


    }


    /*
    public void moveReverse_Autonomous_Robot(double distance)
    {

        Position pos=new Position();
        calculateRobotNewDisplacement(pos);

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        newLeftFrontTarget = this.leftFront.getCurrentPosition() + (int)(distance*COUNTS_PER_INCH );
        newRightFrontTarget = this.rightFront.getCurrentPosition() + (int)(distance*COUNTS_PER_INCH );
        newLeftRearTarget= this.leftRear.getCurrentPosition() + (int )(distance*COUNTS_PER_INCH );
        newRightRearTarget= this.rightRear.getCurrentPosition() + (int) (distance*COUNTS_PER_INCH );

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);


        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION );

        leftFront.setPower(AUTONOMOUS_DRIVE_SPEED);
        rightFront.setPower(AUTONOMOUS_DRIVE_SPEED);
        leftRear.setPower(AUTONOMOUS_DRIVE_SPEED);
        rightRear.setPower(AUTONOMOUS_DRIVE_SPEED);

        while ((leftFront.isBusy() ) || (rightFront.isBusy() )||(leftRear.isBusy() )||(rightRear.isBusy() ))
        {
            //Hold On
        }
    }
    */


    public void moveForward_Autonomous_Robot(double distance)
    {
        moveForward_Autonomous_Robot(distance, AUTONOMOUS_DRIVE_SPEED );
    }

    public void moveForward_Autonomous_Robot(double distance, double motorspeed)
    {

        Position pos=new Position();

        calculateRobotNewDisplacement(pos);

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        newLeftFrontTarget = this.leftFront.getCurrentPosition() - (int)(distance*COUNTS_PER_INCH );
        newRightFrontTarget = this.rightFront.getCurrentPosition() - (int)(distance*COUNTS_PER_INCH );
        newLeftRearTarget= this.leftRear.getCurrentPosition() - (int )(distance*COUNTS_PER_INCH );
        newRightRearTarget= this.rightRear.getCurrentPosition() - (int) (distance*COUNTS_PER_INCH );

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);


        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION );

        leftFront.setPower(motorspeed );
        rightRear.setPower(motorspeed );
        rightFront.setPower(motorspeed );
        leftRear.setPower(motorspeed);



        while ((leftFront.isBusy() ) || (rightFront.isBusy() )||(leftRear.isBusy() )||(rightRear.isBusy() ))
        {
            //Hold On
        }



    }

    public void turnClock_Autonomous_Robot(double angleDegrees)
    {
        turnClock_Autonomous_Robot(angleDegrees,TURN_SPEED );
    }


    public void turnClock_Autonomous_Robot(double angleDegrees, double motorspeed)
    {
        Position pos=new Position();
        calculateRobotNewDisplacement(pos);
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        newLeftFrontTarget = this.leftFront.getCurrentPosition() + (int) ((3.14159*ROBOT_WIDTH_INCHES*1.44*angleDegrees/360)*COUNTS_PER_INCH);
        newRightFrontTarget = this.rightFront.getCurrentPosition() - (int)((3.14159*ROBOT_WIDTH_INCHES*1.44*angleDegrees/360)*COUNTS_PER_INCH);
        newLeftRearTarget= this.leftRear.getCurrentPosition() + (int )((3.14159*ROBOT_WIDTH_INCHES*1.44*angleDegrees/360)*COUNTS_PER_INCH);
        newRightRearTarget= this.rightRear.getCurrentPosition() - (int) ((3.14159*ROBOT_WIDTH_INCHES*1.44*angleDegrees/360)*COUNTS_PER_INCH);

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);


        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION );
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION );

        leftFront.setPower(motorspeed );
        rightFront.setPower(motorspeed );
        leftRear.setPower(motorspeed );
        rightRear.setPower(motorspeed );

        while ((leftFront.isBusy() ) || (rightFront.isBusy() )||(leftRear.isBusy() )||(rightRear.isBusy() ))
        {
            //Hold On
        }



    }

    public void moveReverse_Autonomous_Robot(int i) {
    }
}
