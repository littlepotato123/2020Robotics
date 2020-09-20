package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




import org.firstinspires.ftc.teamcode.DRobotClass;


@TeleOp(name="Basic:DTeleOp", group="Linear Opmode")

public class DTeleOpClass extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //pass the hardwaremap to the RobotClass for initializing the Robot
    public DRobotClass myRobot = new DRobotClass();
    double powerMultiplier = 1;
    double placerservoposition = 0.3;
    double gripperservoposition = 0.4;
    double pullservoposition = 0.9;
    int liftmotorendocder =0;
    double capstoneservoposition = 0.2;



    @Override
    public void runOpMode() {

        //Initialize the robot
        myRobot.Initialize_TeleOp_Robot(hardwareMap);

        telemetry.addData("Initialize Robot", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();
        myRobot.setplacerServoPosition(placerservoposition );
        myRobot.setgripperServoPosition(gripperservoposition );
        myRobot.capstoneservo.setPosition(capstoneservoposition );
        myRobot.setpullServoPosition(pullservoposition);

        sleep(50);


        double leftFrontPower=0, rightFrontPower=0;
        double leftRearPower=0, rightRearPower=0;
        boolean rampup = true;
        //sliderarmposition =0.5;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Check Slider Arm first
            // Setup a variable for each drive wheel to save power level for telemetry
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            //double strafe = gamepad1.left_stick_x;
            double strafe1 = gamepad1.left_trigger;
            double strafe2 = gamepad1.right_trigger;
            double lift = gamepad2.left_stick_y;


            if (gamepad1.a)
            {
                myRobot.capstoneservo.setPosition(0.9);
                        sleep(50);
            }
            else
            if (gamepad1.b )
            {
                myRobot.capstoneservo.setPosition(0.2);
                sleep(50);

            }
            else
            if (gamepad2.x)
            {

                 placerservoposition = placerservoposition +0.01 ;
                 if (placerservoposition > 1) placerservoposition =1;
                 myRobot.setplacerServoPosition(placerservoposition);
                 //wait(100) ;
                sleep(10);
                 telemetry.addData("Placer Servo Position",myRobot.positionservo.getPosition()   );
                 telemetry.update();

                /*
                powerMultiplier = powerMultiplier + 0.1;
                if (powerMultiplier >= 1.0) powerMultiplier = 1;
                //SetPower_TeleOp_Robot(leftFrontPower ,rightFrontPower, leftRearPower,rightRearPower);
*/
            }
            else
            if (gamepad2.y)
            {
                placerservoposition = placerservoposition -0.01 ;
                if (placerservoposition < 0) placerservoposition =0;
                myRobot.setplacerServoPosition(placerservoposition);
                sleep(10);
                telemetry.addData("Placer Servo Position",myRobot.positionservo.getPosition()   );
                telemetry.update();


            }
            else
            if (gamepad2.left_trigger > 0.1)
            {

                pullservoposition = pullservoposition +0.1 ;
                if (pullservoposition > 0.9) pullservoposition =0.9;
                myRobot.setpullServoPosition(pullservoposition);
                //wait(100) ;
                sleep(10);
                telemetry.addData("Pull Servo Position left",myRobot.pullservoleft.getPosition()    );
                telemetry.addData("Pull Servo Position right",myRobot.pullservoright.getPosition()    );
                telemetry.update();

                /*
                powerMultiplier = powerMultiplier + 0.1;
                if (powerMultiplier >= 1.0) powerMultiplier = 1;
                //SetPower_TeleOp_Robot(leftFrontPower ,rightFrontPower, leftRearPower,rightRearPower);
*/
            }
            else
            if (gamepad2.right_trigger > 0.1 )
            {
                pullservoposition = pullservoposition -0.1 ;
                if (pullservoposition < 0) pullservoposition =0;
                myRobot.setpullServoPosition(pullservoposition);
                sleep(10);
                telemetry.addData("Pull Servo Position left",myRobot.pullservoleft.getPosition()    );
                telemetry.addData("Pull Servo Position right",myRobot.pullservoright.getPosition()    );
                telemetry.update();
            }
            else
            if (gamepad2.a)
            {

                gripperservoposition = gripperservoposition +0.1 ;
                if (gripperservoposition > 1.0) gripperservoposition =1.0;
                myRobot.setgripperServoPosition(gripperservoposition);

                //myRobot.opengripper();

                //wait(100) ;
                sleep(10);
                telemetry.addData("Gripper Servo Position",myRobot.gripperservo.getPosition()   );
                telemetry.update();

            }
            else
            if (gamepad2.b)
            {

                gripperservoposition = gripperservoposition -0.1 ;
                if (gripperservoposition < 0.0) gripperservoposition = 0;
                myRobot.setgripperServoPosition(gripperservoposition);

                //myRobot.closegripper();

                sleep(10);
                telemetry.addData("Gripper Servo Position",myRobot.gripperservo.getPosition()   );
                telemetry.update();


            }




                if ((strafe1 != 0) || (strafe2 != 0)) {
                    if (strafe1 != 0) {
                        leftFrontPower = strafe1 * powerMultiplier;
                        rightFrontPower = -strafe1 * powerMultiplier;
                        leftRearPower = -strafe1 * powerMultiplier;
                        rightRearPower = strafe1 * powerMultiplier;
                    } else {

                        leftFrontPower = -strafe2 * powerMultiplier;
                        rightFrontPower = strafe2 * powerMultiplier;
                        leftRearPower = strafe2 * powerMultiplier;
                        rightRearPower = -strafe2 * powerMultiplier;

                    }
                }
                else
                {
                    leftFrontPower = (Range.clip(drive - turn, -1.0, 1.0)) * powerMultiplier;
                    rightFrontPower = (Range.clip(drive + turn, -1.0, 1.0)) * powerMultiplier;
                    leftRearPower = leftFrontPower;
                    rightRearPower = rightFrontPower;
                }



                /*
                if ((strafe != 0) && (turn != 0))
                {
                    leftFrontPower = -strafe*powerMultiplier;
                    rightFrontPower = strafe*powerMultiplier;
                    leftRearPower = strafe*powerMultiplier;
                    rightRearPower = -strafe*powerMultiplier;
                }
                else
                    {
                    leftFrontPower = (Range.clip(drive - turn, -1.0, 1.0))*powerMultiplier;
                    rightFrontPower = (Range.clip(drive + turn, -1.0, 1.0))*powerMultiplier ;
                    leftRearPower = leftFrontPower;
                    rightRearPower = rightFrontPower;
                }
                */



                // Send calculated power to wheels
                myRobot.SetPower_TeleOp_Robot(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
                myRobot.liftmotor.setPower(lift*0.5);
                // Show the elapsed game time and wheel power

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Lift Motor","Pos"+ myRobot.liftmotor.getCurrentPosition());
                telemetry.addData("Servos","Gripper Pos (%.1f), Place Pos (%.1f)",myRobot.gripperservo.getPosition(), myRobot.positionservo.getPosition());
                telemetry.addData("Motors", "LF (%.2f), RF (%.2f), LR (%.2f), RR (%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);

                telemetry.update();

        }
    }
}


