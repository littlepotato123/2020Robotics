/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.DRobotUtil.CAMERA_FORWARD_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.DRobotUtil.CAMERA_LEFT_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.DRobotUtil.CAMERA_VERTICAL_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.DRobotUtil.VUFORIA_KEY;
import static org.firstinspires.ftc.teamcode.DRobotUtil.bridgeRotY;
import static org.firstinspires.ftc.teamcode.DRobotUtil.bridgeRotZ;
import static org.firstinspires.ftc.teamcode.DRobotUtil.bridgeX;
import static org.firstinspires.ftc.teamcode.DRobotUtil.bridgeY;
import static org.firstinspires.ftc.teamcode.DRobotUtil.bridgeZ;
import static org.firstinspires.ftc.teamcode.DRobotUtil.halfField;
import static org.firstinspires.ftc.teamcode.DRobotUtil.mmPerInch;
import static org.firstinspires.ftc.teamcode.DRobotUtil.mmTargetHeight;
import static org.firstinspires.ftc.teamcode.DRobotUtil.quadField;
import static org.firstinspires.ftc.teamcode.DRobotUtil.stoneZ;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


@Autonomous(name="Red Auto Skystone 1", group ="Concept")

public class DAutonomousRedSkystoneJustpark extends LinearOpMode {

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    //private static final String VUFORIA_KEY = "ASK5yBH/////AAABmTU1FcFdrUcoj+ogluo/eWoShGiH8QPFpaGBeLrauvdQuQHSqF4YZEiXCwqFG38x4zKjIztJg2dxwD5lLlnQIs54XbqL01Xp3vzcORMq7pdV6ydGBOsAQlga62JkWlMw4jax3Z23WPQcvUSYeAhUv4CYmNKDMyXlUW2Dx4R4Gvevay5dVVQKvcfBZFcwbPL5uSC7q8MXgvksbsJHLIa+DNvo2QseEwLd0NT9oYA0g6MbBmj8UGneV+z3FNVjrHYv1DJtnFypT/VUIidBiexruourO5x0Ulnah2Hu03PnyDVftuBs6f9NeKQfBmTkGvHFa9r0tPqntoI1ehvdjYmilB4tBveeXvm3fv+lBA2u2Fv0";


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here


    // Class Members
    private OpenGLMatrix lastLocation = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    public DRobotClass myRobot = new DRobotClass();

    //Vuforia Camera
    int cameraMonitorViewId;
    public VuforiaLocalizer vuforia;
    public VuforiaTrackables targetsSkyStone;
    public List<VuforiaTrackable> allTrackables;
    public VuforiaLocalizer.Parameters parameters;
    public VuforiaTrackable stoneTarget, blueRearBridge, redRearBridge, redFrontBridge, blueFrontBridge, red1, red2, blue1, blue2, front1, front2, rear1, rear2;

    public void Initialize_Autonomous() {
        //Initialize Robot
        myRobot.Initialize_Autonomous_Robot(hardwareMap);

        //Initialize Vuforia
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        // Instantiate Vuforia Engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        CameraDevice.getInstance().setFlashTorchMode(true);


        //Load OpenCV
        /*
        if (!OpenCVLoader.initDebug()) {
            //Log.d("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            //RobotLog.ee(TAG, e, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            telemetry.addData("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            telemetry.update();
        } else {
            telemetry.addData("OpenCV", "Loaded");
            telemetry.update();
        }
        */

    }


    void Initialize_Vuforia_Trackables() {
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Aldliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(+bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:


    }


    @Override
    public void runOpMode() {

        Position pos = new Position();

        VectorF translation = null;
        Initialize_Autonomous();
        Initialize_Vuforia_Trackables();
        targetsSkyStone.activate();
        pos.x = 0;
        pos.y = 0;
        pos.z = 0;


       // myRobot.moveForward_Autonomous_Robot()




        waitForStart();

        myRobot.setpullServoPosition(0.9);
        sleep(50);
        myRobot.positionservo.setPosition(0.3) ;
        sleep(50);
        myRobot.gripperservo.setPosition(0.9);
        sleep(100);
        myRobot.capstoneservo.setPosition(0.2);
        sleep(50);

        myRobot.moveForward_Autonomous_Robot(24,0.5 );
        pos.x+=24;
        sleep(500);



        targetVisible = false;
        while ((!targetVisible )&& (pos.y >= -17)) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                    if (trackable.getName() == "Stone Target") {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }
            }

            if (targetVisible) {
                // express position (translation) of robot in inches.
                translation = lastLocation.getTranslation();
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            } else {
                telemetry.addData(" Target not visible ", "none");
                myRobot.moveLeft_Autonomous_Robot(-8,0.3);
                pos.y=pos.y-8;
                sleep(500);
            }
            //telemetry.update();
        }

        if (targetVisible )
        {

            int y_displacement = (int)(translation.get(1)/mmPerInch);
            RobotLog.ii("Target Visible  ", "%.1f %.1f %.1f", translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            //Position properly
            myRobot.moveLeft_Autonomous_Robot(y_displacement  ,0.5);
            pos.y = pos.y-y_displacement ;

            //Move and pickup
            myRobot.moveForward_Autonomous_Robot(9,0.5);
            pos.x+=9;

            //Close gripper
            myRobot.setgripperServoPosition(0);
            sleep(500);

            //After grabbing move back
            myRobot.moveForward_Autonomous_Robot(-5,0.5);
            pos.x-=5;

            //Turn Clockwise 90 degrees towards the foundation side
            myRobot.turnClock_Autonomous_Robot(-83,0.9);

            RobotLog.ii("Target distance  ", "%.1f",(39-pos.y) );
            myRobot.moveForward_Autonomous_Robot((39 - pos.y ), 1);
            pos.y = pos.y +39-pos.y;

            //myRobot.lift_Move(-0.6);

            //myRobot.moveForward_Autonomous_Robot((13),0.4);

            //pos.y=pos.y+13;



            //myRobot.lift_Move(0.1);

            //Open Gripper
            myRobot.setgripperServoPosition(1);
            sleep(500);


            // myRobot.moveRight_Autonomous_Robot(17,0.5);

            //myRobot.moveForward_Autonomous_Robot(-15,1);
            //myRobot.lift_Move(0.5);

            myRobot.moveForward_Autonomous_Robot(-16,1);

            myRobot.setgripperServoPosition(0);
            sleep(500);

        }
        else
        {

            myRobot.moveLeft_Autonomous_Robot(-4,0.5);
            pos.y=pos.y-(-4);

            //Move and pickup
            myRobot.moveForward_Autonomous_Robot(7,0.5);
            pos.x+=7;

            //Close gripper
            myRobot.setgripperServoPosition(0);
            sleep(500);

            //After grabbing move back
            myRobot.moveForward_Autonomous_Robot(-6,0.5);
            pos.x-=6;

            //Turn Clockwise 90 degrees towards the foundation side
            myRobot.turnClock_Autonomous_Robot(-89,0.9);



            myRobot.moveForward_Autonomous_Robot((48 - pos.y ), 1);
            pos.y = pos.y +48-pos.y;

            //myRobot.lift_Move(-0.6);

            //myRobot.moveForward_Autonomous_Robot((13    ),0.4);

            //pos.y=pos.y+13;

            //myRobot.lift_Move(0.2);

            //Open Gripper
            myRobot.setgripperServoPosition(1);
            sleep(500);


            // myRobot.moveRight_Autonomous_Robot(17,0.5);

            //myRobot.moveForward_Autonomous_Robot(-2,1);
           // myRobot.lift_Move(0.4);

            myRobot.moveForward_Autonomous_Robot(-14,1);

            myRobot.setgripperServoPosition(0);
            sleep(500);


        }


        idle();
        targetsSkyStone.deactivate();

    }



}
