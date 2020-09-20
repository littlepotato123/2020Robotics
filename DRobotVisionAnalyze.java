
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

import com.vuforia.Frame;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.CameraDevice;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;


import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

import org.opencv.android.Utils;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Moments;

import org.opencv.android.OpenCVLoader ;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.DRobotUtil.FTC_FIELD_WIDTH;


public class DRobotVisionAnalyze {

    Point direction = new Point(0,0);
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    //public Bitmap currentCameraView = new Bitmap();

    int cameraMonitorViewId;
    VuforiaLocalizer vuforia;

    OpenGLMatrix redTargetLocationOnField;



    public Mat srcmat;
    public Mat dstmat;
    public Mat hsvmat;


    public final static int IMAGE_WIDTH = 320;
    public final static int IMAGE_HEIGHT = 180;

    public final static Scalar BEACON_YELLOW_LOW = new Scalar(18, 100, 0);
    public final static Scalar BEACON_YELLOW_HIGH = new Scalar(30, 255, 255);

    public final static Scalar BEACON_BLACK_LOW = new Scalar(0, 0, 0);
    public final static Scalar BEACON_BLACK_HIGH = new Scalar(255, 255, 0);


    public final static Scalar BEACON_WHITE_LOW = new Scalar(108, 0, 220);
    public final static Scalar BEACON_WHITE_HIGH = new Scalar(178, 255, 255);

    //public final static Scalar BEACON_YELLOW_LOW = new Scalar(18, 100, 0);
    //public final static Scalar BEACON_YELLOW_HIGH = new Scalar(30 , 255, 255);

    public final static Scalar BEACON_RED_LOW = new Scalar(108, 0, 220);
    public final static Scalar BEACON_RED_HIGH = new Scalar(178, 255, 255);

    public final static Scalar BEACON_BLUE_LOW = new Scalar(108, 0, 220);
    public final static Scalar BEACON_BLUE_HIGH = new Scalar(178, 255, 255);

   // public final static Scalar BEACON_WHITE_LOW = new Scalar(108, 0, 220);
    //public final static Scalar BEACON_WHITE_HIGH = new Scalar(178, 255, 255);


      public Point Analyze_Red(Mat srcMat,Mat hsvMat)
      {
          return direction;
      }

       public Point Analyze_Blue(Mat srcMat, Mat hsvMat)
       {
           return direction;
       }


    public List<Position> Analyze_yellow(int pass) {

        List<Position> mPositions = new ArrayList<Position>();
        Mat yellow = new Mat();
        Mat yellow_mask = new Mat();
        Mat Yellow_and = new Mat();
        Mat mHierarchy = new Mat();
        Mat gray = new Mat();
        Moments mMoments;
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Point[] vertices = new Point[4];
        // <MatOfPoint> boxcontours = new ArrayList<>();
        double maxArea = 0;

        if (srcmat == null) srcmat = new Mat();
        if (dstmat == null) dstmat = new Mat();
        if (hsvmat == null) hsvmat = new Mat();
        RobotLog.ii("In Analyze Yellow","Mat created")  ;




        //sleep(500);
        srcmat = getRobotView() ;

        if (srcmat == null) {
            RobotLog.ii("In Analyze_yellow", "Source Image is null");
            return mPositions;

        }

        sleep(400);
       Imgproc.cvtColor(srcmat, dstmat, Imgproc.COLOR_BGR2GRAY);
       Imgproc.cvtColor(srcmat, hsvmat, Imgproc.COLOR_BGR2HSV);


        Core.inRange(hsvmat, BEACON_YELLOW_LOW, BEACON_YELLOW_HIGH, yellow_mask);

        Core.bitwise_and(srcmat,srcmat,Yellow_and,yellow_mask);
        Imgproc.cvtColor(Yellow_and, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(gray , yellow , new Size(7, 7), 0);

        //Now on gray image do Canny Edge Detection
        //Imgproc.Canny(gray, yellow,50,100 );
        //Dilate and Erode to only leave Yellow objects
        //Imgproc.dilate(yellow,yellow,kernel,anchor,iterations);
        //Imgproc.erode(yellow,yellow,kernel,anchor,iterations);

        Imgproc.findContours(yellow, contours, mHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint  mMinerals = each.next();
            mMoments = Imgproc.moments(mMinerals);
            //Calculate Centroid and Area
            double area = Imgproc.contourArea(mMinerals);


            /*
            r.points(vertices);
            */
            if ((area > 390) && (area < 2150)) {

                Position centerOfMass = new Position();

                MatOfPoint2f pointsMat = new MatOfPoint2f(mMinerals.toArray());
                RotatedRect r = Imgproc.minAreaRect(pointsMat);
                double contourLength = Imgproc.arcLength(pointsMat,true);

                centerOfMass.x = mMoments.m10 / mMoments.m00;
                centerOfMass.y = mMoments.m01 / mMoments.m00;
                centerOfMass.z = area;
                centerOfMass.acquisitionTime = (long)contourLength;
                RobotLog.ii("In Analyze_yellow", "%f %f %f %d", centerOfMass.x, centerOfMass.y, centerOfMass.z,centerOfMass.acquisitionTime);
//                RobotLog.ii("In Analyze Yellow", "In Analyze Yellow  %f %f %f %d", centerOfMass.x, centerOfMass.y, centerOfMass.z,centerOfMass.acquisitionTime);
                mPositions.add(centerOfMass);
                /*

                boxcontours.add(new MatOfPoint(vertices));
                Imgproc.drawContours(srcMat,boxcontours,0,new Scalar(128,0,0),1);
                boxcontours.clear();
                */

            }



        }


        Collections.sort(mPositions, new Comparator<Position>() {
            @Override
            public int compare(Position p1, Position p2) {
                if (p1.y > p2.y) return 1;
                else if (p2.y > p1.y) return -1;
                else {
                    if (p1.x > p2.x) return 1;
                    else if (p2.x > p1.x) return -1;
                    else
                        return 0;
                }
            }
        });

        /*
        if (mPositions.size() > 0) {
            Imgcodecs imageCodecs = new Imgcodecs();

            File file = new File(captureDirectory, String.format(Locale.getDefault(), "RoboView-%d.bmp", pass));
            imageCodecs.imwrite(file.getAbsolutePath(), srcmat);

            File file1 = new File(captureDirectory, String.format(Locale.getDefault(), "Yellow-%d.bmp", pass));
            imageCodecs.imwrite(file1.getAbsolutePath(), Yellow_and);
        } */
        return mPositions;
    }

    public List<Position> Analyze_black(int pass) {

        List<Position> mPositions = new ArrayList<Position>();
        Mat black = new Mat();
        Mat black_mask = new Mat();
        Mat Black_and = new Mat();
        Mat mHierarchy = new Mat();
        Mat gray = new Mat();
        Moments mMoments;
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Point[] vertices = new Point[4];
        // <MatOfPoint> boxcontours = new ArrayList<>();
        double maxArea = 0;

        if (srcmat == null) srcmat = new Mat();
        if (dstmat == null) dstmat = new Mat();
        if (hsvmat == null) hsvmat = new Mat();
        RobotLog.ii("In Analyze Black","Mat created")  ;


        //sleep(500);
        srcmat = getRobotView() ;

        if (srcmat == null) {
            RobotLog.ii("In Analyze_black", "Source Image is null");
            return mPositions;

        }

        sleep(400);
        Imgproc.cvtColor(srcmat, dstmat, Imgproc.COLOR_BGR2GRAY);
        Imgproc.cvtColor(srcmat, hsvmat, Imgproc.COLOR_BGR2HSV);
        Imgproc.GaussianBlur(dstmat , black , new Size(5, 5), 0);

        Imgproc.threshold(black,black_mask,2,10,Imgproc.THRESH_BINARY );

        Imgproc.findContours(black_mask, contours, mHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint  mMinerals = each.next();
            mMoments = Imgproc.moments(mMinerals);
            //Calculate Centroid and Area
            double area = Imgproc.contourArea(mMinerals);


            /*
            r.points(vertices);
            */
            if ((area > 0) && (area < 350)) {

                Position centerOfMass = new Position();

                MatOfPoint2f pointsMat = new MatOfPoint2f(mMinerals.toArray());
                RotatedRect r = Imgproc.minAreaRect(pointsMat);
                double contourLength = Imgproc.arcLength(pointsMat,true);

                centerOfMass.x = mMoments.m10 / mMoments.m00;
                centerOfMass.y = mMoments.m01 / mMoments.m00;
                centerOfMass.z = area;
                centerOfMass.acquisitionTime = (long)contourLength;
                RobotLog.ii("In Analyze_black", "%f %f %f %d", centerOfMass.x, centerOfMass.y, centerOfMass.z,centerOfMass.acquisitionTime);
//                RobotLog.ii("In Analyze Yellow", "In Analyze Yellow  %f %f %f %d", centerOfMass.x, centerOfMass.y, centerOfMass.z,centerOfMass.acquisitionTime);
                mPositions.add(centerOfMass);
                /*

                boxcontours.add(new MatOfPoint(vertices));
                Imgproc.drawContours(srcMat,boxcontours,0,new Scalar(128,0,0),1);
                boxcontours.clear();
                */

            }



        }


        Collections.sort(mPositions, new Comparator<Position>() {
            @Override
            public int compare(Position p1, Position p2) {
                if (p1.y > p2.y) return 1;
                else if (p2.y > p1.y) return -1;
                else {
                    if (p1.x > p2.x) return 1;
                    else if (p2.x > p1.x) return -1;
                    else
                        return 0;
                }
            }
        });


        if (mPositions.size() > 0) {
            Imgcodecs imageCodecs = new Imgcodecs();

            File file = new File(captureDirectory, String.format(Locale.getDefault(), "RoboView-%d.bmp", pass));
            imageCodecs.imwrite(file.getAbsolutePath(), srcmat);

            File file1 = new File(captureDirectory, String.format(Locale.getDefault(), "Black-%d.bmp", pass));
            Imgproc.cvtColor(black_mask,black, Imgproc.COLOR_GRAY2BGR);
            imageCodecs.imwrite(file1.getAbsolutePath(), black );
        }
        return mPositions;
    }










    public List<Position> Analyze_white(int pass) {

        List<Position> mPositions = new ArrayList<Position>();
        Mat white = new Mat();
        Mat gray = new Mat();
        Mat White_and = new Mat();
        Mat white_mask = new Mat();
        Mat mHierarchy = new Mat();



        Moments mMoments;
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        double maxArea = 0;

        if (srcmat == null) srcmat = new Mat();
        if (dstmat == null) dstmat = new Mat();
        if (hsvmat == null) hsvmat = new Mat();
        RobotLog.ii("In White","Mat created")  ;








        //RobotLog.ii("Before Call", "%d %d", srcMat.rows(), srcMat.cols());
        srcmat = getRobotView() ;

        if (srcmat == null) {
            RobotLog.ii("In Analyze_white", "Source Image is null");
            return mPositions;
        }
        Imgproc.cvtColor(srcmat, dstmat, Imgproc.COLOR_BGR2GRAY);
        Imgproc.cvtColor(srcmat, hsvmat, Imgproc.COLOR_BGR2HSV);

        Core.inRange(hsvmat, BEACON_WHITE_LOW, BEACON_WHITE_HIGH, white_mask);

        //Bitwise And with original image to put only Yellow Colors
        Core.bitwise_and(srcmat,srcmat,White_and,white_mask);

        //Convert White colored image to Gray
        Imgproc.cvtColor(White_and, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(gray , white , new Size(5, 5), 0);


        //Imgproc.threshold(srcMat,white,225,255,Imgproc.THRESH_BINARY);
        Imgproc.findContours(white, contours, mHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        Iterator<MatOfPoint> each = contours.iterator();

        while (each.hasNext()) {
            MatOfPoint mMinerals = each.next();
            mMoments = Imgproc.moments(mMinerals);
            //Calculate Centroid and Area
            double area = Imgproc.contourArea(mMinerals);

            if ((area > 350)&& (area < 2500)) {

                Position centerOfMass = new Position();
                MatOfPoint2f pointsMat = new MatOfPoint2f(mMinerals.toArray());
                RotatedRect r = Imgproc.minAreaRect(pointsMat);
                double contourLength = Imgproc.arcLength(pointsMat,true);

                centerOfMass.x = mMoments.m10 / mMoments.m00;
                centerOfMass.y = mMoments.m01 / mMoments.m00;
                centerOfMass.z = area;
                centerOfMass.acquisitionTime = (long)contourLength;
//                RobotLog.ii("In Analyze_yellow", "%f %f %f %d", centerOfMass.x, centerOfMass.y, centerOfMass.z,centerOfMass.acquisitionTime);

                mPositions.add(centerOfMass);

            }


        }


        Collections.sort(mPositions, new Comparator<Position>() {
            @Override
            public int compare(Position p1, Position p2) {
                if (p1.y > p2.y) return 1;
                else if (p2.y > p1.y) return -1;
                else {
                    if (p1.x > p2.x) return 1;
                    else if (p2.x > p1.x) return -1;
                    else
                        return 0;
                }
            }
        });


        if (mPositions.size() > 0) {
            Imgcodecs imageCodecs = new Imgcodecs();

            File file = new File(captureDirectory, String.format(Locale.getDefault(), "RoboView-%d.bmp", pass));
            imageCodecs.imwrite(file.getAbsolutePath(), srcmat);

            File file1 = new File(captureDirectory, String.format(Locale.getDefault(), "White-%d.bmp", pass));
            imageCodecs.imwrite(file1.getAbsolutePath(), white);
        }

        return mPositions;
    }


    public Mat getRobotView()
    {
        //final Mat srcmat = new Mat();

        //sleep(200);

        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Frame fr = frame.clone();
                Bitmap bm = vuforia.convertFrameToBitmap(fr);

                if (bm != null) {

                    //telemetry.log().add("Bitmap is NOTNULL");
                    //telemetry.update();
                    RobotLog.ii("get Robot View","Bitmap is NotNULL")  ;

                    Utils.bitmapToMat(bm, srcmat);
                    Imgproc.cvtColor(srcmat ,srcmat, Imgproc.COLOR_RGB2BGR);

                    //RobotLog.ii("After Utils", "srcmat.rows=%d srcmat.cols=%d", srcmat.rows(), srcmat.cols());
                    //Imgproc.GaussianBlur(srcmat, srcmat, new Size(5, 5), 0);
                    Imgproc.resize(srcmat, srcmat, new Size(IMAGE_WIDTH, IMAGE_HEIGHT));

                    //Imgproc.cvtColor(srcmat, dstmat, Imgproc.COLOR_BGR2GRAY);
                    //Imgproc.cvtColor(srcmat, hsvmat, Imgproc.COLOR_BGR2HSV);


                } else
                    {
                    //telemetry.log().add("Bitmap is NULL");
                    RobotLog.ii("get Robot View","Bitmap is NULL")  ;

                    }

            }
        }));

        RobotLog.ii("GetFrame", "Converted the images into gray and hsv");
        return srcmat;
    }

    public void Initialize_Vision(HardwareMap hardwareMap) {
        //Initialize Robot
        //myRobot.Initialize_Autonomous_Robot(hardwareMap);

        //Initialize Vuforia
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "ASK5yBH/////AAABmTU1FcFdrUcoj+ogluo/eWoShGiH8QPFpaGBeLrauvdQuQHSqF4YZEiXCwqFG38x4zKjIztJg2dxwD5lLlnQIs54XbqL01Xp3vzcORMq7pdV6ydGBOsAQlga62JkWlMw4jax3Z23WPQcvUSYeAhUv4CYmNKDMyXlUW2Dx4R4Gvevay5dVVQKvcfBZFcwbPL5uSC7q8MXgvksbsJHLIa+DNvo2QseEwLd0NT9oYA0g6MbBmj8UGneV+z3FNVjrHYv1DJtnFypT/VUIidBiexruourO5x0Ulnah2Hu03PnyDVftuBs6f9NeKQfBmTkGvHFa9r0tPqntoI1ehvdjYmilB4tBveeXvm3fv+lBA2u2Fv0";
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();
        CameraDevice.getInstance().setFlashTorchMode(true);

        //Load OpenCV
        if (!OpenCVLoader.initDebug()) {
            //Log.d("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            //RobotLog.ee(TAG, e, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            //telemetry.addData("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            //telemetry.update();
            RobotLog.ii("OpenCV","Internal OpenCV library not found. Using OpenCV Manager for initialization")  ;
        }
        else
            {
            //telemetry.addData("OpenCV", "Loaded");
            //telemetry.update();
                RobotLog.ii("OpenCV","OpenCV Loaded")  ;
        }
        // Allocate memory to all the three MAT files

    }


     public void Initialize_Vuforia_Trackables() {
        //Add Code here to Initialize All Trackables

        VuforiaTrackables stonesAndChips = vuforia.loadTrackablesFromAsset("StonesAndChips");
        VuforiaTrackable redTarget = stonesAndChips.get(0);
        redTarget.setName("RedTarget");  // Stones

        VuforiaTrackable blueTarget  = stonesAndChips.get(1);
        blueTarget.setName("BlueTarget");  // Chips

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(stonesAndChips);

         redTargetLocationOnField = OpenGLMatrix
                 /* Then we translate the target off to the RED WALL. Our translation here
                 is a negative translation in X.*/
                 .translation(-FTC_FIELD_WIDTH/2, 0, 0)
                 .multiplied(Orientation.getRotationMatrix(
                         /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                         AxesReference.EXTRINSIC, AxesOrder.XZX,
                         AngleUnit.DEGREES, 90, 90, 0));
         redTarget.setLocationFtcFieldFromTarget(redTargetLocationOnField);
         RobotLog.ii("Hello", "Red Target=%s", format(redTargetLocationOnField));




     }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }


}
