package org.firstinspires.ftc.teamcode;

//CAMERA IMPORTS
import android.annotation.SuppressLint;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

//ROAD RUNNER IMPORTS
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Red Autos")
public class RedRightAutonomous extends LinearOpMode {

    //VARIABLES ------------------------------------------------------------------------------------

    //ROBOT SIZE
    public static double width = 12.75/2; //center of width
    public static double length = 15.625/2; //center of length

    //CONES
    public static int numCones = 4; //ONLY DO 4 IF YOU ARE FAST
    public static int coneOffset = 140;
    public static int coneTop = -750; //prev 750
    public static int coneHeight;

    //CAMSERVO
    public static double right = 0.15;
    public static double left = 1.0;

    //ARM LIFT
    public static int zero = 0;
    public static int low = -1150;  //prev -1625
    public static int medium = -2150; //prev -2650
    public static int high = -2800;

    //SHOULDER
    public static double inward = 0.11;
    public static double scoreArm = 0.315; //prev 0.325
    public static double outward = 0.4;

    //ELBOW
    public static double downward = 0.0;
    public static double scoreElbow = 0.25;
    public static double upward = 0.4;

    //GRABBER
    public static double close = 0.4;
    public static double open = 1.0;

    //CAMERA
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    int ONE = 1;
    int TWO = 2;
    int THREE = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException{
        //HARDWARE INITIALIZATION ------------------------------------------------------------------
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        //SERVO INIT
        robot.camservo.setPosition(right);
        robot.shoulder.setPosition(inward);
        robot.elbow.setPosition(downward);
        robot.grabber.setPosition(close);

        //ROAD RUNNER TRAJECTORIES -----------------------------------------------------------------
        double startHeading = Math.toRadians(0);
        Pose2d startPose = new Pose2d(48-length -4, -48-width -7.5, startHeading);
        drive.setPoseEstimate(startPose);

        //PRELOAD SCORE + GOTO CONE STACK
        TrajectorySequence conePreload = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(48-length - 1.5, -21)) //move to low pole
                .strafeTo(new Vector2d(48-length - 3.5, -14)) //line up with cone stack
                .splineToConstantHeading(new Vector2d(60, -14), startHeading) //go to cone stack position
                .splineToConstantHeading(new Vector2d(65.5, -14), startHeading) //go to cone stack
                .addTemporalMarker(0, () -> robot.liftArm(low, 1)) //raise to low pole
                .addTemporalMarker(0, () -> robot.elbow.setPosition(scoreElbow)) //raise elbow to score
                .addTemporalMarker(1.45, () -> robot.elbow.setPosition(downward)) //dunk
                .addTemporalMarker(1.65, () -> robot.grabber.setPosition(open)) //score preload
                .addTemporalMarker(1.85, () -> robot.liftArm(coneTop, 1)) //raise to low pole
                .build();

        //GOTO POLE + SCORE
        TrajectorySequence coneScore = drive.trajectorySequenceBuilder(conePreload.end())
                .waitSeconds(0.5) //prev 0.5
                .lineTo(new Vector2d(20.5, -14)) //goto score pos
                .waitSeconds(1.5)  //prev 1.5
                .addTemporalMarker(0, () -> robot.grabber.setPosition(close)) //grab cone from stack
                .addTemporalMarker(0.5, () -> robot.liftArm(medium, 1)) //raise arm to pole
                .addTemporalMarker(2, () -> robot.shoulder.setPosition(scoreArm)) //move shoulder to align
                .addTemporalMarker(2, () -> robot.elbow.setPosition(scoreElbow)) //move elbow to align
                .addTemporalMarker(3, () -> robot.elbow.setPosition(downward)) //dunk
                .addTemporalMarker(3.25, () -> robot.grabber.setPosition(open)) //score prev 3
                .addTemporalMarker(3.5, () -> robot.elbow.setPosition(scoreElbow)) //return elbow
                .build();

        //RETURN TO CONE STACK
        TrajectorySequence coneStack = drive.trajectorySequenceBuilder(coneScore.end())
                .addTemporalMarker(0.25, () -> robot.elbow.setPosition(downward)) //default pos
                .lineTo(new Vector2d(66, -14)) //cone stack pos
                .build();

        //PARKING ONE
        TrajectorySequence parkingOne = drive.trajectorySequenceBuilder(coneScore.end())
                .lineTo(new Vector2d(16, -14))
                .build();

        //PARKING TWO
        TrajectorySequence parkingTwo = drive.trajectorySequenceBuilder(coneScore.end())
                .lineTo(new Vector2d(40, -14))
                .build();

        //PARKING THREE
        TrajectorySequence parkingThree = drive.trajectorySequenceBuilder(coneScore.end())
                .lineTo(new Vector2d(64, -14))
                .build();

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //WHILE WAIT FOR START FIND APRIL TAG ------------------------------------------------------
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;
                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ONE || tag.id == TWO || tag.id == THREE)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

            telemetry.update();
            sleep(20);
        }

        //EXECUTE SCRIPT ---------------------------------------------------------------------------
        //PRELOAD + FIRST CONE FROM STACK
        drive.followTrajectorySequence(conePreload);
        drive.followTrajectorySequence(coneScore);

        //CYCLE CONES
        for (int i = 1; i < numCones; i++) {
            robot.shoulder.setPosition(inward);
            sleep(500);
            coneHeight = coneTop + i*coneOffset;
            robot.liftArm(coneHeight, 1);

            drive.followTrajectorySequence(coneStack);
            drive.followTrajectorySequence(coneScore);
        }

        //DROP ARM BEFORE PARKING
        robot.shoulder.setPosition(inward);
        sleep(500);
        robot.liftArm(zero, 1);

        //PARK
        if(tagOfInterest == null || tagOfInterest.id == THREE){
            drive.followTrajectorySequence(parkingThree);
        }else if(tagOfInterest.id == TWO){
            drive.followTrajectorySequence(parkingTwo);
        }else{
            drive.followTrajectorySequence(parkingOne);
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}