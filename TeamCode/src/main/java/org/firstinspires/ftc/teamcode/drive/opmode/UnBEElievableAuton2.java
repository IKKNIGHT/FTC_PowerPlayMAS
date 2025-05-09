package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Op mode for computing kV, kStatic, and kA from various drive routines. For the curious, here's an
 * outline of the procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the robot (apply constant power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 */
@Autonomous
public class UnBEElievableAuton2 extends LinearOpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;

    private DcMotorEx arm;
    private DcMotorEx arm1;

    private Servo camera;
    private Servo shoulder;
    private Servo elbow;
    private Servo grabber;
    private Servo wrist;






    @Override
    public void runOpMode() throws InterruptedException {


        frontLeft = hardwareMap.get(DcMotorEx.class,"frontleft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backleft");
        backRight = hardwareMap.get(DcMotorEx.class, "backright");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        camera = hardwareMap.get(Servo.class, "camservo");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        elbow = hardwareMap.get(Servo.class, "elbow");
        grabber = hardwareMap.get(Servo.class, "grabber");
        wrist = hardwareMap.get(Servo.class, "wrist");




        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        //backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        boolean grabberOpen = false;
        boolean shoulderOut = false;
        boolean elbowUp = false;

        UselessClass rayyan = new UselessClass(hardwareMap);

        grabber.setPosition(0.35);

        waitForStart();




            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


            Pose2d startPose = new Pose2d(0, -0, Math.toRadians(0));

            TrajectorySequence conePreload = drive.trajectorySequenceBuilder(startPose)

                    .strafeRight(56)
                    .back(22.5)
                    .addTemporalMarker(0, () -> rayyan.liftArm(-2700, 1)) //raise to high pole
                    .addTemporalMarker(0.5, () -> elbow.setPosition(0)) //raise elbow to score
                    .addTemporalMarker(2.5, () -> shoulder.setPosition(0.55)) //open shoulder
                    .addTemporalMarker(3, () -> elbow.setPosition(0.85)) //dunk
                    .addTemporalMarker(4.5, () -> grabber.setPosition(0.45)) //score preload
                    .addTemporalMarker(5, () -> elbow.setPosition(0)) //raise up
                    .addTemporalMarker(5.5, () -> shoulder.setPosition(1)) //close shoulder
                    .addTemporalMarker(6, () -> rayyan.liftArm(0, 1)) //bring back down
                    //.addTemporalMarker(1.85, () -> robot.liftArm(coneTop, 1)) //raise to low pole
                    .build();

            TrajectorySequence grabCone = drive.trajectorySequenceBuilder(conePreload.end())
                  //  .strafeLeft(.4)
                    .forward (49.5)

                    .addTemporalMarker(0.3, () -> rayyan.liftArm(0, 1)) //bring back down

                    .addTemporalMarker(2, () -> elbow.setPosition(0.85)) //lower elbow to pick up
                    .addTemporalMarker(2.5, () -> grabber.setPosition(0.35)) //intake with grabber
                    .waitSeconds(1)
                    .addTemporalMarker(3.5, () -> elbow.setPosition(0)) //raise mini-up
                    .build();
            /*
        TrajectorySequence back = drive.trajectorySequenceBuilder(grabCone.end())
                .back(49.5)
                .addTemporalMarker(0, () -> rayyan.liftArm(-3000, 1)) //raise to high pole
                .build();
        */
        TrajectorySequence score = drive.trajectorySequenceBuilder(grabCone.end())
                // .addTemporalMarker(0, () -> rayyan.liftArm(-3000, 1)) //raise to high pole
                .back(49.5)
               // .strafeLeft(0.75)
                .addTemporalMarker(0., () -> rayyan.liftArm(-2700, 1)) //raise to high pole
                .addTemporalMarker(0.8, () -> elbow.setPosition(0)) //raise elbow to score
                .addTemporalMarker(1.5, () -> shoulder.setPosition(0.55)) //open shoulder
                .waitSeconds(0.8)
                .addTemporalMarker(2.25, () -> elbow.setPosition(0.85)) //dunk
                .addTemporalMarker(2.25, () -> grabber.setPosition(0.45)) //score preload
                .addTemporalMarker(2.5, () -> elbow.setPosition(0)) //raise up
                .addTemporalMarker(4.5, () -> shoulder.setPosition(1)) //close shoulder
                .addTemporalMarker(4.5, () -> rayyan.liftArm(0, 1)) //bring back down
                //.addTemporalMarker(1.85, () -> robot.liftArm(coneTop, 1)) //raise to low pole
                .build();




        drive.followTrajectorySequence(conePreload);
        drive.followTrajectorySequence(grabCone);
        drive.followTrajectorySequence(score);
        drive.followTrajectorySequence(grabCone);
        drive.followTrajectorySequence(score);
        drive.followTrajectorySequence(grabCone);
        drive.followTrajectorySequence(score);
        drive.followTrajectorySequence(grabCone);
        drive.followTrajectorySequence(score);
        drive.followTrajectorySequence(grabCone);
        drive.followTrajectorySequence(score);


        telemetry.addData("elbow", elbow.getPosition());
                telemetry.addData("camera", camera.getPosition());
                telemetry.addData("grabber", grabber.getPosition());
                telemetry.addData("wrist", wrist.getPosition());
                telemetry.addData("shoulder", shoulder.getPosition());
                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("finalX", poseEstimate.getX());
                telemetry.addData("finalY", poseEstimate.getY());
                telemetry.addData("finalHeading", poseEstimate.getHeading());

                telemetry.update();
            }


        }

// Testing Github



