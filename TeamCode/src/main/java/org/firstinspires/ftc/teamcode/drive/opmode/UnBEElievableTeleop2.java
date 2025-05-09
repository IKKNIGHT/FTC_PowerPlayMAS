package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rpmToVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.LoggingUtil;
import org.firstinspires.ftc.teamcode.util.RegressionUtil;

import java.util.ArrayList;
import java.util.List;

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
@TeleOp
public class UnBEElievableTeleop2 extends LinearOpMode {
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
    private DigitalChannel limit;



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
        limit = hardwareMap.get(DigitalChannel.class, "limitSwitch");


        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        //backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        boolean grabberOpen = false;
        boolean shoulderOut = false;
        boolean elbowUp = false;

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;



            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if(limit.getState())
            {
                arm.setPower(gamepad2.left_stick_y);
                arm1.setPower(-gamepad2.left_stick_y);
            }
            else if (gamepad2.left_stick_y < 0)
            {
                arm.setPower(gamepad2.left_stick_y);
                arm1.setPower(-gamepad2.left_stick_y);
            }
            else
            {
                arm.setPower(0);
                arm1.setPower(0);
            }
            if (gamepad2.right_bumper)
            {
                if (grabberOpen)
                {
                    grabber.setPosition(0.45);
                    grabberOpen = false;
                    sleep(500);
                }
                else
                {
                    grabber.setPosition(.35);
                    grabberOpen = true;
                    sleep(500);
                }

            }
            if (gamepad2.y)
            {
                if (elbowUp)
                {
                    elbow.setPosition(0);
                    elbowUp = false;
                    sleep (500);
                }
                else
                {
                    elbow.setPosition(0.85);
                    elbowUp = true;
                    sleep(500);
                }

            }
            if (gamepad2.b)
            {
                camera.setPosition(0.4);
            }
            if(gamepad2.a)
            {
                wrist.setPosition(01);
            }
            if(gamepad2.x)
            {
                if (shoulderOut){
                    shoulder.setPosition(1);
                    shoulderOut = false;
                    sleep(500);
                }
                else
                {
                    shoulder.setPosition(.55);
                    shoulderOut = true;
                    sleep(500);
                }

            }

            telemetry.addData("elbow", elbow.getPosition());
            telemetry.addData("camera", camera.getPosition());
            telemetry.addData("grabber", grabber.getPosition());
            telemetry.addData("wrist", wrist.getPosition());
            telemetry.addData("shoulder", shoulder.getPosition());
            telemetry.addData("armEncoder", arm.getCurrentPosition());
            telemetry.addData("arm1Encoder", arm1.getCurrentPosition());
            telemetry.addData("limit", limit.getState());

            telemetry.update();
        }



}
}