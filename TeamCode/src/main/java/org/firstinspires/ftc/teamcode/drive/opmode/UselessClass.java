package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class UselessClass {

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

    public UselessClass(HardwareMap hardwareMap) {
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


    }

    public void liftArm(int distance, double power){
        arm.setTargetPosition(distance);
        arm1.setTargetPosition(-distance);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setPower(power);
        arm.setPower(power);
    }
}
