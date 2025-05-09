package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
DEFINE ALL ROBOT HARDWARE IN THIS CLASS ONLY. REFERENCE IN AUTONOMOUS OPMODE FROM HERE
 */

public class BaseRobotMethods {

    //VARIABLES ------------------------------------------------------------------------------------
    private ElapsedTime     runtime = new ElapsedTime();

    public DcMotorEx arm;
    public DcMotorEx frontleft;
    public DcMotorEx frontright;
    public DcMotorEx backleft;
    public DcMotorEx backright;

    public Servo camservo;
    public Servo shoulder;
    public Servo elbow;
    public Servo grabber;

    public LinearOpMode parent;
    public Telemetry telemetry;

    private int timeout = 7; //seconds until command abort

    //HARDWARE SETUP -------------------------------------------------------------------------------
    public BaseRobotMethods(HardwareMap hardwareMap) { //init all hardware here
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        backleft = hardwareMap.get(DcMotorEx.class, "backleft");
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        shoulder = hardwareMap.get(Servo.class, "shoulder");
        elbow = hardwareMap.get(Servo.class, "elbow");
        grabber = hardwareMap.get(Servo.class, "grabber");

        camservo = hardwareMap.get(Servo.class,"camservo");

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //MOTOR COMMANDS   -----------------------------------------------------------------------------
    public void setMotorMode(DcMotor.RunMode mode){
        frontleft.setMode(mode);
        frontright.setMode(mode);
        backleft.setMode(mode);
        backright.setMode(mode);
    }

    public void setMotorPosition(int pos1, int pos2, int pos3, int pos4){
        frontleft.setTargetPosition(pos1); //set encoder ticks target
        frontright.setTargetPosition(pos2);
        backleft.setTargetPosition(pos3);
        backright.setTargetPosition(pos4);
    }

    public void setMotorPower(double speed1, double speed2, double speed3, double speed4){
        frontleft.setPower(speed1); //set motor power target
        frontright.setPower(speed2);
        backleft.setPower(speed3);
        backright.setPower(speed4);

        //run motors until one of them stops
        while(parent.opModeIsActive() &&  (runtime.seconds() < timeout) && (frontleft.isBusy()
                && frontright.isBusy() && backleft.isBusy() && backright.isBusy())){

            telemetry.addData("encoder-fwd-left", frontleft.getCurrentPosition() + "busy=" + frontleft.isBusy());
            telemetry.addData("encoder-fwd-right", frontright.getCurrentPosition() + "busy=" + frontright.isBusy());
            telemetry.addData("encoder-bkw-left", backleft.getCurrentPosition() + "busy=" + backleft.isBusy());
            telemetry.addData("encoder-bkw-right", backright.getCurrentPosition() + "busy=" + backright.isBusy());
            telemetry.update();
        }

        stopMovement();
        runtime.reset();
    }

    public void stopMovement(){
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleft.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
    }

    //ARM COMMANDS ---------------------------------------------------------------------------------
    public void liftArm(int distance, double power){
        arm.setTargetPosition(distance);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
    }

    public void stopArm(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(0);
    }
    public static double Tiles(double amt_of_tiles){
        return (double) amt_of_tiles*24;
    }
}
