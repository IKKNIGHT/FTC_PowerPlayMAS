package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TeleopAndroid extends LinearOpMode {

    private DcMotor frontright;
    private DcMotor backright;
    private DcMotor frontleft;
    private DcMotor backleft;
    private DcMotor arm;
    private Servo grabber;
    private Servo shoulder;
    private Servo elbow;
    private Servo camservo;


    private String state = "grabberPos";
    private Boolean translating = false;
    private Boolean firstTouch = false;
    private int destination;
    private int armTarget = 0;

    private int counter = 0;
    private static int low = -1500;
    private static int med = -2800;
    private static int high = -2800;
    private boolean buttonReleased = true;

    private double position1 = 0.1; //shoulder position LR [0.11 - 0.4]
    private double position2 = 0.0; //elbow position DU [0.0 - 0.4]

    public ElapsedTime timer;

    public void runOpMode() {
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        arm = hardwareMap.get(DcMotor.class, "arm");
        grabber=hardwareMap.get(Servo.class,"grabber");
        shoulder=hardwareMap.get(Servo.class,"shoulder");
        elbow=hardwareMap.get(Servo.class,"elbow");
        camservo=hardwareMap.get(Servo.class,"camservo");

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // arm.setTargetPosition(0);
        // arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // arm.setPower(1.0);

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         */

        backright.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();


        while (opModeIsActive()) {

            //(1) DRIVE CONTROLLER SETUP ---------------------------------------
            double lefty1 = gamepad1.left_stick_y;
            double leftx1 = gamepad1.left_stick_x;
            double rightx1 = gamepad1.right_stick_x;


            //DRIVING JOYSTICK
            if(gamepad1.right_bumper){
                lefty1 = gamepad1.left_stick_y*0.5;
                leftx1 = gamepad1.left_stick_x*0.5;
                rightx1 = gamepad1.right_stick_x*0.5;
                frontleft.setPower((0.8 * -lefty1) + (leftx1 * 0.5) + rightx1);
                backright.setPower((0.725 * lefty1) - (leftx1 * 0.425) + rightx1);
                frontright.setPower((0.725 * lefty1) + (leftx1 * 0.425) + rightx1);
                backleft.setPower((0.8 * lefty1) - (-leftx1 * 0.5) + -rightx1);
            } else {
                lefty1 = gamepad1.left_stick_y;
                leftx1 = gamepad1.left_stick_x;
                rightx1 = gamepad1.right_stick_x;
                frontleft.setPower((0.8 * -lefty1) + (leftx1 * 0.5) + rightx1);
                backright.setPower((0.725 * lefty1) - (leftx1 * 0.425) + rightx1);
                frontright.setPower((0.725 * lefty1) + (leftx1 * 0.425) + rightx1);
                backleft.setPower((0.8 * lefty1) - (-leftx1 * 0.5) + -rightx1);

            }


            //(2) GRABBER CONTROLLER SETUP -------------------------------------
            double lefty = gamepad2.left_stick_y;
            boolean rbumper = gamepad2.right_bumper;
            boolean lbumper = gamepad2.left_bumper;

            //ARM UP-DOWN MOVEMENT
            if(gamepad2.left_stick_y > 0.05 || gamepad2.left_stick_y <  -0.05 ){
                arm.setPower(gamepad2.left_stick_y);
            }else{
                arm.setPower(0.0);
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // // BETTER ARM MOVEMENT
            // if(armTarget > -2800 && gamepad2.left_stick_y < 0){
            //     armTarget += gamepad2.left_stick_y * 50;
            //     arm.setTargetPosition(armTarget);
            // }

            // else if(armTarget < 0 && gamepad2.left_stick_y > 0) {
            //     armTarget += gamepad2.left_stick_y * 50;
            //     arm.setTargetPosition(armTarget);

            // }

            // //goes home
            // else if(gamepad2.dpad_down){
            //     position1 = 0.0;
            //     position2 = 0.0;
            //     armTarget = 0;
            //     arm.setTargetPosition(armTarget);
            //     counter = 0;
            // }

            // //goes up 3 levels
            // else if(gamepad2.dpad_up){
            //     if (buttonReleased){
            //         buttonReleased = false;
            //         switch(counter){
            //             case 0:
            //                 armTarget = low;
            //                 counter = 1;
            //                 break;
            //             case 1:
            //                 armTarget = med;
            //                 counter = 2;
            //                 break;
            //             case 2:
            //                 armTarget = high;
            //                 position2 = 4;
            //                 counter = 3;
            //                 break;
            //         }
            //     }
            //     arm.setTargetPosition(armTarget);
            // }

            // else{
            //     buttonReleased = true;
            // }

            //GRABBER MOVEMENT
            if(gamepad2.right_bumper){
                grabber.setPosition(0.4);  //CLOSED GRABBER
                telemetry.addLine("yo");
            } else if(gamepad2.left_bumper){
                grabber.setPosition(1);   //OPEN GRABBER
                telemetry.addLine("bro");
            }

            //SHOULDER MOVEMENT LEFT-RIGHT
            if(gamepad2.right_stick_x > 0.2 && position1 < 0.4) { // CHANGE GAMEPAD.RIGHT_STICK > or < to SWITCH DIRECTION
                position1 += gamepad2.right_stick_x * 0.0125; //MAKE THIS NEGATIVE OR POSITIVIE -= or +=
            } else if(gamepad2.right_stick_x < -0.2 && position1 > 0.11) {
                position1 += gamepad2.right_stick_x * 0.0125;
            }
            shoulder.setPosition(position1);

            //ELBOW MOVEMENT UP-DOWN HIGHT POST
            if(gamepad2.right_stick_y < -0.2 && position2 < 0.4) {
                position2 -= gamepad2.right_stick_y * 0.0125;
            } else if(gamepad2.right_stick_y > 0.2 && position2 > 0.0) {
                position2 -= gamepad2.right_stick_y * 0.0125;
            }
            elbow.setPosition(position2);

            telemetry.addData("gamebad position:", gamepad2.right_stick_x);
            telemetry.addData("", "");
            telemetry.addData("FR encoder pos:", frontright.getCurrentPosition());
            telemetry.addData("BR encoder pos:", backright.getCurrentPosition());
            telemetry.addData("FL encoder pos:", frontleft.getCurrentPosition());
            telemetry.addData("BL encoder pos:", backleft.getCurrentPosition());
            telemetry.addData("", "");
            telemetry.addData("Arm encoder pos:", arm.getTargetPosition());
            telemetry.addData("Arm power:", arm.getPower());
            telemetry.addData("shoulder pos:", shoulder.getPosition());
            telemetry.addData("elbow pos:", elbow.getPosition());
            telemetry.addData("grabber pos:", grabber.getPosition());
            telemetry.addData("armTarget:", armTarget);
            telemetry.addData("counter", counter);

            telemetry.update();

        }
    }
}





