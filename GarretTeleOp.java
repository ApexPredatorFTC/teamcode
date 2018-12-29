package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Admin on 10/25/2017.
 */

@TeleOp(name = "Garret TeleOp", group = "")
public class GarretTeleOp extends LinearOpMode {
    DcMotor motor_left;
    DcMotor motor_right;
    DcMotor lifter;
    DcMotor flipper;
    DcMotor leftIntake;
    DcMotor rightIntake;



    public int armPos = 0;
    @Override
    public void runOpMode(){
        motor_left = hardwareMap.get(DcMotor.class, "left");
        motor_right = hardwareMap.get(DcMotor.class,"right");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        flipper = hardwareMap.get(DcMotor.class, "flipper");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");


        flipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();

        while(opModeIsActive()) {


            handleDriveTrain();
            liftingTheArm();
            flipTheBlock();
            Intake();
            telemetry.update();
        }


        motor_left.setPower(0);
        //motor_left2.setPower(0);
        motor_right.setPower(0);
        //motor_right2.setPower(0);




    }

    public void handleDriveTrain()
    {
        double a = .75;

        double yvalue = a*Math.pow(gamepad1.right_stick_y,3) + ((1-a)*gamepad1.right_stick_y);;
        double xvalue = a*Math.pow(gamepad1.right_stick_x, 3) + ((1-a)*gamepad1.right_stick_x);
        //xvalue=-xvalue;

        double W = (1-Math.abs(yvalue)) * (xvalue) + xvalue;
        double V = (1-Math.abs(xvalue)) * (yvalue) + yvalue;
        double R = (W+V)/2;
        double L = (W-V)/2;

        motor_left.setPower(L);
        //motor_left2.setPower(L);
        motor_right.setPower(R);
        //motor_right2.setPower(R);

    }
    public void liftingTheArm() {
        if (gamepad2.left_stick_y < 0) {
            lifter.setPower(.7);
        }

        else if (gamepad2.left_stick_y > 0) {
                lifter.setPower(-.3);
            }

            else {
            lifter.setPower(0);
        }
    }

    public void Intake() {
        if (gamepad2.right_trigger > 0) {
            leftIntake.setPower(1);
            rightIntake.setPower(-1);
        }
        if (gamepad2.left_trigger > 0) {
            leftIntake.setPower(-1);
            rightIntake.setPower(1);
        }
        else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }
    public void flipTheBlock ()
    {
        if (gamepad2.dpad_up) {
            flipper.setPower(.4);
        }
        else if (gamepad2.dpad_down){
            flipper.setPower(-.7);
        }
        else{
            flipper.setPower(0);
        }

    }

    ///////////////////////////
    //controls movment of the claw

    //controls movment of the Arm
    ///////////////////////////
   }
