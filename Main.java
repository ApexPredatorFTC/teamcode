package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Admin on 10/3/2018.
 */
//Drive Train Works for 2018 Robotics
@TeleOp(name = "Main", group = "")
public class Main extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor motor_lift_hook;
    DcMotor motor_lift_grabber1;
    DcMotor motor_lift_grabber2;
    Servo leftGrab;
    Servo rightGrab;
    private double x = 1;
    public int armPos = 0;
    double left_position = 0;
    double right_position = 0;

    public void runOpMode() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        motor_lift_hook = hardwareMap.dcMotor.get("motor_lift_hook");
        motor_lift_grabber1 = hardwareMap.dcMotor.get("motor_lift_grabber1");
        motor_lift_grabber2 = hardwareMap.dcMotor.get("motor_lift_grabber2");
        leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        rightGrab = hardwareMap.get(Servo.class, "rightGrab");


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_lift_hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_lift_grabber1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_lift_grabber2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor_lift_grabber1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_lift_grabber2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_lift_hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_lift_grabber1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_lift_grabber2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftGrab.setPosition(0);
        rightGrab.setPosition(0);
        waitForStart();

        while (opModeIsActive()) {
            HandleTheDriveTrain();
            ManipulateHookLift();
            ManipulateGrabbing();
            liftGrabber();
        }
    }

    public void HandleTheDriveTrain() {
        //Scaling the joystick inputs for sensitivity
        float y_raw = gamepad1.left_stick_y * .9f;
        float x_raw = gamepad1.left_stick_x * .9f;
        float z_raw = gamepad1.right_stick_x * 0.85f;
        float xscale = (float) 0.75;
        float yscale = (float) 0.75;
        float zscale = (float) 0.65;
        float x = -(xscale * (float) Math.pow(x_raw, 7.0) + (1 - xscale) * x_raw);
        float y = yscale * (float) Math.pow(y_raw, 7.0) + (1 - yscale) * y_raw;
        float z = -(zscale * (float) Math.pow(z_raw, 7.0) + (1 - zscale) * z_raw);

        //Mapping the joystick values on what the motors are doing
        float FL = -x + y + z;
        float FR = -x - y + z;
        float BL = x + y + z;
        float BR = x - y + z;

        //Normalizing the Motor Values
        float[] joystickVals = new float[]{FR, BL, FL, BR};
        float maxVal = Math.abs(FR);
        for (int i = 1; i < 4; i++) {
            if (Math.abs(joystickVals[i]) > maxVal) {
                maxVal = Math.abs(joystickVals[i]);
            }
        }

        if ((maxVal) > 1) {
            FR /= maxVal;
            BL /= maxVal;
            FL /= maxVal;
            BR /= maxVal;

        }


        //Set powers
        frontLeft.setPower(FL);
        backLeft.setPower(BL);
        frontRight.setPower(FR);
        backRight.setPower(BR);
        idle();
    }

    public void ManipulateHookLift() {
        if (gamepad2.dpad_up) {
            motor_lift_hook.setPower(-0.17);

        } else if (gamepad2.dpad_down) {
            motor_lift_hook.setPower(0.17);

        } else {
            motor_lift_hook.setPower(0);
            motor_lift_hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void ManipulateGrabbing() {
        float x = gamepad2.right_trigger;
        if (x > 0) {
            rightGrab.setPosition(.52);
            leftGrab.setPosition(.4);
            telemetry.addData("", "Servo Position is 1");

        } else if (gamepad2.right_bumper) {
            rightGrab.setPosition(.4);
            leftGrab.setPosition(.6);
            telemetry.addData("", "Servo Position is .5");
        }
    }
    public void liftGrabber() {

        telemetry.addData("grabber1 encoder", motor_lift_grabber1.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("grabber2 encoder", motor_lift_grabber2.getCurrentPosition());
        telemetry.update();

        double a = .85;

        double yvalue1 = a * Math.pow(gamepad2.left_stick_y, 3) + ((1 - a) * gamepad2.left_stick_y);
        double yvalue2 = a * Math.pow(gamepad2.right_stick_y, 3) + ((1 - a) * gamepad2.right_stick_y);
//reset encoders
        //experiment and find the encoder count for the outwards position of the claw
        //have a scaled joystick value that slowly increases the encoder position up to the point

        if (gamepad2.left_stick_y > 0.1) {
            motor_lift_grabber1.setPower(0.5);
        }
        else if (gamepad2.left_stick_y < -0.1)
            motor_lift_grabber1.setPower(-0.5);
        else motor_lift_grabber2.setPower(0);


        if (gamepad2.right_stick_y > 0.1){
            motor_lift_grabber2.setPower(0.5);
        }
        else if (gamepad2.right_stick_y < -0.1) {
            motor_lift_grabber2.setPower(-0.5);
        }
        else motor_lift_grabber2.setPower(0);

        if(motor_lift_grabber1.getCurrentPosition() < -426){
            motor_lift_grabber1.setTargetPosition(0);
            motor_lift_grabber2.setTargetPosition(0);
        }
        if (motor_lift_grabber2.getTargetPosition() < 127){
            motor_lift_grabber1.setTargetPosition(0);
            motor_lift_grabber2.setTargetPosition(0);
        }
        //... same with maxes**/



    }
}