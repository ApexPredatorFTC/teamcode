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
@TeleOp(name = "ArmTest", group = "")
public class ArmTest extends LinearOpMode {
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

        /*while (opModeIsActive()) {
            telemetry.addLine();
            telemetry.addData("Encoder Count: ", motor_lift_grabber2.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Joystick left Y Value: ", gamepad2.left_stick_y);
            //telemetry.update();
            telemetry.addLine();
            telemetry.addData("Joystick right Y Value: ", gamepad2.right_stick_y);
            telemetry.update();


            if (gamepad2.left_stick_y > .1) {
                // if (motor_lift_grabber1.getCurrentPosition() > 90) {
                //motor_lift_grabber1.Motor_power_max=-.5;
                motor_lift_grabber1.setPower(-0.1);
                // } else {
                //     motor_lift_grabber1.setPower(0.6);
                // }
            } else if (gamepad2.left_stick_y < -.1) {
                //  if (motor_lift_grabber1.getCurrentPosition() < 50) {
                motor_lift_grabber1.setPower(0.1);
                //  } else {
                //     motor_lift_grabber1.setPower(0.6);
                //  }
            } else {
                motor_lift_grabber1.setPower(0);
                motor_lift_grabber1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }

            if (gamepad2.right_stick_y > .1) {
                ///    if (motor_lift_grabber2.getCurrentPosition() > 90) {
                motor_lift_grabber2.setPower(0.025);
                //    } else {
                //        motor_lift_grabber2.setPower(0.6);
                //    }

            } else if (gamepad2.right_stick_y < -0.1) {
                //    if (motor_lift_grabber2.getCurrentPosition() < 50) {
                motor_lift_grabber2.setPower(-0.025);
                //    } else {
                //       motor_lift_grabber2.setPower(0.6);
                //  }
            } else {
                motor_lift_grabber2.setPower(0);
                motor_lift_grabber2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
        }*/

        int bottomPos = 0;
        while (opModeIsActive()) {
            motor_lift_grabber1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad2.right_stick_y > 0) {
                bottomPos += 10;
            }
            if (gamepad2.right_stick_y < 0) {
                bottomPos -= 10;
            }

            if (gamepad2.a){
                bottomPos = -180;

            }
            if (gamepad2.b){
                bottomPos = -30;
            }

            if(bottomPos < -180){
                bottomPos = -180;
            }

            telemetry.addLine();
            telemetry.addData("Encoder 1: ", motor_lift_grabber1.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Encoder 2: ", motor_lift_grabber2.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Bottom pos: ", bottomPos);
            telemetry.addLine();
            telemetry.addData("Right Stick: ",gamepad2.right_stick_y);
            telemetry.update();


            motor_lift_grabber1.setTargetPosition(bottomPos);


            if (Math.abs(motor_lift_grabber1.getCurrentPosition() - bottomPos) < 20) {
                motor_lift_grabber1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor_lift_grabber1.setPower(.05);
            }
            else{

                double grabberSpeed = (Math.abs(motor_lift_grabber1.getCurrentPosition() - bottomPos) * 0.0002);
                motor_lift_grabber1.setPower(grabberSpeed);
            }

            //telemetry.addData("Grabber Speed", grabberSpeed);
            //telemetry.update();

        }

    }
}