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
@TeleOp(name = "DriveTrainTest", group = "")
public class DriveTrainTest extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;


    public void runOpMode() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive()) {
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
            float FL = x + y + z;
            float FR = x - y + z;
            float BL = -x + y + z;
            float BR = -x - y + z;

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
    }
}