package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

/**
 * Created by Admin on 12/9/2017.
 */

@TeleOp(name = "OmniTeleOP", group = "")
public class OmniTeleOP extends LinearOpMode{
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor rightSpin;
    DcMotor leftSpin;
    DcMotor lifter;
    Servo DownServo;
    Servo ColorServo;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    Servo DownServoR;
    Servo ColorServoR;
    ColorSensor sensorColorR;
    DistanceSensor sensorDistanceR;

    int liftLock = 0;

    public void runOpMode(){
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        rightSpin = hardwareMap.get(DcMotor.class, "rightSpin");
        leftSpin = hardwareMap.get(DcMotor.class, "leftSpin");
        lifter = hardwareMap.get(DcMotor.class, "lifter");

        DownServo = hardwareMap.get(Servo.class, "DownServo");
        ColorServo = hardwareMap.get(Servo.class, "ColorServo");
        sensorColor = hardwareMap.get(ColorSensor.class, "js");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ColorServoR = hardwareMap.get(Servo.class, "ColorServo2");
        sensorColorR = hardwareMap.get(ColorSensor.class, "js2");
        DownServoR = hardwareMap.get(Servo.class, "DownServo2");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DownServo.setPosition(.67);
        ColorServo.setPosition(0);
        DownServoR.setPosition(.2);
        ColorServoR.setPosition(1);

        sensorColor.enableLed(false);
        waitForStart();

        while(opModeIsActive()){
            DownServo.setPosition(.67);
            ColorServo.setPosition(0);
            DownServoR.setPosition(.2);
            ColorServoR.setPosition(1);
            //Scaling the joystick inputs for sensitivity
            float y_raw = gamepad1.left_stick_y* .9f;
            float x_raw = gamepad1.left_stick_x* .9f;
            float z_raw = gamepad1.right_stick_x * 0.85f;
            float xscale = (float) 0.75;
            float yscale = (float) 0.75;
            float zscale = (float) 0.65;
            float x = -(xscale*(float)Math.pow(x_raw, 7.0) + (1-xscale)*x_raw);
            float y = yscale*(float)Math.pow(y_raw, 7.0) + (1-yscale)*y_raw;
             float z = -(zscale*(float)Math.pow(z_raw, 7.0) + (1-zscale)*z_raw);

            //Mapping the joystick values on what the motors are doing
            float FL = x+y+z;
            float FR = x-y+z;
            float BL = -x+y+z;
            float BR = -x-y+z;

            //Normalizing the Motor Values
            float [] joystickVals = new float[] {FR, BL, FL, BR};
            float maxVal = Math.abs(FR);
            for (int i = 1; i<4; i++)
            {
                if(Math.abs(joystickVals[i])>maxVal){
                    maxVal = Math.abs(joystickVals[i]);
                }
            }

            if ((maxVal)>1){
                FR/=maxVal;
                BL/=maxVal;
                FL/=maxVal;
                BR/=maxVal;

            }
            if(gamepad2.y){
                leftSpin.setPower(-.6);
                rightSpin.setPower(.6);
            }
            else if(gamepad2.x){
                leftSpin.setPower(1);
                rightSpin.setPower(-1);
            }
            else if(gamepad2.a){
                leftSpin.setPower(.5);
                rightSpin.setPower(-.5);
            }
            else{
                leftSpin.setPower(0);
                rightSpin.setPower(0);
            }

            if(gamepad2.dpad_up){
                liftLock =1;
            }
            else if(gamepad2.dpad_right){
                liftLock = 0;
            }

            if(gamepad2.left_stick_y > 0){
                lifter.setPower(-.2);
            }
            else if(gamepad2.left_stick_y < 0){
                lifter.setPower(.7);
            }
            else{
                if(liftLock == 0) {
                    lifter.setPower(0);
                }
                else{
                    lifter.setPower(.1);
                }
            }

            if(gamepad2.right_trigger == 1){
                DownServo.setPosition(1);
                ColorServo.setPosition(0);
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
