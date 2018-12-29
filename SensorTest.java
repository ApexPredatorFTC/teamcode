package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Admin on 10/25/2017.
 */

@TeleOp(name = "SensorTest", group = "")
public class SensorTest extends LinearOpMode {
    DcMotor motor_left;
    DcMotor motor_right;
    DcMotor motor_center;
    DcMotor motor_arm;

    Servo Servo_L;
    Servo Servo_R;

    @Override
    public void runOpMode(){

        // 1080 encoder counts is one foot for both motors
        motor_left = hardwareMap.get(DcMotor.class, "left");
        motor_right = hardwareMap.get(DcMotor.class,"right");
        motor_center = hardwareMap.get(DcMotor.class, "center");
        motor_arm = hardwareMap.get(DcMotor.class, "arm");

        Servo_L = hardwareMap.get(Servo.class, "left_hand");
        Servo_R = hardwareMap.get(Servo.class, "right_hand");

        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        while(opModeIsActive()) {

            telemetry.addData("Left Encoder", motor_left.getCurrentPosition());
            telemetry.addData("Right Encoder", motor_right.getCurrentPosition());
            telemetry.addData("Center Encoder", motor_center.getCurrentPosition());
            telemetry.addData("Arm Encoder", motor_arm.getCurrentPosition());
            telemetry.update();
       }

        motor_left.setPower(0);
        motor_right.setPower(0);
        motor_center.setPower(0);
        motor_arm.setPower(0);


    }
}
