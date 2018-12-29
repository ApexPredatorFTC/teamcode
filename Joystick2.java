package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Admin on 10/25/2017.
 */

@TeleOp(name = "Joystick2", group = "")
public class Joystick2 extends LinearOpMode {


    DcMotor motor_arm;
    Servo Servo_L;
    Servo Servo_R;
    float x = 0;
    float y = 0;


    @Override
    public void runOpMode(){

  //      // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        Servo_L = hardwareMap.get(Servo.class, "left_hand");
        Servo_R = hardwareMap.get(Servo.class, "right_hand");
        motor_arm = hardwareMap.get(DcMotor.class, "arm");
        waitForStart();
        while(opModeIsActive()) {
            motor_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            x = gamepad2.right_trigger;
            if (x > 0) {
                Servo_R.setPosition(1);
                telemetry.addData("", "Servo Position is 1");
            } else if (gamepad2.right_bumper) {
                Servo_R.setPosition(0);
                telemetry.addData("", "Servo Position is 0");
            }
            y = gamepad2.left_trigger;
            if (y > 0) {
                Servo_L.setPosition(0);
                telemetry.addData("", "Servo Position is 0");
            } else if (gamepad2.left_bumper) {
                Servo_L.setPosition(1);
                telemetry.addData("", "Servo Position is 1");
            }
            motor_arm.setPower(gamepad2.left_stick_y*.5);
        }
        motor_arm.setPower(0);

    }
}
