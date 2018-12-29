package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by BRAIN on 9/27/2017.
 */
@TeleOp(name = "TeleOpTest", group = "")

public class Tele_Op_Test extends LinearOpMode{
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;

    @Override
    public void runOpMode(){
        motor1 = hardwareMap.get(DcMotor.class,"left_drive");
        motor2 = hardwareMap.get(DcMotor.class, "right_drive");
        motor3 = hardwareMap.get(DcMotor.class, "left_drive2");
        motor4 = hardwareMap.get(DcMotor.class, "right_drive2");

        waitForStart();
        while(opModeIsActive()){

            motor1.setPower(-gamepad1.left_stick_y);
            motor2.setPower(gamepad1.right_stick_y);
            motor3.setPower (-gamepad1.left_stick_y);
            motor4.setPower (gamepad1.right_stick_y);
        }
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
}
