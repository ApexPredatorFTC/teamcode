package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Admin on 1/10/2018.
 */
@TeleOp(name = "LifterTest", group = "")
public class LifterTest extends LinearOpMode{
    DcMotor rightSpin;
    DcMotor leftSpin;
    DcMotor lifter;

    @Override
    public void runOpMode(){
        rightSpin = hardwareMap.get(DcMotor.class, "rightSpin");
        leftSpin = hardwareMap.get(DcMotor.class, "leftSpin");
        lifter = hardwareMap.get(DcMotor.class, "lifter");

        waitForStart();
        while(opModeIsActive()){
            if(gamepad2.y){
                leftSpin.setPower(-.4);
                rightSpin.setPower(.4);
            }
            else if(gamepad2.x){
                leftSpin.setPower(.2);
                rightSpin.setPower(-.2);
            }
            else if(gamepad2.a){
                leftSpin.setPower(.3);
                rightSpin.setPower(-.3);
            }
            else{
                leftSpin.setPower(0);
                rightSpin.setPower(0);
            }


            if(gamepad2.left_stick_y > 0){
                lifter.setPower(-.4);
            }
            else if(gamepad2.left_stick_y < 0){
                lifter.setPower(1);
            }
            else{
                lifter.setPower(0);
            }

            idle();

        }
    }
}
