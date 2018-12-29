package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cIrSeekerSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Admin on 2/10/2018.
 */
@TeleOp
public class BrainBoard extends LinearOpMode {

    DcMotor m1;
    DcMotor m2;

    Servo s1;

    ModernRoboticsI2cIrSeekerSensorV3 irSensor;
    ModernRoboticsI2cGyro gyroSensor;
    ModernRoboticsI2cColorSensor colorSensor;
    ModernRoboticsAnalogOpticalDistanceSensor odsSensor;

    double y_raw;
    double y_raw1;

    public void runOpMode(){

        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m1");
        s1 = hardwareMap.get(Servo.class, "s1");
        irSensor = hardwareMap.get(ModernRoboticsI2cIrSeekerSensorV3.class, "irSensor");
        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyroSensor");
        odsSensor = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "odsSensor");

        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()){
            if(y_raw < 0){
                m1.setPower(.25);
            }
            else if(y_raw > 0){
                m1.setPower(-.25);
            }
            if(y_raw1 < 0){
                m2.setPower(.25);
            }
            if(y_raw1 > 0){
                m2.setPower(-.25);
            }


            s1.setPosition(.4);
            if(colorSensor.blue() > 10 || colorSensor.red() > 10 )
            {
                s1.setPosition(1);
                sleep(4000);
            }

            telemetry.addData("Gyro Heading ::", gyroSensor.getHeading());



        }

    }
}
