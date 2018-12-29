package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Admin on 12/16/2017.
 */
@Autonomous(name = "OmniColorTest", group = "")
public class OmniColorTest extends LinearOpMode{
    ColorSensor cSensor;
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    public void runOpMode(){
        cSensor = hardwareMap.colorSensor.get("cSensor");

        waitForStart();

        while(opModeIsActive()){
            Color.RGBToHSV((int) (cSensor.red() * SCALE_FACTOR),
                    (int) (cSensor.green() * SCALE_FACTOR),
                    (int) (cSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            if((cSensor.red() * SCALE_FACTOR) > (cSensor.blue() * SCALE_FACTOR)) {

            }
            else if ((cSensor.red() * SCALE_FACTOR) < (cSensor.blue() * SCALE_FACTOR)) {

            }

            telemetry.addData("Red" ,(cSensor.red()*SCALE_FACTOR));
            telemetry.addData("Blue" ,(cSensor.blue()*SCALE_FACTOR));
            telemetry.addData("Green" ,(cSensor.green()*SCALE_FACTOR));
            telemetry.update();
        }


    }
    public void detectColor() {
        Color.RGBToHSV((int) (cSensor.red() * SCALE_FACTOR),
                (int) (cSensor.green() * SCALE_FACTOR),
                (int) (cSensor.blue() * SCALE_FACTOR),
                hsvValues);
    }
}
