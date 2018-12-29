package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/**
 * Created by Admin on 1/29/2018.
 */
@TeleOp(name = "SomethingIntelligent", group = "Sensor")
public class SomethingIntelligent extends LinearOpMode{
    BNO055IMU imu;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    //DcMotor rightSpin;
    //DcMotor leftSpin;
    //DcMotor lifter;
    // State used for updating telemetry
    Orientation angles;

    @Override public void runOpMode() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        //rightSpin = hardwareMap.get(DcMotor.class, "rightSpin");
        //leftSpin = hardwareMap.get(DcMotor.class, "leftSpin");
        // lifter = hardwareMap.get(DcMotor.class, "lifter");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.log().setCapacity(12);
        telemetry.log().add("");
        telemetry.log().add("Please refer to the calibration instructions");
        telemetry.log().add("contained in the Adafruit IMU calibration");
        telemetry.log().add("sample opmode.");
        telemetry.log().add("");
        telemetry.log().add("When sufficient calibration has been reached,");
        telemetry.log().add("press the 'A' button to write the current");
        telemetry.log().add("calibration data to a file.");
        telemetry.log().add("");

        // We are expecting the IMU to be attached to an I2C port on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();
        telemetry.log().add("Waiting for start...");

        // Wait until we're told to go
        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        telemetry.log().add("...started...");

        while (opModeIsActive()) {
//Scaling the joystick inputs for sensitivity
            float y_raw = gamepad1.left_stick_y;
            float x_raw = gamepad1.left_stick_x;
            float z_raw = gamepad1.right_stick_x;
            float xscale = (float) 0.85;
            float yscale = (float) 0.85;
            float zscale = (float) 0.85;
            float x = -(xscale * (float) Math.pow(x_raw, 3.0) + (1 - xscale) * x_raw);
            float y = yscale * (float) Math.pow(y_raw, 3.0) + (1 - yscale) * y_raw;
            float z = -(zscale * (float) Math.pow(z_raw, 3.0) + (1 - zscale) * z_raw);

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
            /*if (gamepad2.y) {
                leftSpin.setPower(-.4);
                rightSpin.setPower(.4);
            } else if (gamepad2.x) {
                leftSpin.setPower(.2);
                rightSpin.setPower(-.2);
            } else if (gamepad2.a) {
                leftSpin.setPower(.3);
                rightSpin.setPower(-.3);
            } else {
                leftSpin.setPower(0);
                rightSpin.setPower(0);
            }*/


            // if(gamepad2.left_stick_y > 0){
            //else if(gamepad2.left_stick_y < 0){
            //    lifter.setPower(1);
            frontLeft.setPower(FL);
            backLeft.setPower(BL);
            frontRight.setPower(FR);
            backRight.setPower(BR);
            //else{
            //lifter.setPower(0);
            //}


            //Set powers

            idle();
            telemetry.update();

        }
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
    }




    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

