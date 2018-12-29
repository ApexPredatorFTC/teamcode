package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Created by Admin on 2/4/2018.
 */
@TeleOp
public class TeleTest extends LinearOpMode {
    BNO055IMU imu;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor rightSpin;
    DcMotor leftSpin;
    Orientation angles;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    Servo ColorServo;
    Servo DownServo;
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_INCH = 115.555;
    static final double P_DRIVE_COEFF = .01;
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.008;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        rightSpin = hardwareMap.get(DcMotor.class, "rightSpin");
        leftSpin = hardwareMap.get(DcMotor.class, "leftSpin");
        // lifter = hardwareMap.get(DcMotor.class, "lifter");
        ColorServo = hardwareMap.get(Servo.class, "ColorServo");
        DownServo = hardwareMap.get(Servo.class, "DownServo");
        sensorColor = hardwareMap.get(ColorSensor.class, "js");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "js");
        sensorColor.enableLed(true);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (opModeIsActive()) {
            int blueValue = sensorColor.blue();
            int redValue = sensorColor.red();
            telemetry.addData("Color Values", "Blue: " + blueValue + " Red: " + redValue);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Distance Sensor:", sensorDistance);
            telemetry.addData("Down Servo:", DownServo);
            telemetry.update();
        }
    }
}
