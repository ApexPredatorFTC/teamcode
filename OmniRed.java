package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Admin on 12/30/2017.
 */
@Autonomous(name = "OmniRed", group = "Sensor")
public class OmniRed extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    ModernRoboticsI2cGyro gyro = null;

    static final double COUNTS_PER_MOTOR_REV = 1100;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double P_TURN_COEFF = 0.017;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.10;


    @Override
    public void runOpMode() throws InterruptedException{
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();


        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
            idle();
        }
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        gyro.resetZAxisIntegrator();
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void Move(double distance, double speed, double direction) throws InterruptedException {
        if (opModeIsActive()) {
            direction = direction * Math.PI / 180;

            telemetry.addLine("0.1");
            telemetry.update();

            gyro.resetZAxisIntegrator();

            int frontLeftPosition = (int) (distance * COUNTS_PER_INCH * Math.cos(direction) - frontLeft.getCurrentPosition());
            int frontRightPosition = (int) (distance * COUNTS_PER_INCH * Math.sin(direction) - frontRight.getCurrentPosition());
            int backLeftPosition = (int) (distance * COUNTS_PER_INCH * Math.sin(direction) - backLeft.getCurrentPosition());
            int backRightPosition = (int) (distance * COUNTS_PER_INCH * Math.cos(direction) - backRight.getCurrentPosition());

            frontLeft.setTargetPosition(-frontLeftPosition);
            frontRight.setTargetPosition(-frontRightPosition);
            backLeft.setTargetPosition(-backLeftPosition);
            backRight.setTargetPosition(-backRightPosition);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addLine("0.2");
            telemetry.update();

            while (opModeIsActive() &&
                    ((frontLeft.isBusy() && backRight.isBusy()) || (backLeft.isBusy() && frontRight.isBusy()))) {
                telemetry.addLine("0.3");
                // adjust relative speed based on heading error.
                double error = getError(0);
                double steer = getSteer(error, P_DRIVE_COEFF);

                double frontLeftSpeed = (speed * Math.cos(direction) + steer);
                double frontRightSpeed = (speed * Math.sin(direction)) + steer;
                double backRightSpeed = (speed * Math.cos(direction)) - steer;
                double backLeftSpeed = (speed * Math.sin(direction)) - steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                double max1 = Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed));
                double max2 = Math.max(Math.abs(backRightSpeed), Math.abs(backLeftSpeed));
                double max = Math.max(max1, max2);
                if (max > 1.0) {
                    frontLeftSpeed /= max;
                    frontRightSpeed /= max;
                    backRightSpeed /= max;
                    backLeftSpeed /= max;

                }

                frontLeft.setPower(-frontLeftSpeed);
                frontRight.setPower(-frontRightSpeed);
                backLeft.setPower(-backLeftSpeed);
                backRight.setPower(-backRightSpeed);
                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d:%7d:%7d", frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition);
                telemetry.addData("Actual", "%7d:%7d:%7d:%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f:%5.2f:%5.2f", frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }
            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Stop all motion;
            frontRight.setPower(0);
            backLeft.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);

        }
    }

}
