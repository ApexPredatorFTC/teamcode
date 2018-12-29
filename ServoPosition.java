package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.Locale;

/**
 * Created by Admin on 1/31/2018.
 */
@TeleOp
public class ServoPosition extends LinearOpMode {
    private double errorTurn = 0;

    boolean on=true;
    BNO055IMU imu;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor rightSpin;
    DcMotor leftSpin;
    //DcMotor lifter;
    // State used for updating telemetry
    Orientation angles;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    Servo ColorServo;
    Servo DownServo;
    ColorSensor sensorColorR;
    DistanceSensor sensorDistanceR;
    Servo ColorServoR;
    Servo DownServoR;
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    static final double COUNTS_PER_INCH = 115.555;
    static final double P_DRIVE_COEFF = .01;
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.008;

    private double Rpos=0;
    private double Rpos2=0;
    private double pos=0;
    private double pos2=0;
    @Override public void runOpMode() {
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
        sensorColorR = hardwareMap.get(ColorSensor.class, "js2");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "js2");
        ColorServoR = hardwareMap.get(Servo.class, "ColorServo2");
        DownServoR = hardwareMap.get(Servo.class, "DownServo2");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters imuparameters = new BNO055IMU.Parameters();
        imuparameters.loggingEnabled = true;
        imuparameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuparameters);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AWckC9P/////AAAAGQAbrzfWpEF1qYwCpiUOr7RVSfW6jrhfxzTHAroEzCmsYWamxL3Ouv5nHuVtyVDiu0lzb1kWGfkCs5wTzyQIIiKDNYP70491X/5gnZgwXxEj2EkIM5u/ek+G14ilZlDIeaCh6nMXglX8Q/NVXfK1ox33KR68lYMIBWcJdLmLbKPUWylHXXSvFLHpUSvyvpXKwgovzlxtmXTCcYqvMoVwTBWcHPazaupqXBxp4aeF1w9xvIr5GYdq5kzzL7Vs/AUH5QU5PG/0UFLlM+frXCdJcWzLK0u8X+CyNIJjMskZBfBW+9bGC0uynIP4gP94HmGdviuoTtPuWXUTJrvCsrPHpD3rXDt6o6BIiNGQs28a3W2p";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        //composeTelemetry();

        DownServo.setPosition(1);
        ColorServo.setPosition(0);
        DownServoR.setPosition(1);
        ColorServoR.setPosition(0);
        relicTrackables.activate();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && on == true) {
            DownServoR.setPosition(Rpos);
            ColorServoR.setPosition(Rpos2);
            DownServo.setPosition(pos);
            ColorServo.setPosition(pos2);

            telemetry.addData("DownServoR", Rpos);
            telemetry.addData("ColorServoR", Rpos2);
            telemetry.addData("DownServo", pos);
            telemetry.addData("ColorServo", pos2);
            telemetry.update();

            if(gamepad1.dpad_up){
                Rpos+=.001;
            }
            else if(gamepad1.dpad_down){
                Rpos-=.001;
            }
            if(gamepad1.dpad_right  ){
                Rpos2+=.001;
            }
            else if(gamepad1.dpad_left){
                Rpos2-=.001;
            }
            if(gamepad1.y){
                pos+=.001;
            }
            else if(gamepad1.a){
                pos-=.001;
            }

            if(gamepad1.x){
                pos2+=.001;
            }
            else if(gamepad1.b){
                pos-=.001;
            }

            if(Rpos > 1){
                Rpos = 1;
            }

            if(Rpos < 0){
                Rpos = 0;
            }

            if(Rpos2 > 1){
                Rpos2 = 1;
            }

            if(Rpos2 < 0){
                Rpos2 = 0;
            }

            if(pos > 1){
                pos = 1;
            }

            if(pos < 0){
                pos2 = 0;
            }

            if(pos2 > 1){
                pos2 = 1;
            }

            if(pos2 < 0){
                pos2 = 0
                ;
            }

        }}
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle + angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void Move(double distance, double speed, double direction) {
        if (opModeIsActive()) {
            direction = direction * Math.PI / 180;

            //Reset Encoder Counts 0
            resetEncoders();
            double yDist = Math.cos(direction) * distance;
            double xDist = Math.sin(direction)* distance;

            resetEncoders();

            double ySpeed = Math.cos(direction) * speed;
            double xSpeed = Math.sin(direction)* speed;

            double var = 1/Math.sqrt(2);
            double frontLeftDistOffset = -(var * xDist) - (var*yDist);
            double backLeftDistOffset = (var * xDist) - (var*yDist);
            double backRightDistOffset = (var * xDist) + (var*yDist);
            double frontRightDistOffset = -(var * xDist) + (var*yDist);


            int frontLeftPosition = (int)(frontLeft.getCurrentPosition()+(frontLeftDistOffset*COUNTS_PER_INCH));
            int backLeftPosition = (int)(backLeft.getCurrentPosition()+(backLeftDistOffset*COUNTS_PER_INCH));
            int backRightPosition = (int)(backRight.getCurrentPosition()+(backRightDistOffset*COUNTS_PER_INCH));
            int frontRightPosition = (int)(frontRight.getCurrentPosition()+(frontRightDistOffset*COUNTS_PER_INCH));

            frontLeft.setTargetPosition(frontLeftPosition);
            frontRight.setTargetPosition(frontRightPosition);
            backLeft.setTargetPosition(backLeftPosition);
            backRight.setTargetPosition(backRightPosition);


            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (opModeIsActive() &&
                    (frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy() && frontRight.isBusy())) {

                // adjust relative speed based on heading error.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double error = getError(errorTurn);
                double steer = getSteer(error, P_DRIVE_COEFF);

                double frontLeftSpeed = -(var * xSpeed) - (var*ySpeed) -steer;
                double backLeftSpeed = (var * xSpeed) - (var*ySpeed) - steer;
                double backRightSpeed = (var * xSpeed) + (var*ySpeed) - steer;
                double frontRightSpeed  = -(var * xSpeed) + (var*ySpeed) - steer;

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

                frontLeft.setPower(frontLeftSpeed);
                frontRight.setPower(frontRightSpeed);
                backLeft.setPower(backLeftSpeed);
                backRight.setPower(backRightSpeed);
                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d:%7d:%7d", frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition);
                telemetry.addData("Actual", "%7d:%7d:%7d:%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());

                telemetry.addData("Speed", "%5.2f:%5.2f:%5.2f:%5.2f", frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
                telemetry.addData("Heading", angles.firstAngle);
                telemetry.update();

            }

            // Stop all motion;
            frontRight.setPower(0);
            backLeft.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        }
    }

    public void resetEncoders(){
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void EncoderMove(double distance, double speed, double direction) {
        if (opModeIsActive()) {
            //Convert direction to radians
            direction = direction * Math.PI / 180;

            //Reset Encoder Counts 0
            resetEncoders();

            double yDist = Math.cos(direction) * distance;
            double xDist = Math.sin(direction)* distance;

            double ySpeed = Math.cos(direction) * speed;
            double xSpeed = Math.sin(direction)* speed;

            double var = 1/Math.sqrt(2);
            double frontLeftDistOffset = -(var * xDist) - (var*yDist);
            double backLeftDistOffset = (var * xDist) - (var*yDist);
            double backRightDistOffset = (var * xDist) + (var*yDist);
            double frontRightDistOffset = -(var * xDist) + (var*yDist);

            double frontLeftSpeed = -(var * xSpeed) - (var*ySpeed);
            double backLeftSpeed = (var * xSpeed) - (var*ySpeed);
            double backRightSpeed = (var * xSpeed) + (var*ySpeed);
            double frontRightSpeed  = -(var * xSpeed) + (var*ySpeed);

            int frontLeftPosition = (int)(frontLeft.getCurrentPosition()+(frontLeftDistOffset*COUNTS_PER_INCH));
            int backLeftPosition = (int)(backLeft.getCurrentPosition()+(backLeftDistOffset*COUNTS_PER_INCH));
            int backRightPosition = (int)(backRight.getCurrentPosition()+(backRightDistOffset*COUNTS_PER_INCH));
            int frontRightPosition = (int)(frontRight.getCurrentPosition()+(frontRightDistOffset*COUNTS_PER_INCH));

            frontLeft.setTargetPosition(frontLeftPosition);
            frontRight.setTargetPosition(frontRightPosition);
            backLeft.setTargetPosition(backLeftPosition);
            backRight.setTargetPosition(backRightPosition);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (opModeIsActive() &&
                    (frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy() && frontRight.isBusy())) {


                // Normalize speeds if any one exceeds +/- 1.0;
                double max1 = Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed));
                double max2 = Math.max(Math.abs(backRightSpeed), Math.abs(backLeftSpeed));
                double max = Math.max(max1, max2);

                //0.7 is motor max speed for turning
                if (max > .7) {
                    frontLeftSpeed = 0.7*(frontLeftSpeed/max);
                    frontRightSpeed = 0.7*(frontRightSpeed/max);
                    backLeftSpeed = 0.7*(backRightSpeed/max);
                    backRightSpeed = 0.7*(backLeftSpeed/max);


                }

                frontLeft.setPower(frontLeftSpeed);
                frontRight.setPower(frontRightSpeed);
                backLeft.setPower(backLeftSpeed);
                backRight.setPower(backRightSpeed);
                // Display drive status for the driver.

                telemetry.addData("Target", "%7d:%7d:%7d:%7d", frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition);
                telemetry.addData("Actual", "%7d:%7d:%7d:%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());

                telemetry.addData("Speed", "%5.2f:%5.2f:%5.2f:%5.2f", frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
                telemetry.update();

            }

            // Stop all motion;
            frontRight.setPower(0);
            backLeft.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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
                        return formatAngle(angles.angleUnit, -angles.firstAngle);
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

    public void GyroTurn(double degrees) {
        if (opModeIsActive()) {

            errorTurn += degrees ;

            double turnError;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degrees = degrees - angles.firstAngle;

            boolean turn = false;

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive() && (turn == false)) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                turnError = getError(degrees);



                if (Math.abs(turnError) <= HEADING_THRESHOLD) {
                    turn = true;
                } else {

                    double steer = getSteer(turnError, P_TURN_COEFF);

                    double frontLeftSpeed = -(steer);
                    double frontRightSpeed = -(steer);
                    double backRightSpeed = -(steer);
                    double backLeftSpeed = -(steer);

                    double max1 = Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed));
                    double max2 = Math.max(Math.abs(backRightSpeed), Math.abs(backLeftSpeed));
                    double max = Math.max(max1, max2);
                    if (max > 1.0) {
                        frontLeftSpeed /= max;
                        frontRightSpeed /= max;
                        backRightSpeed /= max;
                        backLeftSpeed /= max;

                    }

                    frontLeft.setPower(frontLeftSpeed);
                    frontRight.setPower(frontRightSpeed);
                    backLeft.setPower(backLeftSpeed);
                    backRight.setPower(backRightSpeed);

                    telemetry.addData("heading", angles.firstAngle);
                    telemetry.addData("turnerror", turnError);
                    telemetry.update();

                    idle();

                }
            }
            frontRight.setPower(0);
            backLeft.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);


            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void Intake() {
        leftSpin.setPower(-.2);
        rightSpin.setPower(.2);
    }
    public void Outtake() {
        leftSpin.setPower(.4);
        rightSpin.setPower(-.4);
    }
    public void stopOuttake() {
        leftSpin.setPower(0);
        rightSpin.setPower(0);
    }
    public void Sensor() {
        sleep(500);
        int blueValue = sensorColor.blue();
        int redValue = sensorColor.red();
        sleep(1000);
        blueValue = sensorColor.blue();
        redValue = sensorColor.red();
        //composeTelemetry();
        telemetry.addData("Blue: ", blueValue);
        telemetry.addData("Red: ", redValue);
        telemetry.update();

        // Show the elapsed game time and wheel power.
        if(redValue > blueValue){
            ColorServo.setPosition(.7);
            sleep(500);//Forwards
        }
        else if (blueValue > redValue){
            ColorServo.setPosition(.2);
            sleep(500); //Backwards

        }
    }
    public void DropTheServo(){
        DownServo.setPosition(1);
    }
    public void LiftTheServo(){
        DownServo.setPosition(0);
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
