package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

/**
 * Created by Admin on 1/31/2018.
 */
@Autonomous
public class AutoNew extends LinearOpMode {
    private double errorTurn = 0;

    boolean on = true;
    BNO055IMU imu;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor motor_lift_hook;
    DcMotor motor_lift_grabber1;
    DcMotor motor_lift_grabber2;
    //DcMotor lifter;
    // State used for updating telemetry
    Orientation angles;


    static final double COUNTS_PER_INCH = 115.555;
    static final double P_DRIVE_COEFF = .01;
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.008;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AQDpaUn/////AAABmQYuuMBIJUjRnYNrOAhhlqSCYBK588RWSoiOpDW7hW2M0Dc1fyQPqH5hfRzTayd66If0v0gPXVAWewkOmijLnafz2OzLaex4oF0WyI14x70yRv6vseyhDI3jKfJBKdAr77j8hmKdYlxo1ArsMlU9TAkNQ1NqQ7YxUe76FzWHnW7GdMCbVoQCOMGr4eyUurMDCpskxb2ZTr8prLm8YY9mPk6MhpsYdHKbtgUPliAHOCdT7RxeQH1+m5W4Nr1CnejOP7alXm+nPfU/UMVi/quZhZr/rWdAoP5A+UDHnpXGPXYZM41H67fNDyMnG32yQIJlE3bE+aR/n9Yi2hKsQS2ztO3HRn0vq5bHRe8e+MUvCMWQ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        motor_lift_hook = hardwareMap.dcMotor.get("motor_lift_hook");
        motor_lift_grabber1 = hardwareMap.dcMotor.get("motor_lift_grabber1");
        motor_lift_grabber2 = hardwareMap.dcMotor.get("motor_lift_grabber2");

        motor_lift_grabber1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_lift_grabber2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_lift_hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_lift_grabber1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_lift_grabber2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_lift_hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_lift_grabber1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_lift_grabber2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_lift_hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters imuparameters = new BNO055IMU.Parameters();
        imuparameters.loggingEnabled = true;
        imuparameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuparameters);


        //composeTelemetry();
        initVuforia();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && on == true) {
            /*telemetry.addLine();
            telemetry.addData("Hook: ", motor_lift_hook.getCurrentPosition());
            telemetry.update();*/

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
        if(tfod !=null)
        {
            tfod.shutdown();
        }
        
    }




    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }



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
