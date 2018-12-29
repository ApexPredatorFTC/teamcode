package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

/**
 * Created by Admin on 11/29/2017.
 */
@Autonomous(name = "OtherCornerBlue", group = "")
public class OtherCornerBlue extends LinearOpMode {
    boolean on = true;

    DcMotor motor_left;
    DcMotor motor_right;

    DcMotor motor_arm;
    Servo Servo_L;
    Servo Servo_R;
    float x = 0;
    float y = 0;


    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        motor_left = hardwareMap.get(DcMotor.class, "left");
        motor_right = hardwareMap.get(DcMotor.class, "right");


        Servo_L = hardwareMap.get(Servo.class, "left_hand");
        Servo_R = hardwareMap.get(Servo.class, "right_hand");
        motor_arm = hardwareMap.get(DcMotor.class, "arm");

        motor_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AWckC9P/////AAAAGQAbrzfWpEF1qYwCpiUOr7RVSfW6jrhfxzTHAroEzCmsYWamxL3Ouv5nHuVtyVDiu0lzb1kWGfkCs5wTzyQIIiKDNYP70491X/5gnZgwXxEj2EkIM5u/ek+G14ilZlDIeaCh6nMXglX8Q/NVXfK1ox33KR68lYMIBWcJdLmLbKPUWylHXXSvFLHpUSvyvpXKwgovzlxtmXTCcYqvMoVwTBWcHPazaupqXBxp4aeF1w9xvIr5GYdq5kzzL7Vs/AUH5QU5PG/0UFLlM+frXCdJcWzLK0u8X+CyNIJjMskZBfBW+9bGC0uynIP4gP94HmGdviuoTtPuWXUTJrvCsrPHpD3rXDt6o6BIiNGQs28a3W2p";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
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
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        closeClaw();
        waitForStart();
        relicTrackables.activate();
        while (opModeIsActive() && on == true) {


            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {


                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;

                    telemetry.addData("tX", tX);
                    telemetry.addData("tY", tY);
                    telemetry.addData("tZ", tZ);

                    telemetry.addData("rX", rX);
                    telemetry.addData("rY", rY);
                    telemetry.addData("rZ", rZ);

                }
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    driveForward(3, 0.4);
                    sleep(1000);
                    otherTurn(-98, 0.4);
                    sleep(1000);
                    driveForward(.8, 0.4);
                    sleep(500);
                    openClaw();
                    sleep(500);
                    driveForward(-0.3, 0.5);
                    resetEncoders();
                    on = false;
                }

                else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    driveForward(3, 0.4);
                    sleep(1000);
                    otherTurn(-81.5, 0.4);
                    sleep(1000);
                    driveForward(1, 0.4);
                    sleep(500);
                    openClaw();
                    sleep(500);
                    driveForward(-0.3, 0.5);
                    resetEncoders();
                    on = false;
                }

                else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    driveForward(3.1, 0.4);
                    sleep(1000);
                    otherTurn(-69, 0.4);
                    sleep(1000);
                    driveForward(1, 0.4);
                    sleep(500);
                    openClaw();
                    sleep(500);
                    driveForward(-0.3, 0.5);
                    resetEncoders();
                    on = false;
                }

            }

        }
    }

    public void driveForward(double feetDist, double power) {
        double distance = feetDist * 1080;
        motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor_left.setTargetPosition((int) distance + motor_left.getCurrentPosition());
        motor_right.setTargetPosition((-(int) distance + motor_right.getCurrentPosition()));

        motor_left.setPower(power);
        motor_right.setPower(-power);


        while (opModeIsActive() && (motor_left.isBusy() && motor_right.isBusy())) {

            telemetry.addData("Encoder Target", distance);
            telemetry.addData("Left Encoder", motor_left.getCurrentPosition());
            telemetry.addData("Right Encoder", motor_right.getCurrentPosition());
            telemetry.update();

        }

        motor_left.setPower(0);
        motor_right.setPower(0);

    }

    public void turn(double degrees, double power) {
        //-2500 to both motors is 180 degree turn
        //-13.8 encoder counts is one degree
        motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double position = (-13.8 * degrees);
        motor_left.setTargetPosition((int) position + motor_left.getCurrentPosition());
        motor_right.setTargetPosition((int) (position) + motor_right.getCurrentPosition());


        if (position < 0) {
            motor_left.setPower(power);
            motor_right.setPower(power);
        } else if (position > 0) {
            motor_left.setPower(-power);
            motor_right.setPower(-power);
        }
    }
    public void otherTurn(double degrees, double power) {
        //-2500 to both motors is 180 degree turn
        //-13.8 encoder counts is one degree
        motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double position = (-20.76 * degrees);


        if (position < 0) {
            motor_left.setTargetPosition((int) position + motor_left.getCurrentPosition());

            motor_left.setPower(power);

            while (opModeIsActive() && (motor_left.isBusy())) {

                telemetry.addData("Encoder Target", position);
                telemetry.addData("Left Encoder", motor_left.getCurrentPosition());
                telemetry.addData("Right Encoder", motor_right.getCurrentPosition());
                telemetry.update();

            }
        }
        else if(position > 0) {
            motor_right.setTargetPosition((int) position + motor_right.getCurrentPosition());

            motor_right.setPower(power);

            while (opModeIsActive() && (motor_right.isBusy())) {

                telemetry.addData("Encoder Target", position);
                telemetry.addData("Left Encoder", motor_left.getCurrentPosition());
                telemetry.addData("Right Encoder", motor_right.getCurrentPosition());
                telemetry.update();

            }
        }

        motor_left.setPower(0);
        motor_right.setPower(0);
    }

    public void resetEncoders() {
        motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(200);


    }

    public void closeClaw() {
        Servo_L.setPosition(.5);
        Servo_R.setPosition(.5);


        sleep(500);
    }

    public void openClaw() {
        Servo_R.setPosition(1);
        Servo_L.setPosition(0);
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}