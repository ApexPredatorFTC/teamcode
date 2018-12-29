package org.firstinspires.ftc.teamcode;

import android.support.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareDeviceHealth;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.TimestampedData;

/**
 * Created by Admin on 9/30/2017.
 */

@TeleOp(name = "color sensor test", group = "")
public class RevSensorTest extends LinearOpMode{

    public I2cDevice colorSensor;

    public I2cDeviceSynch colorSensorReader;

    byte[] colorCache;
    public static final I2cAddr c1Addr = I2cAddr.create8bit(0x40);
    public static final int c1StartReg = 0x04;
    public static final int c1ReadLength = 26;
    public static final int c1CmdReg = 0x03;

    public static final int c1PassiveCmd = 0x01;
    public static final int c1ActiveCmd = 0x00;

    @Override
    public void runOpMode(){

        colorSensor = hardwareMap.i2cDevice.get("colorSensor");

        colorSensorReader = new I2cDeviceSynchImpl(colorSensor, c1Addr, false);

        colorSensorReader.engage();
        colorSensorReader.write8(c1CmdReg, c1PassiveCmd);

        waitForStart();
        while(opModeIsActive()){
            colorSensorReader.write8(c1CmdReg, c1PassiveCmd);
            colorCache = colorSensorReader.read(c1StartReg, c1ReadLength);

            telemetry.addData("Color #",  (colorCache[0] & 0xff));
            telemetry.addData("Red", (colorCache[1] & 0xff));
            telemetry.addData("Green", (colorCache[2] & 0xff));
            telemetry.addData("Blue", (colorCache[3] & 0xff));
            telemetry.addData("White", (colorCache[4] & 0xff));
            telemetry.addData("Color Index Number", (colorCache[5] & 0xff));
            telemetry.addData("Red Index", (colorCache[6] & 0xff)) ;
            telemetry.addData("Green Index", (colorCache[7] & 0xff));
            telemetry.addData("Blue Index", (colorCache[8] & 0xff));
            telemetry.addData("Undefined", (colorCache[9] & 0xff));
            telemetry.addData("Red LSB", (colorCache[10] & 0xff));
            telemetry.addData("Green LSB", (colorCache[12] & 0xff));
            telemetry.addData("Blue LSB", (colorCache[14] & 0xff));
            telemetry.addData("White LSB", (colorCache[16] & 0xff));
            telemetry.addData("Norm Red LSB", (colorCache[18] & 0xff));
            telemetry.addData("Norm Green LSB", (colorCache[20] & 0xff));
            telemetry.addData("Norm Blue LSB", (colorCache[22] & 0xff));
            telemetry.addData("Norm White LSB", (colorCache[24] & 0xff));
            telemetry.update();

        }



    }
}
