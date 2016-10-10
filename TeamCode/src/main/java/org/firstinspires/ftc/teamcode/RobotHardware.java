package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by david on 9/30/16.
 */
public class RobotHardware
{
    HardwareMap hwMap = null;
    public DcMotor frontRight = null;
    public DcMotor frontLeft = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;
    public ModernRoboticsI2cGyro gyro = null;
    public ModernRoboticsI2cColorSensor colorLeft = null;
    public ModernRoboticsI2cColorSensor colorRight = null;
    public ElapsedTime timer = null;

    public RobotHardware()
    {

    }

    public void init(HardwareMap ahwMap)
    {
        hwMap = ahwMap;
        frontRight = hwMap.dcMotor.get("frontRight");
        frontLeft = hwMap.dcMotor.get("frontLeft");
        backRight = hwMap.dcMotor.get("backRight");
        backLeft = hwMap.dcMotor.get("backLeft");
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
        colorLeft = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("cLeft");
        colorRight = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("cRight");

        I2cAddr newAddress = I2cAddr.create8bit(0x42);
        colorLeft.setI2cAddress(newAddress);

        timer = new ElapsedTime();

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        gyro.calibrate();
    }

}
