package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

public class RobotHardware
{
    HardwareMap hwMap = null;
    public DcMotor frontRight = null;
    public DcMotor frontLeft = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;
    public DcMotor sweeper = null;
    public DcMotor flyWheel = null;
    public DcMotor rightLifter = null;
    public DcMotor leftLifter = null;
    public Servo angleAdjust = null;
    public Servo feedControl = null;
    public ModernRoboticsI2cGyro gyro = null;
    public ModernRoboticsI2cColorSensor colorLeft = null;
    public ModernRoboticsI2cColorSensor colorRight = null;
    public ModernRoboticsI2cColorSensor beaconColor = null;
    public ModernRoboticsI2cRangeSensor rangeSensor = null;
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
        flyWheel = hwMap.dcMotor.get("flyWheel");
        sweeper = hwMap.dcMotor.get("sweeper");

        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
        colorLeft = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("cLeft");
        colorRight = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("cRight");
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");



        I2cAddr newAddress = I2cAddr.create8bit(0x42);
        colorLeft.setI2cAddress(newAddress);

        timer = new ElapsedTime();

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        sweeper.setDirection(DcMotor.Direction.REVERSE);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        gyro.calibrate();
    }

}
