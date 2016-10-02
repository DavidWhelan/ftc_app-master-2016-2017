package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

        timer = new ElapsedTime();

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        gyro.calibrate();
    }

}
