package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
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
    public DcMotor lifter = null;

    public Servo angleAdjust = null;
    public Servo feedControl = null;
    public Servo buttonPushRight = null;
    public Servo buttonPushLeft = null;

    public ModernRoboticsI2cColorSensor colorLeft = null;
    public ModernRoboticsI2cColorSensor colorRight = null;
    public ModernRoboticsI2cColorSensor beaconColor = null;
    public ModernRoboticsI2cRangeSensor rangeSensor = null;

    public ElapsedTime timer = null;

    public final double TICKS = 91;

    //**********************************************************************************************


    private final int NAVX_DIM_I2C_PORT = 0;
    public AHRS navx_device;
    public navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    public double TOLERANCE_DEGREES = 1.0;
    public final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    public final double MAX_MOTOR_OUTPUT_VALUE = 1.0;

    private final double TURN_YAW_PID_P = 0.1; //0.075
    private final double TURN_YAW_PID_I = 0.0; //0.003
    private final double TURN_YAW_PID_D = 0.35;//0.02

    private final double DRIVE_YAW_PID_P = 0.065;
    private final double DRIVE_YAW_PID_I = 0.0;
    private final double DRIVE_YAW_PID_D = 0.0;

    private final double SLIDE_YAW_PID_P = 0.04;
    private final double SLIDE_YAW_PID_I = 0.0;
    private final double SLIDE_YAW_PID_D = 0.2 ;

    public navXPIDController.PIDResult yawPIDResult;

    //**********************************************************************************************

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
        lifter = hwMap.dcMotor.get("lifter");

        feedControl = hwMap.servo.get("feed");
        buttonPushRight = hwMap.servo.get("buttonPushRight");
        buttonPushLeft = hwMap.servo.get("buttonPushLeft");
        angleAdjust = hwMap.servo.get("angle");

        colorLeft = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("cLeft");
        colorRight = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("cRight");
        beaconColor = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("beacon");

        beaconColor.enableLed(false);

        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");



        I2cAddr newAddress = I2cAddr.create8bit(0x42);
        colorLeft.setI2cAddress(newAddress);

        newAddress = I2cAddr.create8bit(0x44);
        beaconColor.setI2cAddress(newAddress);

        timer = new ElapsedTime();

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        sweeper.setDirection(DcMotor.Direction.REVERSE);
        lifter.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stop_and_reset_encoder();

        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //TODO set flyWheel max
        flyWheel.setMaxSpeed(2500);

        setButtonRightInit();
        setButtonLeftInit();
        setAngleInit();
        //setFeedInit();

        //******************************************************************************************

        navx_device = AHRS.getInstance(ahwMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(TURN_YAW_PID_P, TURN_YAW_PID_I, TURN_YAW_PID_D);
        yawPIDController.enable(true);

        yawPIDResult = new navXPIDController.PIDResult();

        //******************************************************************************************
    }

    public void initLoop()
    {
            navx_device.zeroYaw();
            yawPIDResult = new navXPIDController.PIDResult();
    }


    public void setTurnPid()
    {
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, 2);
        yawPIDController.setPID(TURN_YAW_PID_P, TURN_YAW_PID_I, TURN_YAW_PID_D);
    }

    public void setDrivePid()
    {
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, 1);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(DRIVE_YAW_PID_P, DRIVE_YAW_PID_I, DRIVE_YAW_PID_D);
    }

    public void setSlidePid()
    {
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, 3);
        yawPIDController.setPID(SLIDE_YAW_PID_P, SLIDE_YAW_PID_I, SLIDE_YAW_PID_D);
    }

    public void run_to_position()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void run_using_encoder()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void run_without_encoder()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop_and_reset_encoder()
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMaxSpeed(int speed)
    {
        frontLeft.setMaxSpeed(speed);
        frontRight.setMaxSpeed(speed);
        backLeft.setMaxSpeed(speed);
        backRight.setMaxSpeed(speed);
    }


    public void singleModeSet(DcMotor motor, DcMotor.RunMode mode)
    {
        motor.setMode(mode);
    }

    public void setFeedInit()
    {
        feedControl.setPosition(.94);
    }

    public void setFeedStop()
    {
        feedControl.setPosition(.94);
    }


    public void setButtonRightInit()
    {
        buttonPushRight.setPosition(0);
    }

    public void setButtonRightPress()
    {
        buttonPushRight.setPosition(1);
    }


    public void setButtonLeftInit()
    {
        buttonPushLeft.setPosition(1);
    }

    public void setButtonLeftPress()
    {
        buttonPushLeft.setPosition(0);
    }

    public void setButtonLeftColorRead()
    {
        buttonPushLeft.setPosition(.48);
    }


    //TODO set these to right values

    public void setAngleInit()
    {
        angleAdjust.setPosition(0);
    }

    public void setAngleForward()
    {
        angleAdjust.setPosition(0);
    }

    public void setAngleBackward()
    {
        angleAdjust.setPosition(.27);
    }

    public void setAngleButtonPress()
    {
        angleAdjust.setPosition(.1);
    }
}
