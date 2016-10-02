package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by david on 9/30/16.
 */
public class MotorControl
{
    private RobotHardware robot = new RobotHardware();

    public MotorControl(RobotHardware robot)
    {
        this.robot = robot;
    }

    public void forward(double power)
    {
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
    }

    public void backward(double power)
    {
        robot.frontLeft.setPower(-power);
        robot.frontRight.setPower(-power);
        robot.backLeft.setPower(-power);
        robot.backRight.setPower(-power);
    }

    public void left(double power)
    {
        robot.frontLeft.setPower(-power);
        robot.frontRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(-power);
    }

    public void right(double power)
    {
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(-power);
        robot.backLeft.setPower(-power);
        robot.backRight.setPower(power);
    }

    public boolean counterClockwise(double power, int degrees)
    {
        int degreesTurned = 0;

        if(robot.gyro.getHeading() >= 180 && robot.gyro.getHeading() <= 360)
        {
            degreesTurned = 360 - robot.gyro.getHeading();
        }
        if (degreesTurned >= degrees)
        {
            stop();
            robot.gyro.resetZAxisIntegrator();
            return true;
        }
        else
        {
            power = 1 - (degreesTurned/degrees);
            if(power < 0.3)
            {
                power = 0.3;
            }
            robot.frontLeft.setPower(-power);
            robot.frontRight.setPower(power);
            robot.backLeft.setPower(-power);
            robot.backRight.setPower(power);
            return false;
        }
    }

    public boolean clockwise(double power, int degrees)
    {
        int degreesTurned = 0;

        if(robot.gyro.getHeading() >= 0 && robot.gyro.getHeading() < 180)
        {
            degreesTurned = robot.gyro.getHeading();
        }
        if (degreesTurned >= degrees)
        {
            stop();
            robot.gyro.resetZAxisIntegrator();
            return true;
        }
        else
        {
            power = 1 - (degreesTurned/degrees);
            if(power < 0.3)
            {
                power = 0.3;
            }
            robot.frontLeft.setPower(power);
            robot.frontRight.setPower(-power);
            robot.backLeft.setPower(power);
            robot.backRight.setPower(-power);
            return false;
        }

    }

    public void stop()
    {
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

}
