package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

public class MotorControl
{
    private RobotHardware robot = new RobotHardware();
    private double Kp = .0625;

    public MotorControl(RobotHardware robot)
    {
        this.robot = robot;
    }
    //Note that the power left and right is relative to the side the robot is moving. Look that direction and you know left and right
    public void forward(double power, boolean stop)
    {
        double powerRight;
        double powerLeft;
        if(stop)
        {
            stop();
            robot.gyro.resetZAxisIntegrator();
            return;
        }

        powerLeft = (Range.clip(power + ((double) decodeHeading() * Kp), -1, 1));
        powerRight= (Range.clip(power - ((double) decodeHeading() * Kp), -1, 1));

        robot.frontLeft.setPower(powerLeft);
        robot.frontRight.setPower(powerRight);
        robot.backLeft.setPower(powerLeft);
        robot.backRight.setPower(powerRight);
    }

    public void backward(double power, boolean stop)
    {
        robot.frontLeft.setPower(-power);
        robot.frontRight.setPower(-power);
        robot.backLeft.setPower(-power);
        robot.backRight.setPower(-power);
    }

    public void left(double power, boolean stop)
    {
        robot.frontLeft.setPower(-power);
        robot.frontRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(-power);
    }

    public void right(double power, boolean stop)
    {
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(-power);
        robot.backLeft.setPower(-power);
        robot.backRight.setPower(power);
    }

    public boolean counterClockwise(double power, double degrees)
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
            power = power - power * (degreesTurned/degrees);
            if(degrees - degreesTurned < 45)
            {
                power = .05;
            }
            robot.frontLeft.setPower(-power);
            robot.frontRight.setPower(power);
            robot.backLeft.setPower(-power);
            robot.backRight.setPower(power);
            return false;
        }
    }

    public boolean clockwise(double power, double degrees)
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
            power = power - power * (degreesTurned/degrees);
            if(degrees - degreesTurned < 45)
            {
                power = .05;
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

    public int decodeHeading()
    {
        int degreeHeading = 0;
        if(robot.gyro.getHeading()>=180)
        {
            degreeHeading = -(360 - robot.gyro.getHeading());
        }
        else if(robot.gyro.getHeading() >= 0 && robot.gyro.getHeading() < 180)
        {
            degreeHeading = robot.gyro.getHeading();
        }

        return degreeHeading;
    }
}
