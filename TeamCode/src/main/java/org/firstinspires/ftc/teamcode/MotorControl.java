package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

public class MotorControl
{
    private RobotHardware robot = new RobotHardware();
    private double driveKp = .0625;
    private int stoppingDegrees = 90;
    private double stopSpeed = 0.05;

    public MotorControl(RobotHardware robot)
    {
        this.robot = robot;
    }
    //Note that the power left and right is relative to the side the robot is moving. Look that direction and you know left and right
    public void forward(double power)
    {
        double powerLeft = (Range.clip(power - ((double) decodeHeading() * driveKp), -1, 1));
        double powerRight= (Range.clip(power + ((double) decodeHeading() * driveKp), -1, 1));

        robot.frontLeft.setPower(powerLeft);
        robot.frontRight.setPower(powerRight);
        robot.backLeft.setPower(powerLeft);
        robot.backRight.setPower(powerRight);
    }

    public void backward(double power)
    {
        double powerLeft = (Range.clip(power - ((double) decodeHeading() * driveKp), -1, 1));
        double powerRight= (Range.clip(power + ((double) decodeHeading() * driveKp), -1, 1));

        robot.frontLeft.setPower(-powerLeft);
        robot.frontRight.setPower(-powerRight);
        robot.backLeft.setPower(-powerLeft);
        robot.backRight.setPower(-powerRight);
    }

    public void left(double power)
    {
        double powerLeft = (Range.clip(power - ((double) decodeHeading() * driveKp), -1, 1));
        double powerRight= (Range.clip(power + ((double) decodeHeading() * driveKp), -1, 1));

        robot.frontLeft.setPower(-powerRight);
        robot.frontRight.setPower(powerRight);
        robot.backLeft.setPower(powerLeft);
        robot.backRight.setPower(-powerLeft);
    }

    public void right(double power)
    {
        double powerLeft = (Range.clip(power - ((double) decodeHeading() * driveKp), -1, 1));
        double powerRight= (Range.clip(power + ((double) decodeHeading() * driveKp), -1, 1));

        robot.frontLeft.setPower(powerLeft);
        robot.frontRight.setPower(-powerLeft);
        robot.backLeft.setPower(-powerRight);
        robot.backRight.setPower(powerRight);
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
            if(degrees - degreesTurned < stoppingDegrees)
            {
                power = stopSpeed;
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
            if(degrees - degreesTurned < stoppingDegrees)
            {
                power = stopSpeed;
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
