package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MotorControl
{
    private RobotHardware robot = new RobotHardware();
    private double driveKp = .035;
    private double lightKp = .004;
    private double powerMinmum = 0.8;
    public String debug = "";
    public String debug2 = "";
    public String debug3 = "";

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
        double powerLeft = (Range.clip(power + ((double) decodeHeading() * driveKp), -1, 1));
        double powerRight= (Range.clip(power - ((double) decodeHeading() * driveKp), -1, 1));

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

    public boolean counterClockwise(double power, double degrees, ElapsedTime timer)
    {
        int degreesTurned = 0;

        if(robot.gyro.getHeading() >= 180 && robot.gyro.getHeading() <= 360)
        {
            degreesTurned = 360 - robot.gyro.getHeading();
        }
        if (degreesTurned >= degrees)
        {
            //robot.gyro.resetZAxisIntegrator();
            debug = String.valueOf(degreesTurned)  + ":Time:" + timer.time();
            stop();
            debug2 = String.valueOf(degreesTurned)  + ":Time:" + timer.time();
            return true;
        }
        else
        {
            if(power < powerMinmum)
            {
                power = powerMinmum;
            }
            robot.frontLeft.setPower(-power);
            robot.frontRight.setPower(power);
            robot.backLeft.setPower(-power);
            robot.backRight.setPower(power);
            return false;
        }
    }

    public boolean clockwise(double power, double degrees, ElapsedTime timer)
    {
        int degreesTurned = 0;

        if(robot.gyro.getHeading() >= 0 && robot.gyro.getHeading() < 180)
        {
            degreesTurned = robot.gyro.getHeading();
        }
        if (degreesTurned >= degrees)
        {
            //robot.gyro.resetZAxisIntegrator();

            stop();
            debug2 = String.valueOf(degreesTurned)  + ":Time:" + timer.time();
            return true;
        }
        else
        {
            debug = String.valueOf(degreesTurned)  + ":Time:" + timer.time();
            power = power - power * (degreesTurned/degrees);
            if(power < powerMinmum)
            {
                power = powerMinmum;
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

    public boolean followLine(boolean stop)
    {
        double power = 0.2;
        double powerLeft;
        double powerRight;
        double difference;
        double colorLeftAlpha = -robot.colorLeft.alpha();
        double colorRightAlpha = robot.colorRight.alpha();

        if(stop)
        {
            stop();
            return true;
        }

        if(robot.colorLeft.alpha() > 10 && robot.colorRight.alpha() <= 10)
        {
            difference = colorLeftAlpha/5;
        }
        else if(robot.colorLeft.alpha() <= 10 && robot.colorRight.alpha() >10)
        {
            difference = colorRightAlpha/5;
        }
        else
        {
            difference = 0;
        }
        powerLeft = power + (difference * lightKp);
        powerRight = power - (difference * lightKp);

        robot.frontLeft.setPower(Range.clip(powerLeft, -1, 1));
        robot.backLeft.setPower(Range.clip(powerLeft, -1, 1));

        robot.frontRight.setPower(Range.clip(powerRight, -1, 1));
        robot.backRight.setPower(Range.clip(powerRight, -1, 1));

        return false;
    }

    public void followWallRight(double power, double minDist, double maxDist, boolean stop)
    {
        if(stop)
        {
            stop();
            return;
        }
        //If it constantly zigzags find a way to implement a time/distance so it goes past boundry
        if(robot.rangeSensor.getDistance(DistanceUnit.CM) < minDist)
        {
            backward(power/2);
        }
        else if(robot.rangeSensor.getDistance(DistanceUnit.CM) > maxDist)
        {
            forward(power/2);
        }
        else
        {
            right(power);
        }
    }

    public void followWallLeft(double power, double minDist, double maxDist, boolean stop)
    {
        if(stop)
        {
            stop();
            return;
        }
        //If it constantly zigzags find a way to implement a time/distance so it goes past boundry
        if(robot.rangeSensor.getDistance(DistanceUnit.CM) < minDist)
        {
            backward(power/2);
        }
        else if(robot.rangeSensor.getDistance(DistanceUnit.CM) > maxDist)
        {
            forward(power/2);
        }
        else
        {
            left(power);
        }
    }
}
