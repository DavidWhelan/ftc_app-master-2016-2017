package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Timer;

public class MotorControl
{
    private RobotHardware robot = new RobotHardware();
    private boolean setup = false;
    private boolean turnSetup = false;
    private ElapsedTime internalTimer = new ElapsedTime();
    private WallPID pid = new WallPID();
    private WallPIDSide pidSide = new WallPIDSide();
    private Timer pidTime = new Timer();

    public String debug = "";
    public String debug2 = "";
    public String debug3 = "";
    public double heading = 0.0;
    public double lightKp = 0.01;

    public double[] storedPower ={.5, .5, .5, .5,};

    public MotorControl(RobotHardware robot)
    {
        this.robot = robot;
    }

    public boolean turn ()
    {
        if (robot.yawPIDController.isNewUpdateAvailable(robot.yawPIDResult))
        {
            if (robot.yawPIDResult.isOnTarget())
            {
                stop();
                if(robot.timer.time()>0.2)
                {
                    return true;
                }
                return false;
            }
            else
            {
                double output = robot.yawPIDResult.getOutput();
                robot.frontLeft.setPower(output);
                robot.backLeft.setPower(output);
                robot.frontRight.setPower(-output);
                robot.backRight.setPower(-output);
                robot.timer.reset();
                return false;
            }
        }
        return false;
    }

    public boolean turn (double p, double i, double d)
    {
        if(!turnSetup)
        {
            robot.yawPIDController.setPID(p, i, d);
            turnSetup = true;
            return false;
        }
        if (robot.yawPIDController.isNewUpdateAvailable(robot.yawPIDResult))
        {
            if (robot.yawPIDResult.isOnTarget())
            {
                stop();
                if(robot.timer.time()>0.2)
                {
                    turnSetup = false;
                    return true;
                }
                return false;
            }
            else
            {
                double output = robot.yawPIDResult.getOutput();
                robot.frontLeft.setPower(output);
                robot.backLeft.setPower(output);
                robot.frontRight.setPower(-output);
                robot.backRight.setPower(-output);
                robot.timer.reset();
                return false;
            }
        }
        return false;
    }

    /*public boolean freezeTurn ()
    {
        if (robot.yawPIDController.isNewUpdateAvailable(robot.yawPIDResult))
        {
            if (robot.yawPIDResult.isOnTarget())
            {
                stop();
                if(robot.timer.time()>.5)
                {
                    return true;
                }
                return false;
            }
            else
            {
                double output = robot.yawPIDResult.getOutput();
                robot.frontLeft.setPower(0);
                robot.backLeft.setPower(0);
                robot.frontRight.setPower(-output);
                robot.backRight.setPower(-output);
                robot.timer.reset();
                return false;
            }
        }
        return false;
    }*/

    public boolean forward(double power, boolean stop)
    {
        if(stop)
        {
            stop();
            return true;
        }
        if (robot.yawPIDController.isNewUpdateAvailable(robot.yawPIDResult))
        {
            if (robot.yawPIDResult.isOnTarget())
            {
                robot.frontLeft.setPower(power);
                robot.backLeft.setPower(power);
                robot.frontRight.setPower(power);
                robot.backRight.setPower(power);
            }
            else
            {
                double output = robot.yawPIDResult.getOutput();
                robot.frontLeft.setPower(power + output);
                robot.backLeft.setPower(power + output);
                robot.frontRight.setPower(power - output);
                robot.backRight.setPower(power - output);
            }
        }
        return false;
    }

    public boolean backward(double power, boolean stop)
    {
        if(stop)
        {
            stop();
            return true;
        }
        if (robot.yawPIDController.isNewUpdateAvailable(robot.yawPIDResult))
        {
            if (robot.yawPIDResult.isOnTarget())
            {
                robot.frontLeft.setPower(-power);
                robot.backLeft.setPower(-power);
                robot.frontRight.setPower(-power);
                robot.backRight.setPower(-power);
            }
            else
            {
                double output = robot.yawPIDResult.getOutput();
                robot.frontLeft.setPower(-power + output);
                robot.backLeft.setPower(-power + output);
                robot.frontRight.setPower(-power - output);
                robot.backRight.setPower(-power - output);
            }
        }
        return false;
    }

    public boolean left(double power, boolean stop)
    {
        if(stop)
        {
            stop();
            return true;
        }
        if (robot.yawPIDController.isNewUpdateAvailable(robot.yawPIDResult))
        {
            if (robot.yawPIDResult.isOnTarget())
            {
                robot.frontLeft.setPower(-power);
                robot.backLeft.setPower(power);
                robot.frontRight.setPower(power);
                robot.backRight.setPower(-power);
            }
            else
            {
                double output = robot.yawPIDResult.getOutput();
                robot.frontLeft.setPower(-power + output);
                robot.backLeft.setPower(power + output);// + .3);
                robot.frontRight.setPower(power - output);
                robot.backRight.setPower(-power - output);// - .3);
            }
        }
        return false;
    }

    public boolean right(double power, boolean stop)
    {
        if(stop)
        {
            stop();
            return true;
        }
        if (robot.yawPIDController.isNewUpdateAvailable(robot.yawPIDResult))
        {
            if (robot.yawPIDResult.isOnTarget())
            {
                robot.frontLeft.setPower(power);
                robot.backLeft.setPower(-power);
                robot.frontRight.setPower(-power);
                robot.backRight.setPower(power);
            }
            else
            {
                double output = robot.yawPIDResult.getOutput();

                robot.frontLeft.setPower(power + output);
                robot.backLeft.setPower(-power + output - .3 );
                robot.frontRight.setPower(-power - output);
                robot.backRight.setPower(power - output +.3);
            }
        }
        return false;
    }

    public boolean followLine(boolean stop)
    {
        double power = 0.3;
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

        robot.frontLeft.setPower(Range.clip(-powerRight, -1, 1));
        robot.backLeft.setPower(Range.clip(-powerRight, -1, 1));

        robot.frontRight.setPower(Range.clip(-powerLeft, -1, 1));
        robot.backRight.setPower(Range.clip(-powerLeft, -1, 1));

        return false;
    }

    public boolean driveWithEncoder(double distance, double power, String direction)
    {
        if(direction.equals("f"))
        {
            forward(power, false);
        }
        else
        {
            backward(power, false);
        }
        double ticksNeeded = distance * robot.TICKS;
        if(Math.abs(robot.frontRight.getCurrentPosition()) > ticksNeeded)
        {
            stop();
            return true;
        }
        return false;
    }

    public boolean followWallRight(double power, double minDist, double maxDist, boolean stop)
    {
        //If it constantly zigzags find a way to implement a time/distance so it goes past boundry
        if(robot.rangeSensor.getDistance(DistanceUnit.INCH) < minDist)
        {
            forward(.5, stop);

        }
        else if(robot.rangeSensor.getDistance(DistanceUnit.INCH) > maxDist)
        {
            backward(.5, stop);
        }
        else
        {
            right(power, stop);
        }
        return stop;
    }

    public boolean followWallLeft(double power, double minDist, double maxDist, boolean stop)
    {
        //If it constantly zigzags find a way to implement a time/distance so it goes past boundry
        if(robot.rangeSensor.getDistance(DistanceUnit.INCH) < minDist)
        {
            forward(.5, stop);
        }
        else if(robot.rangeSensor.getDistance(DistanceUnit.INCH) > maxDist)
        {
            backward(.5, stop);
        }
        else
        {
            left(power, stop);
        }
        return stop;
    }


    public double limit(double a)
    {
        return Math.min(Math.max(a, robot.MIN_MOTOR_OUTPUT_VALUE), robot.MAX_MOTOR_OUTPUT_VALUE);
    }

    public void stop()
    {
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }

    public void setPidDegrees(double point)
    {
        robot.yawPIDController.setSetpoint(point + heading);
        heading += point;
    }

    public void setTargetEncoder(int pos)
    {
        robot.frontLeft.setTargetPosition(pos);
        robot.backLeft.setTargetPosition(pos);
        robot.frontRight.setTargetPosition(pos);
        robot.backRight.setTargetPosition(pos);
    }
    public boolean wallPID(double kp, double kd, double power, double distance)
    {
        if(!setup)//Sets up the local variables ince
        {
            pid = new WallPID(kp, kd, power, distance, robot); //Create a new wall PD
            pidTime = new Timer(); // Create a new timer to run a process in a new thread
            pidTime.scheduleAtFixedRate(pid, 10, 10); //Schedule the PD to run every 10 milliseconds in a separate thread
            internalTimer.reset(); // Reset the timer for correct positions
            setup = true; // make sure we only do this once
        }

        if(pid.onTarget()) // If we are where we are supposed to be
        {
            if(internalTimer.time() >.5) // if we are here for half a second (we are not changing position) then clean up and continue
            {
                pidTime.cancel();
                pidTime.purge();
                setup = false;
                return true;
            }
            stop(); // Stop motors
        }
        else
        {
            //if we need to adjust, grab the adjustment power and apply to the motors
            internalTimer.reset(); // Reset because we are moving
            double newPower = pid.getReturnPower();
            forward(newPower, false); //  apply power to the four motors
        }
        return false;
    }

    public boolean wallPIDSide(double kp, double kd, double power, double distance)
    {
        if(!setup)//Sets up the local variables ince
        {
            pidSide = new WallPIDSide(kp, kd, power, distance, robot); //Create a new wall PD
            pidTime = new Timer(); // Create a new timer to run a process in a new thread
            pidTime.scheduleAtFixedRate(pidSide, 10, 10); //Schedule the PD to run every 10 milliseconds in a separate thread
            internalTimer.reset(); // Reset the timer for correct positions
            setup = true; // make sure we only do this once
        }

        if(pidSide.onTarget()) // If we are where we are supposed to be
        {
            if(internalTimer.time() >.5) // if we are here for half a second (we are not changing position) then clean up and continue
            {
                pidTime.cancel();
                pidTime.purge();
                setup = false;
                return true;
            }
            stop(); // Stop motors
        }
        else
        {
            //if we need to adjust, grab the adjustment power and apply to the motors
            internalTimer.reset(); // Reset because we are moving
            double newPower = pidSide.getReturnPower();
            left(newPower, false); //  apply power to the four motors
        }
        return false;
    }
}
    /*
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
    */
