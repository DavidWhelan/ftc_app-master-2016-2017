package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.TimerTask;

/**
 * Created by David on 12/21/2016.
 */

public class WallPIDSide extends TimerTask //extends a task so it can be run in a seperate thread in order to prevent a slowdown throwing the derivitave off
{
    //Constants used in the equation
    private double kp = 0.05;
    private double kd = 0.05;
    private double setPoint = 0;
    private double basePower = 0.5;
    //for the derivative
    private double lastError = 0;
    //power to return back
    private double returnPower = 0;
    //Prevent reads when it is being written by this thread
    private boolean safeRead = false;
    //Robot hardware
    private RobotHardware robot;
    //Default constructor
    public WallPIDSide()
    {

    }
    //Constructor
    public WallPIDSide(double p, double d, double power, double setPoint, RobotHardware robot)
    {
        this.setPoint = setPoint;
        this.robot = robot;
        setPower(power);
        setConstants(p, d);
        lastError = robot.sideRange.getDistance(DistanceUnit.CM);
    }
    //Allows access to class members and change the constants
    public void setConstants(double p, double d)
    {
        kp = p;
        kd = d;
    }
    //Change base power
    public void setPower(double power)
    {
        basePower = power;
    }
    //Where we want to be
    public  void setSetPoint(double setPoint)
    {
        this.setPoint = setPoint;
    }
    //If we are not writing to it we can read it
    public double getReturnPower()
    {
        if(safeRead)return returnPower;
        else return basePower;
    }
    //if we are withing the acceptable error range return true
    public boolean onTarget()
    {
        return (Math.abs(setPoint - robot.sideRange.getDistance(DistanceUnit.CM))<= 1);
    }
    //Code that runs in the seperate thread to mataion correct timing
    public void run()
    {
        //calculate the difference between position and setpoint
        double error = setPoint - robot.sideRange.getDistance(DistanceUnit.CM);
        //calculate p
        double p = Range.clip(((kp * error)), -1, 1);
        //calculate d
        double d = Range.clip((kd *((error - lastError)/10)), -1, 1);
        //write the error out for the d calc
        lastError = error;
        //write the power the robot needs to be and make sure that nothing tries to read it
        safeRead = false;
        returnPower = basePower * (p+d);
        safeRead = true;
    }

}
