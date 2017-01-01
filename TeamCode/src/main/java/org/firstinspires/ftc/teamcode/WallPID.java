package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.TimerTask;

/**
 * Created by David on 12/21/2016.
 */

public class WallPID extends TimerTask
{
    private double kp = 0.05;
    private double kd = 0.05;
    private double setPoint = 0;
    private double basePower = 0.5;
    //TODO::If weird behavior look here
    private double lastError = 0;

    private double returnPower = 0;

    private boolean safeRead = false;

    private RobotHardware robot;

    public WallPID()
    {

    }

    public WallPID(double p, double d, double power, double setPoint, RobotHardware robot)
    {
        this.setPoint = setPoint;
        this.robot = robot;
        setPower(power);
        setConstants(p, d);
        lastError = robot.rangeSensor.getDistance(DistanceUnit.CM);
    }

    public void setConstants(double p, double d)
    {
        kp = p;
        kd = d;
    }

    public void setPower(double power)
    {
        basePower = power;
    }

    public  void setSetPoint(double setPoint)
    {
        this.setPoint = setPoint;
    }

    public double getReturnPower()
    {
        if(safeRead)return returnPower;
        else return basePower;
    }

    public boolean onTarget()
    {
        return (Math.abs(setPoint - robot.rangeSensor.getDistance(DistanceUnit.CM))<= 1);
    }

    public void run()
    {
        double error = setPoint - robot.rangeSensor.getDistance(DistanceUnit.CM);

        double p = Range.clip(((kp * error)), -1, 1);
        double d = Range.clip((kd *((error - lastError)/10)), -1, 1);

        lastError = error;

        safeRead = false;
        returnPower = basePower * (p+d);
        safeRead = true;
    }

}
