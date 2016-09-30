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

    public void counterClockwise(double power)
    {
        robot.frontLeft.setPower(-power);
        robot.frontRight.setPower(power);
        robot.backLeft.setPower(-power);
        robot.backRight.setPower(power);
    }

    public void clockwise(double power)
    {
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(-power);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(-power);
    }
}
