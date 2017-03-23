/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Sensor Test", group="TechHogs")
public class SensorTest extends OpMode
{
    RobotHardware robot = new RobotHardware();
    MotorControl motor = new MotorControl(robot);
    boolean stop = false;
    boolean release = false;
    int max = 2900;
    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {
        robot.navx_device.zeroYaw();
    }

    @Override
    public void loop()
    {
        //telemetry.addData("gyro", "Gyro: " + robot.gyro.getHeading());
        telemetry.addData("cLeft Red", "Color Left Red: " + robot.colorLeft.red());
        telemetry.addData("cLeft Blue","Color Left Blue: " + robot.colorLeft.blue());
        telemetry.addData("cLeft Green", "Color Left Green: " + robot.colorLeft.green());
        telemetry.addData("cLeft Alpha", "Color Left Alpha: " + robot.colorLeft.alpha());
        telemetry.addData("cRight Red", "Color Right Red: " + robot.colorRight.red());
        telemetry.addData("cRight Blue","Color Right Blue: " + robot.colorRight.blue());
        telemetry.addData("cRight Green", "Color Right Green: " + robot.colorRight.green());
        telemetry.addData("cRight Alpha", "Color Right Alpha: " + robot.colorRight.alpha());
        telemetry.addData("range", "Range: " + robot.rangeSensor.getDistance(DistanceUnit.CM));

        telemetry.addData("rangeSide", "Range Side: " + robot.sideRange.getDistance(DistanceUnit.CM));
        boolean test = true;
        I2cAddr testAddr = I2cAddr.create8bit(0x30);
        test = testAddr == robot.sideRange.getI2cAddress();
        telemetry.addData("rangeSide", "Range address: " + test);

        telemetry.addData("Beacon Red", robot.beaconColor.red());
        telemetry.addData("Beacon Blue", robot.beaconColor.blue());
        telemetry.addData("Beacon Green", robot.beaconColor.green());
        telemetry.addData("Beacon Alpha", robot.beaconColor.alpha());

        telemetry.addData("Gyro", robot.navx_device.getYaw());

        telemetry.addData("FR", robot.frontRight.getCurrentPosition());
        telemetry.addData("BR", robot.backRight.getCurrentPosition());
        telemetry.addData("FL", robot.frontLeft.getCurrentPosition());
        telemetry.addData("BL", robot.backLeft.getCurrentPosition());

        telemetry.addData("FW", robot.flyWheel.getCurrentPosition());

        if(gamepad1.a)
        {
            robot.flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            stop = true;
        }
        else if(gamepad1.y)
        {
            robot.flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stop = false;
        }

        if(stop)
        {
            robot.timer.reset();
        }
        else if(!stop)
        {
            telemetry.addData("Speed", robot.flyWheel.getCurrentPosition()/robot.timer.time());
        }
        if(gamepad1.a && release == false)
        {
            release = true;
            max -= 100 ;
        }
        else if(gamepad1.y && release == false)
        {
            release = true;
            max += 100;
        }
        if(!gamepad1.a && !gamepad1.y)
        {
            release = false;
        }
        robot.flyWheel.setMaxSpeed(max);

        telemetry.addData("Speed", max);
    }

    @Override
    public void stop()
    {

    }

}
