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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="PID", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class PIDTest extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    MotorControl motor;

    double setPoint = 25;

    double lastError;

    double kp = .05;
    double kd = .05;
    double ki = .001;

    double i = 0;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);

        motor = new MotorControl(robot);
        lastError = setPoint - robot.rangeSensor.getDistance(DistanceUnit.CM);
        robot.run_using_encoder();
        robot.setDrivePid();
        motor.setPidDegrees(0);
    }
    @Override
    public void init_loop()
    {
    }
    @Override
    public void start()
    {
        runtime.reset();
    }
    @Override
    public void loop()
    {
        double power = .5;
        double error;

        error = setPoint - robot.rangeSensor.getDistance(DistanceUnit.CM);

        if(runtime.milliseconds() >= 10)
        {
            double p = Range.clip(((kp * error)), -1, 1);
            double d = Range.clip((kd *((error - lastError)/10)), -1, 1);
            i = Range.clip((ki *(i + (error * 10))), -1, 1);

            lastError = error;

            power  = power*(p + d);

            motor.forward(power, false);

            runtime.reset();
        }
        telemetry.addData("Sonic Distance", robot.rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Power", power);
    }

    @Override
    public void stop()
    {
        robot.navx_device.close();
    }

}
