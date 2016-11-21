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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Red Beacon", group="TechHogs")
public class RedBeaconScore extends OpMode
{
    RobotHardware robot = new RobotHardware();
    MotorControl motor = new MotorControl(robot);
    int caseSwitch = 0;
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
        robot.gyro.resetZAxisIntegrator();
    }

    @Override
    public void loop()
    {
        switch(caseSwitch)
        {
            case 0:
            {
                motor.forward(0.3);
                caseSwitch++;
                break;
            }

            case 1:
            {
                if(robot.rangeSensor.getDistance(DistanceUnit.CM) > 65)
                {
                    motor.stop();
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }

            case 2:
            {
                if(robot.timer.time() > 1.5)
                {
                    caseSwitch++;
                }
                break;
            }

            case 3:
            {
                if(motor.clockwise(1, 90))
                {
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }

            case 4:
            {
               if(robot.timer.time() > 1.5)
               {
                   caseSwitch++;
               }
                break;
            }

            case 5:
            {
                motor.backward(0.3);
                caseSwitch++;
                break;
            }

            case 6:
            {
                if(robot.rangeSensor.getDistance(DistanceUnit.CM) < 35)
                {
                    motor.stop();
                    caseSwitch++;
                }
                break;
            }
        }
        telemetry.addData("Debug", motor.debug);
        telemetry.addData("Debug2", motor.debug2);
        telemetry.addData("Range", robot.rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Gyro", robot.gyro.getHeading());
    }

    @Override
    public void stop()
    {

    }

}
