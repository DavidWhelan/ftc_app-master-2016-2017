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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="BeaconPress", group="TechHogs")

public class BeaconPress extends OpMode
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
        robot.initLoop();
    }

    @Override
    public void start()
    {

    }

    @Override
    public void loop()
    {
        switch(caseSwitch)
        {
            case 0:
            {
                if(motor.followLine(robot.rangeSensor.getDistance(DistanceUnit.INCH) < 7.5))
                {
                    robot.timer.reset();
                    robot.setTurnPid();
                    caseSwitch++;
                }
                break;
            }
            case 1:
            {
                if(robot.timer.time() > 1.5) caseSwitch++;
                break;
            }
            case 2:
            {
                if(robot.colorRight.red() > robot.colorLeft.blue())
                {
                    motor.setPidDegrees(15);
                    caseSwitch = 3;
                }
                else
                {
                    motor.setPidDegrees(-15);
                    caseSwitch = 4;
                }
                robot.timer.reset();
                break;

            }
            case 3:
            {
                if(motor.turn())
                {
                    caseSwitch=5;
                }

                break;
            }

            case 4:
            {
                if(motor.turn())
                {
                    caseSwitch=5;
                }

                break;
            }


        }
        telemetry.addData("Debug", motor.debug);
        telemetry.addData("Debug2", motor.debug2);
        telemetry.addData("Range", robot.rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Gyro", robot.navx_device.getYaw());
        telemetry.addData("Case", caseSwitch);
    }

    @Override
    public void stop()
    {
        robot.navx_device.close();
    }

}
