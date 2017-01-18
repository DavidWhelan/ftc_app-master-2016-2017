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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Red BeaconIm", group="TechHogs")
@Disabled
public class RedBeaconScoreIm extends OpMode
{
    RobotHardware robot = new RobotHardware();
    MotorControl motor = new MotorControl(robot);
    ElapsedTime caseTimer = new ElapsedTime();
    int caseSwitch = 4;
    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        motor.heading = -20;
    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {
        robot.run_using_encoder();
        robot.setMaxSpeed(2400);
        //robot.navx_device.zeroYaw();
    }

    @Override
    public void loop()
    {
        switch(caseSwitch)
        {

            case 4:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                robot.stop_and_reset_encoder();
                robot.run_using_encoder();
                caseSwitch++;
                break;
            }

            case 5:
            {
                if(motor.driveWithEncoder(42, 1, "b"))
                {
                    caseSwitch++;
                }
                break;
            }

            case 6:
            {
                robot.setTurnPid();
                motor.setPidDegrees(-70);
                caseSwitch++;
                break;
            }

            case 7:
            {
                if(motor.turn())
                {
                    caseSwitch++;
                }
                break;
            }

            case 8:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                caseSwitch++;
                break;
            }

            case 9:
            {
                caseSwitch++;
                break;
            }

            case 10:
            {
                robot.setSlidePid();
                motor.setPidDegrees(0);
                caseSwitch++;
                break;
            }

            case 11:
            {
                if(motor.left(.3, robot.colorLeft.alpha() > 10))
                {
                    caseSwitch++;
                }
                break;


            }

            case 12:
            {
                robot.setTurnPid();
                robot.TOLERANCE_DEGREES  = 1;
                motor.setPidDegrees(0);
                caseSwitch++;
                break;
            }

            case 13:
            {
                if(motor.turn())
                {
                    robot.setDrivePid();
                    motor.setPidDegrees(0);
                    robot.TOLERANCE_DEGREES = 2;
                    caseSwitch++;
                }
                break;
            }

            case 14:
            {
                if(motor.followLine(robot.rangeSensor.getDistance(DistanceUnit.CM) < 10 ))
                {
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }

            case 15:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                caseSwitch++;
                break;
            }

            case 16:
            {
                if(motor.wallPID(.05, .05, 1, 10))
                {
                    caseSwitch++;
                }
                break;
            }

            case 17:
            {
                robot.setTurnPid();
                motor.setPidDegrees(0);
                robot.timer.reset();
                caseSwitch++;
                break;
            }

            case 18:
            {
                if(motor.turn())
                {
                    robot.setButtonLeftColorRead();
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }

            case 19:
            {
                if(robot.timer.time() > .25)
                {
                    caseSwitch++;
                }
                break;
            }

            case 20:
            {
                if((robot.beaconColor.red() > robot.beaconColor.blue()))
                {
                    robot.timer.reset();
                    caseSwitch = 21;
                }
                else
                {
                    robot.timer.reset();
                    caseSwitch = 22;
                }
                robot.setAngleButtonPress();
                break;
            }
//TODO If changing case statement number change these absolute changes
            case 21:
            {
                robot.setButtonLeftPress();
                caseSwitch = 23;
                break;
            }

            case 22:
            {
                robot.setButtonRightPress();
                caseSwitch = 23;
                break;
            }

            case 23:
            {
                if(robot.timer.time() > 1)
                {
                    robot.setButtonLeftInit();
                    robot.setButtonRightInit();
                    robot.setDrivePid();
                    motor.setPidDegrees(0);
                    caseSwitch++;
                }
                break;
            }

            case 24:
            {
                if(motor.wallPID(.05, .05, 1, 43))
                {
                    robot.setAngleBackward();
                    caseSwitch++;
                }
                break;
            }

            case 25:
            {
                robot.flyWheel.setPower(1);
                robot.timer.reset();
                caseSwitch++;
                break;
            }
            case 26:
            {
                robot.setTurnPid();
                motor.setPidDegrees(0);
                caseSwitch++;
                break;
            }

            case 27:
            {
                if(motor.turn())
                {
                    caseSwitch++;
                    break;
                }
            }

            case 28:
            {
                if(robot.timer.time() > 1 )
                {
                    robot.sweeper.setPower(1);
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }

            case 29:
            {
                if(robot.timer.time() > 2)
                {
                    robot.flyWheel.setPower(0);
                    robot.sweeper.setPower(0);
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }

            case 30:
            {
                robot.setTurnPid();
                motor.setPidDegrees(-35);
                caseSwitch++;
                break;
            }

            case 31:
            {
                if(motor.turn())
                {
                    caseSwitch++;
                }
                break;
            }
            //This is so we can push
            case 32:
            {
                robot.stop_and_reset_encoder();
                robot.run_using_encoder();
                robot.setDrivePid();
                motor.setPidDegrees(0);
                caseSwitch++;
                break;
            }

            case 33:
            {
                if(motor.driveWithEncoder(27, 1, "f"))
                {
                    caseSwitch++;
                }
                break;
            }
        }
    }

    @Override
    public void stop()
    {
        robot.navx_device.close();
    }

}