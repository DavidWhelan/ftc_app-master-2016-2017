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


@Autonomous(name="Blue Beacon", group="TechHogs")
public class BlueBeaconScore extends OpMode
{
    RobotHardware robot = new RobotHardware();
    MotorControl motor = new MotorControl(robot);
    ElapsedTime caseTimer = new ElapsedTime();
    int caseSwitch = 0;
    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        motor.heading = 0;
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
        robot.navx_device.zeroYaw();
    }

    @Override
    public void loop()
    {
        switch(caseSwitch)
        {
            case 0:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                robot.stop_and_reset_encoder();
                robot.run_using_encoder();
                caseSwitch++;
                break;
            }

            case 1:
            {
                if(motor.driveWithEncoder(6, 1, "b"))
                {
                    caseSwitch++;
                }
                break;
            }

            case 2:
            {
                robot.setTurnPid();
                motor.setPidDegrees(45);
                caseSwitch++;
                break;
            }

            case 3:
            {
                if(motor.turn())
                {
                    caseSwitch++;
                }
                break;
            }

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
                motor.setPidDegrees(45);
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
                if(motor.backward(1, robot.rangeSensor.getDistance(DistanceUnit.INCH) < 13))
                {
                    caseSwitch++;
                }
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
                if(motor.right(.3, robot.colorRight.alpha() > 10))
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
                if(motor.followLine(robot.rangeSensor.getDistance(DistanceUnit.CM) < 11 ))
                {
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }

            case 15:
            {
                robot.setTurnPid();
                motor.setPidDegrees(0);
                robot.timer.reset();
                caseSwitch++;
                break;
            }

            case 16:
            {
                if(motor.turn())
                {
                    robot.setButtonLeftColorRead();
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }

            case 17:
            {
                if(robot.timer.time() > 1)
                {
                    caseSwitch++;
                }
                break;
            }

            case 18:
            {
                if((robot.beaconColor.red() > robot.beaconColor.blue()))
                {
                    robot.timer.reset();
                    caseSwitch = 19;
                }
                else
                {
                    robot.timer.reset();
                    caseSwitch = 20;
                }
                robot.setAngleButtonPress();
                break;
            }
//TODO If changing case statement number change these absolute changes
            case 19:
            {
                robot.setButtonRightPress();
                caseSwitch = 21;
                break;
            }

            case 20:
            {
                robot.setButtonLeftPress();
                caseSwitch = 21;
                break;
            }

            case 21:
            {
                if(robot.timer.time() > 2)
                {
                    robot.setButtonLeftInit();
                    robot.setButtonRightInit();
                    robot.setDrivePid();
                    motor.setPidDegrees(0);
                    caseSwitch++;
                }
                break;
            }

            case 22:
            {
                if(motor.forward(.5,robot.rangeSensor.getDistance(DistanceUnit.INCH) > 17))
                {
                    robot.setAngleBackward();
                    caseSwitch++;
                }
                break;
            }

            case 23:
            {
                robot.flyWheel.setPower(1);
                robot.timer.reset();
                caseSwitch++;
                break;
            }
            case 24:
            {
                robot.setTurnPid();
                motor.setPidDegrees(0);
                caseSwitch++;
                break;
            }

            case 25:
            {
                if(motor.turn())
                {
                    caseSwitch++;
                    break;
                }
            }

            case 26:
            {
                if(robot.timer.time() > 2 )
                {
                    robot.sweeper.setPower(1);
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }

            case 27:
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

            case 28:
            {
                robot.setTurnPid();
                motor.setPidDegrees(35);
                caseSwitch++;
                break;
            }

            case 29:
            {
                if(motor.turn())
                {
                    caseSwitch++;
                }
                break;
            }

            case 30:
            {
                robot.stop_and_reset_encoder();
                robot.run_using_encoder();
                robot.setDrivePid();
                motor.setPidDegrees(0);
                caseSwitch++;
                break;
            }

            case 31:
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