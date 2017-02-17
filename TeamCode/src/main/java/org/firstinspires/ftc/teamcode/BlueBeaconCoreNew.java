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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Blue Beacon Improved", group="TechHogs")
//@Disabled
public class BlueBeaconCoreNew extends OpMode
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
        robot.setMaxSpeed(1800);
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
                if(motor.driveWithEncoder(12, 1, "f"))
                {
                    caseSwitch++;
                }
                break;
            }

            case 2:
            {
                robot.setTurnPid();
                motor.setPidDegrees(-22);
                caseSwitch++;
                break;
            }

            case 3:
            {
                if(motor.turn())
                {
                    caseSwitch++;
                    robot.timer.reset();
                    robot.setAngleBackward();
                    robot.flyWheel.setPower(1);
                }
                break;
            }

            case 4:
            {
                if(robot.timer.time() > 1.3)
                {
                    robot.sweeper.setPower(1);
                    caseSwitch++;
                    robot.timer.reset();
                }
                break;
            }

            case 5:
            {
                if(robot.timer.time() > 2.3)
                {
                    robot.sweeper.setPower(0);
                    robot.flyWheel.setPower(0);
                    robot.setAngleForward();
                    caseSwitch++;
                }
                break;
            }

            case 6:
            {
                robot.setTurnPid();
                motor.setPidDegrees(62);
                caseSwitch++;
                break;
            }

            case 7:
            {
                if(motor.turn(.05, 0, .13))
                {
                    caseSwitch++;
                }
                break;
            }

            case 8:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                robot.stop_and_reset_encoder();
                robot.run_using_encoder();
                caseSwitch++;
                break;
            }

            case 9:
            {
                if(motor.driveWithEncoder(34 , 1, "f"))
                {
                    caseSwitch++;
                }
                break;
            }

            case 10:
            {
                robot.setTurnPid();
                motor.setPidDegrees(-40);
                caseSwitch++;
                break;
            }

            case 11:
            {
                if(motor.turn(.05, 0, .15))
                {
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }

            case 12:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                robot.stop_and_reset_encoder();
                robot.run_using_encoder();
                caseSwitch++;
                break;
            }

            case 13:
            {
                caseSwitch++;
                break;
            }

            case 14:
            {
                robot.setSlidePid();
                motor.setPidDegrees(0);
                caseSwitch++;
                break;
            }

            case 15:
            {
                if(motor.wallPIDSide(.05, .05, 1, 4))//TODO Set Distance and make sure we are straight with gyro
                {
                    caseSwitch++;
                }
                break;
            }


            case 16:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                caseSwitch++;
                break;
            }

            case 17:
            {
                if(motor.forward(.35, robot.colorLeft.alpha() > 10)) //TODO rename and replace sensors
                {
                    caseSwitch++;
                }
                break;
            }

            case 18:
            {
                if(motor.wallPIDSide(.15, .05, 1, 4))//TODO Set Distance and make sure we are straight with gyro
                {
                    robot.setSlidePid();
                    motor.setPidDegrees(0);
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }
            case 19:
            {
                if(motor.right(1, robot.timer.time() > .35 ))
                {
                    caseSwitch++;
                }
                break;
            }
            case 20:
            {
                if(robot.beaconColor.red() > robot.beaconColor.blue())
                {
                    caseSwitch++;
                }
                else
                {
                    caseSwitch+=2;
                }
                break;
            }

            case 21:
            {
                robot.setButtonPushLeft();
                caseSwitch += 2;
                robot.timer.reset();
                break;
            }

            case 22:
            {
                robot.setButtonPushRight();
                caseSwitch ++ ;
                robot.timer.reset();
                break;
            }

            case 23:
            {
                if(robot.timer.time() > 2)
                {
                    robot.setButtonPushInit();
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }

            case 24:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                robot.run_without_encoder();
                robot.run_using_encoder();
                caseSwitch++;
                break;
            }

            case 25:
            {
                if(motor.driveWithEncoder(28, 1, "f"))
                {
                    caseSwitch++;
                }
                break;
            }

            case 26:
            {
                if(motor.forward(.35, robot.colorLeft.alpha() > 10 && robot.timer.time() > 1)) //TODO rename and replace sensors
                {
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }
            case 27:
            {
                if(motor.wallPIDSide(.15, .05, 1, 4))//TODO Set Distance and make sure we are straight with gyro
                {
                    robot.setSlidePid();
                    motor.setPidDegrees(0);
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }
            case 28:
            {
                if(motor.right(1, robot.timer.time() > .35 ))
                {
                    caseSwitch++;
                }
                break;
            }
            case 29:
            {
                if(robot.beaconColor.red() > robot.beaconColor.blue())
                {
                    caseSwitch++;
                }
                else
                {
                    caseSwitch+=2;
                }
                break;
            }

            case 30:
            {
                robot.setButtonPushLeft();
                caseSwitch += 2;
                robot.timer.reset();
                break;
            }

            case 31:
            {
                robot.setButtonPushRight();
                caseSwitch ++ ;
                robot.timer.reset();
                break;
            }

            case 32:
            {
                if(robot.timer.time() > 2)
                {
                    robot.setButtonPushInit();
                    caseSwitch++;
                }
                break;
            }
        }
        telemetry.addData("Case", caseSwitch);
        telemetry.addData("PidOutput", robot.yawPIDResult.getOutput());
        telemetry.addData("BackLeft", robot.backLeft.getPower());
        telemetry.addData("BackRight",  robot.backRight.getPower());
        telemetry.addData("FrontLeft", robot.frontLeft.getPower());
        telemetry.addData("FrontRight",  robot.frontRight.getPower());
        telemetry.addData("Gyro", robot.navx_device.getYaw());
        telemetry.addData("Yaw Set", robot.yawPIDController.getSetpoint());
    }

    @Override
    public void stop()
    {

        robot.navx_device.close();
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

}