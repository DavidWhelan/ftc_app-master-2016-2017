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

import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Blue Beacon Improved 3", group="TechHogs")
//@Disabled
public class BlueBeaconCoreNew3 extends OpMode
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
        robot.flyWheel.setMaxSpeed(2175);
        robot.navx_device.zeroYaw();
    }

    @Override
    public void loop()
    {
        switch(caseSwitch)
        {
            //prepare and drive forward 6 inches
            case 0:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                robot.stop_and_reset_encoder();
                robot.run_using_encoder();
                robot.blockUp();
                caseSwitch++;

                break;
            }

            case 1:
            {
                if(motor.driveWithEncoder(6, 1, "f"))
                {
                    caseSwitch+=3;
                }
                break;
            }
            //preapre and turn 45 degrees
            case 2:
            {
                //robot.setTurnPid();
                robot.yawPIDController.setPID(.04, 0, .105);
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
            //preapre and drive to beacon
            case 4:
            {
                robot.setDrivePid();
                motor.setPidDegrees(45);
                robot.stop_and_reset_encoder();
                robot.run_using_encoder();
                caseSwitch++;
                break;
            }

            case 5:
            {
                if(motor.driveWithEncoder(32 , 1, "f"))
                {
                    caseSwitch++;
                }
                break;
            }
            //Turn even with the beacon
            case 6:
            {
                //robot.setTurnPid();
                robot.yawPIDController.setPID(.045, 0, .11);
                motor.setPidDegrees(-45);
                caseSwitch++;
                break;
            }

            case 7:
            {
                if(motor.turn())
                {
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }
            //drive to the beacon
            case 8:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                robot.stop_and_reset_encoder();
                robot.run_using_encoder();
                caseSwitch+=2;
                break;
            }

            case 9:
            {
                if(motor.driveWithEncoder(0, 1, "b"))
                {
                    caseSwitch++;
                }
                break;
            }
            //Slide into beacon
            case 10:
            {
                robot.setSlidePid();
                motor.setPidDegrees(0);
                caseSwitch++;
                break;
            }

            case 11:
            {
                if(motor.wallPIDSide(.05, .05, 1, 4))//TODO Set Distance and make sure we are straight with gyro
                {
                    caseSwitch++;
                }
                break;
            }

            //Drive untill we see the line
            case 12:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                robot.timer.reset();
                robot.sweeper.setPower(1);
                caseSwitch++;
                break;
            }

            case 13:
            {
                if(robot.timer.time() > .2)
                {
                    robot.sweeper.setPower(0);
                }
                if(motor.forward(.25, robot.colorLeft.alpha() > 10)) //TODO rename and replace sensors
                {
                    caseSwitch++;
                }
                break;
            }
            //Slide closer to the wall
            case 14:
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
            case 15:
            {
                if(motor.right(1, robot.timer.time() > .35 ))
                {
                    caseSwitch++;
                }
                break;
            }
            //Check beacons and prees the right button
            case 16:
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

            case 17:
            {
                robot.setButtonPushLeft();
                caseSwitch += 2;
                robot.timer.reset();
                break;
            }

            case 18:
            {
                robot.setButtonPushRight();
                caseSwitch ++ ;
                robot.timer.reset();
                break;
            }

            case 19:
            {
                if(robot.timer.time() > .5)//TODO
                {
                    robot.setButtonPushInit();
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }
            //Drive untill we see the next line
            case 20:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                robot.stop_and_reset_encoder();
                robot.run_using_encoder();
                caseSwitch++;
                break;
            }

            case 21:
            {
                if(motor.driveWithEncoder(34, 1, "f"))
                {
                    caseSwitch++;
                }
                break;
            }

            case 22:
            {
                if(motor.forward(.25, robot.colorLeft.alpha() > 10 && robot.timer.time() > 1)) //TODO rename and replace sensors
                {
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }
            //Slide into the wall
            case 23:
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

            case 24:
            {
                if(motor.right(1, robot.timer.time() > .35 ))
                {
                    caseSwitch++;
                }
                break;
            }
            //check buttons and press right one
            case 25:
            {
                if(robot.beaconColor.red() > robot.beaconColor.blue())
                {
                    caseSwitch++;
                }
                else
                {
                    caseSwitch+=2;
                }
                robot.angleAdjust.setPosition(.11);
                break;
            }

            case 26:
            {
                robot.setButtonPushLeft();
                caseSwitch += 2;
                robot.timer.reset();
                break;
            }

            case 27:
            {
                robot.setButtonPushRight();
                caseSwitch++ ;
                robot.timer.reset();
                break;
            }

            case 28:
            {
                if(robot.timer.time() > .5)//TODO
                {
                    robot.timer.reset();
                    robot.setSlidePid();
                    motor.setPidDegrees(0);
                    robot.setButtonPushInit();
                    caseSwitch++;
                }
                break;
            }
            //slide away from wall
            case 29:
            {
                if(motor.left(1, robot.sideRange.getDistance(DistanceUnit.INCH) > 6))
                {
                    robot.flyWheel.setPower(1);
                    caseSwitch++;
                }
                break;
            }

            case 30:
            {
                robot.yawPIDController.setPID(.04, 0, .11);
                robot.yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, 1);
                motor.setPidDegrees(43);
                caseSwitch++;
                break;
            }
            //turn to face goal
            case 31:
            {
                if(motor.turn())
                {
                    caseSwitch++;
                }
                break;
            }

            case 32:
            {
                robot.setDrivePid();;
                motor.setPidDegrees(0);
                robot.stop_and_reset_encoder();
                robot.run_using_encoder();
                caseSwitch++;
                break;
            }
            //drive to goal
            case 33:
            {
                if(motor.driveWithEncoder(33, 1, "b"))
                {
                    robot.sweeper.setPower(1);
                    robot.timer.reset();
                    caseSwitch++;
                }
                break;
            }
            //shoot
            case 34:
            {
                if(robot.timer.time() > 3)
                {
                    robot.sweeper.setPower(0);
                    robot.flyWheel.setPower(0);
                    caseSwitch++;
                }
                break;
            }
            //drive onto platform
            case 35:
            {
                robot.setDrivePid();
                motor.setPidDegrees(0);
                robot.stop_and_reset_encoder();
                robot.run_using_encoder();
                caseSwitch++;
                break;
            }

            case 36:
            {
                if(motor.driveWithEncoder(18, 1, "b"))
                {
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
        telemetry.addData("Yaw Out", robot.yawPIDResult.getOutput());

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