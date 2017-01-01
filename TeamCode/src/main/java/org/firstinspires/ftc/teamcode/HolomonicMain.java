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
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Holomonic", group="TechHogs")  // @Autonomous(...) is the other common choice
public class HolomonicMain extends OpMode
{

    RobotHardware robot = new RobotHardware();
    boolean release = false;

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
        robot.run_using_encoder();
        robot.setMaxSpeed(3500);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1RightX = -gamepad1.right_stick_x;

            if(Math.abs(gamepad1LeftX) > 0.01 || Math.abs(gamepad1LeftY) > 0.01 || Math.abs(gamepad1RightX) > 0.01)
            {
                float FrontLeft1 = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
                float FrontRight1 = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
                float BackRight1 = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
                float BackLeft1 = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

                FrontRight1 = Range.clip(FrontRight1, -1, 1);
                FrontLeft1 = Range.clip(FrontLeft1, -1, 1);
                BackRight1 = Range.clip(BackRight1, -1, 1);
                BackLeft1 = Range.clip(BackLeft1, -1, 1);

                robot.frontRight.setPower(FrontRight1);
                robot.frontLeft.setPower(FrontLeft1);
                robot.backRight.setPower(BackRight1);
                robot.backLeft.setPower(BackLeft1);
            }
        ///////////////////////////////////////////////////////////
            else
            {
                float gamepad2LeftX = gamepad2.left_stick_x;
                float gamepad2LeftY = -gamepad2.left_stick_y;
                float gamepad2RightX = -gamepad2.right_stick_x;

                float FrontLeft2 = gamepad2LeftY + gamepad2LeftX + gamepad2RightX;
                float FrontRight2 = gamepad2LeftY - gamepad2LeftX - gamepad2RightX;
                float BackRight2 = gamepad2LeftY - gamepad2LeftX + gamepad2RightX;
                float BackLeft2 = gamepad2LeftY + gamepad2LeftX - gamepad2RightX;

                FrontRight2 = Range.clip(FrontRight2, -1, 1) / 2;
                FrontLeft2 = Range.clip(FrontLeft2, -1, 1) / 2;
                BackRight2 = Range.clip(BackRight2, -1, 1) / 2;
                BackLeft2 = Range.clip(BackLeft2, -1, 1) / 2;

                robot.frontRight.setPower(FrontRight2);
                robot.frontLeft.setPower(FrontLeft2);
                robot.backRight.setPower(BackRight2);
                robot.backLeft.setPower(BackLeft2);
            }
        ////////////////////////////////////////////////////////////
        robot.sweeper.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if(gamepad1.right_bumper)
        {
            robot.lifter.setPower(1);
        }
        else if(gamepad1.left_bumper)
        {
            robot.lifter.setPower(-1);
        }
        else
        {
            robot.lifter.setPower(0);
        }

        //******************************************************************************************


        if(gamepad2.a) robot.flyWheel.setPower(0);
        if(gamepad2.y) robot.flyWheel.setPower(1);

        if(gamepad2.x) robot.setButtonRightInit();
        if(gamepad2.b) robot.setButtonRightPress();

        if(gamepad2.dpad_up) robot.setAngleBackward();
        if(gamepad2.dpad_down) robot.setAngleForward();

        //apply gamepad 2 servo for fly wheel

        //apply for trajectory change

        //apply for button pusher servo
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
        robot.navx_device.close();
    }

}
