/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.MultiOpModeManager;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnumHelper;

@Config
@Disabled
@Autonomous(name = "ticksTest", group = "Test")
public class ticksTest extends MultiOpModeManager {
    // Hardware
    private NewNewDrive drive;
    private Servo collectorArm = null;

    // Constants
    public static double speedMin = 0.1;
    public static double speedMax = 0.8;
    public static double r = 16;
    public static double arcLength = 16;
    public static double angle = arcLength / Math.PI * 180.0 / r;
    public static double COLLECTOR_UP = 0.53;
    public static int num = 0;
    public static int Distance = 30;

    // Members
    private AUTO_STATE state = AUTO_STATE.DONE;
    private InputHandler in;

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        try {
            super.register(new NewNewDrive());

            drive = new NewNewDrive();
            super.register(drive);

            in = Globals.input(this);

            super.init();
        } catch (Exception e) {
            telemetry.log().add(String.valueOf(e));
            error = true;
        }

        try {
            collectorArm = hardwareMap.get(Servo.class, "CollectorArm");
        } catch (Exception e) {
            telemetry.log().add("Could not find collector");
            error = true;
        }

        // Initialization status
        String status = "Ready";
        if (error) {
            status = "Hardware Error";
        }
        telemetry.addData("Status", status);
        drive.enableLogging();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        num = 0;
        drive.setDoneFalse();
        state = AUTO_STATE.MOVE;
    }

    @Override
    public void loop() {
        in.loop();

        // Step through the auto commands
        switch (state) {
            case MOVE:
                drive.driveTo(Distance, speedMin, speedMax);
                collectorArm.setPosition(COLLECTOR_UP);
                if (drive.isDone() && !drive.isBusy()) {
                    drive.setDoneFalse();
                    state = state.next();
                }
                break;
            // Stop processing
            case DONE:
                break;
        }

        //log what state it currently is in
        telemetry.addData("Auto Step: ", state);
    }

    @Override
    public void stop() {
        super.stop();
    }

    enum AUTO_STATE implements OrderedEnum {
        MOVE,
        DONE;

        public ticksTest.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }
}