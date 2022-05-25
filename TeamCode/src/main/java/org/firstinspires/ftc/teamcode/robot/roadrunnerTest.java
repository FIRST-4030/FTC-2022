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

package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.momm.MultiOpModeManager;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;

@Config
@Disabled
@Autonomous(name = "roadrunnerTest", group = "Test")
public class roadrunnerTest extends MultiOpModeManager {
    // Hardware
    private Servo collectorArm = null;

    // Constants
    private static float DRIVE_POWER = 0.5f;
    private static double TURN_RATIO = 6.375;
    private int num = 0;
    private static double COLLECTOR_UP = 0.37;
    private static double COLLECTOR_DOWN = 0.90;

    // Members
    private ElapsedTime runtime = new ElapsedTime();
    private AUTO_STATE state = AUTO_STATE.DONE;
    private boolean redAlliance = false;
    private boolean duckSide = false;
    private InputHandler in;
    private SampleTankDrive tankdrive;
    private Pose2d startPose;
    private Trajectory traj1;
    private TelemetryPacket packet = new TelemetryPacket();
    private Canvas fieldOverlay = packet.fieldOverlay();

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        try {
            tankdrive = new SampleTankDrive(hardwareMap);
            collectorArm = hardwareMap.get(Servo.class, "CollectorArm");

            in = Globals.input(this);
            super.init();
        } catch (Exception e) {
            telemetry.log().add(String.valueOf(e));
            error = true;
        }

        // Initialization status
        String status = "Ready";
        if (error) {
            status = "Hardware Error";
        }
        telemetry.addData("Status", status);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_right) redAlliance = true;
        if (gamepad1.dpad_left) redAlliance = false;
        if (gamepad1.dpad_up) duckSide = true;
        if (gamepad1.dpad_down) duckSide = false;
        telemetry.addData("Alliance", redAlliance ? "Red" : "Blue");
        telemetry.addData("Direction", duckSide ? "Duck" : "Warehouse");
    }

    @Override
    public void start() {
        if (redAlliance) {
            if (duckSide) {
                startPose = new Pose2d(-36, -72, 90);
            } else {
                startPose = new Pose2d(12, -72, 90);
            }
        } else {
            if (duckSide) {
                startPose = new Pose2d(-36, 72, -90);
            } else {
                startPose = new Pose2d(12, 72, -90);
            }
        }
        collectorArm.setPosition(COLLECTOR_UP);
        tankdrive.setPoseEstimate(startPose);
        traj1 = tankdrive.trajectoryBuilder(startPose, false)
                .forward(15)
                .build();
        state = AUTO_STATE.MOVE;
    }

    @Override
    public void loop() {
        // Stop when the autoSteps are complete
        tankdrive.update();

        in.loop();

        // Step through the auto commands
        switch (state) {

            case MOVE:
                if (num == 0) {
                    tankdrive.followTrajectoryAsync(traj1);
                    num++;
                }
                DashboardUtil.drawSampledPath(fieldOverlay, traj1.getPath());
                DashboardUtil.drawRobot(fieldOverlay, tankdrive.getPoseEstimate());
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                if (gamepad1.a && !tankdrive.isBusy()) {
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
    }

    enum AUTO_STATE implements OrderedEnum {
        MOVE,
        DONE;

        public roadrunnerTest.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }
}