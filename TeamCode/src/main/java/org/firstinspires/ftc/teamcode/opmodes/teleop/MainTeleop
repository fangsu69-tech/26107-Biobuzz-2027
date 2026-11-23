package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

@Configurable
@TeleOp(name = "main Teleop (2 controllers")
public class MainTeleop extends OpMode {

    @Sorter(sort = 0)  public static double teargetVelocity = 4000;
    private ElapsedTime loopTimer = new ElapsedTime();
    // Using the Robot container instead of individual subsystems
    Robot robot = new Robot();

    @Override
    public void init() {
        // Initializes everything: Subsystems, Hubs, and Telemetry
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        loopTimer.reset();
    }


    @Override
    public void loop() {
        // Refresh Data & Drive (Runs every loop)
        robot.clearCache();
        robot.drive.updateOdo();

        if (gamepad1.a) {
            robot.drive.alignHeadingToAprilTag(1.0);
        } else {
            robot.drive.gamepadDrive(gamepad1);
        }

//        if (gamepad1.xWasPressed()) {
//            robot.drive.switchOrientation();
//        }

            // Transfer Logic
            if (gamepad2.b) {
                robot.intake.runTransfer();
                robot.intake.runIntake();
            } else if (gamepad2.x) {
                robot.intake.retractTransfer();
            } else {
                robot.intake.stopTransfer(); // Stop if B is released
            }

            // Intake Logic
            if (gamepad2.a) {
                robot.intake.runIntake();
            } else if (gamepad2.left_bumper) {
                robot.intake.retractIntake();
            } else {
                robot.intake.stopIntake(); // Stop if A is released
            }

            // Shooter Logic
            robot.shooter.setRPM(teargetVelocity);
            if (gamepad2.y) {
                robot.shooter.run();
                TelemetryUtils.addData("targetRPM", teargetVelocity);
            } else {
                robot.shooter.stop();
                TelemetryUtils.addData("targetRPM", 0);

            }
            // Utility Logic
            if (gamepad1.start) {
                robot.drive.resetHeading();
            }


            // Telemetry
            double loopTime = loopTimer.milliseconds();
            loopTimer.reset();
            TelemetryUtils.addData("Loop Hz", 1000.0 / loopTime);

            TelemetryUtils.addData("speed", robot.shooter.getCurrentRPM());
            TelemetryUtils.update();
        }

    }
