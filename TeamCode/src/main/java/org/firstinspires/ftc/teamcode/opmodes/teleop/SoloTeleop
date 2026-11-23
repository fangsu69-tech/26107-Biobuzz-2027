package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

@Configurable
@TeleOp(name = "test Teleop (1 controller")
public class SoloTeleop extends OpMode {

    @Sorter(sort = 0)  public static double teargetVelocity = 2500;
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
        // 1. Refresh Data & Drive (Runs every loop)
        robot.clearCache();
        robot.drive.updateOdo();

//        if (gamepad1.a) {
//            robot.drive.alignHeadingToAprilTag(1.0);
//        } else {
            robot.drive.gamepadDrive(gamepad1);
//        }

        if (gamepad1.xWasPressed()) {
            robot.drive.switchOrientation();
        }

        // 2. Transfer Logic (Independent)
        if (gamepad1.b) {
            robot.intake.runTransfer();
        } else if (gamepad2.x) {
            robot.intake.retractTransfer();
        } else {
            robot.intake.stopTransfer(); // Stop if B is released
        }

        // 3. Intake Logic (Independent)
        if (gamepad1.a) {
            robot.intake.runIntake();
        } else if (gamepad2.left_bumper) {
            robot.intake.retractIntake();
        } else {
            robot.intake.stopIntake(); // Stop if A is released
        }

        // 4. Shooter Logic (Independent)
        robot.shooter.setRPM(teargetVelocity);
        if (gamepad1.y) {
            robot.shooter.run();
                TelemetryUtils.addData("targetRPM", teargetVelocity);
        } else {
            robot.shooter.stop();
            TelemetryUtils.addData("targetRPM", 0);

        }
//        // Modulate shooter speed with direction pad
//        if (gamepad2.dpad_down) {
//            robot.shooter.speed -= 0.0025;
//            if (robot.shooter.speed <= 0) {
//                robot.shooter.speed = 0;
//            }
//        } else if (gamepad2.dpad_up) {
//            robot.shooter.speed += 0.0025;
//            if (robot.shooter.speed >= 1) {
//                robot.shooter.speed = 1;
//            }
//        }

        // 5. Utility Logic
        if (gamepad1.start) {
            robot.drive.resetHeading();
        }
        robot.drive.acceleration = gamepad1.right_bumper;

        // 6. Telemetry (Moves the "Finish Line" to the end)
        double loopTime = loopTimer.milliseconds();
        loopTimer.reset();
        TelemetryUtils.addData("Loop Hz", 1000.0 / loopTime);

        TelemetryUtils.addData("speed", robot.shooter.getCurrentRPM());
        TelemetryUtils.update();
    }

}
