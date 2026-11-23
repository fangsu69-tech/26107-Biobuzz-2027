//
package org.firstinspires.ftc.teamcode.opmodes.teleop;


@Configurable
@TeleOp(name = "Main Teleop") //create the teleOp
public class MainTeleop extends OpMode {
    @Sorter(sort 0) public static double TargetVelocity = 3000;
    private ElapsedTime loopTimer = new ElapsedTime ();
    Robot robot = new Robot();

    @Override
    public void init() /*british*/ {
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

        // Intake stuff
        if (gamepad2.a) {
            robot.intake.runIntake();
        } else if (gamepad2.left_bumper) {
            robot.intake.retractIntake();
        } else {
            robot.intake.stopIntake(); // does nothing if you dont press A
        }

        // Transfer stuff
        if (gamepad2.b) {
            robot.intake.runTransfer();
            robot.intake.runIntake();
        } else if (gamepad2.x) {
            robot.intake.retractTransfer();
        } else {
            robot.intake.stopTransfer(); // does nothing if you dont press B
        }

        // Shooter stuff
        robot.shooter.setRPM(teargetVelocity);
        if (gamepad2.y) {
            robot.shooter.run();
            TelemetryUtils.addData("targetRPM", teargetVelocity);
        } else {
            robot.shooter.stop();
            TelemetryUtils.addData("targetRPM", 0); // does nothing if you dont press Y
        }

        // reset heading
        if (gamepad1.start) {
            robot.drive.resetHeading();
        }
        // Telemetry stuff
        double loopTime = loopTimer.milliseconds();
        loopTimer.reset();
        TelemetryUtils.addData("Loop Hz", 1000.0 / loopTime);

        TelemetryUtils.addData("speed", robot.shooter.getCurrentRPM());
        TelemetryUtils.update();
    }

}
