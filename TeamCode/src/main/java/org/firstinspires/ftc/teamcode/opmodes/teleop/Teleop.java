//
package org.firstinspires.ftc.teamcode.opmodes.teleop;


@Configurable
@TeleOp(name = "Main Teleop")
public class MainTeleop extends OpMode {
    @Sorter(sort 0) public static double TargetVelocity = 4000;
    private ElapsedTime loopTimer = new ElapsedTime ();
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

// Intake
        if (gamepad2.a) {
            robot.intake.runIntake();
        } else if (gamepad2.left_bumper) {
            robot.intake.retractIntake();
        } else {
            robot.intake.stopIntake(); // Stop if A is released
        }