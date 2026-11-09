package org.firstinspires.ftc.teamcode;
@TeleOp

@Override
public void init() {
   robot.initi(hardwarMap, telemetry);
}

@Override
public void start() {
    loopTimer.reset();
}

@Override
public void loop() {
    robot.clearCache();
    robot.drive.updateOdo();

    if (gamepad1.a) {
        robot.drive.alignHeadingToAprilTag(1.0);
    } else {
        robot.drive.gamepadDrive(gamepad1);
    }
}

// Transfer

if (gapepad2.b) {
    robot.intake.runTransfer();
    robot.intake.runIntake();
} else if (gamepad2.x) {
    robot.intake.retracstTransfer();
} else {
    robot.intake.stopTransfer(); // stops when B is not pressed anymore
}

// Intake

if (gamepad2.a) {
    robot.intake.runIntake();
} else if (gamepad2.left_bumper) {
    robot.intake.retracrIntake();
} else {
    robot.intake.stopIntake(); // stops when A is not pressed anymore
}

//Shooter

robot.shooter.setRPM(teargetVelocity);
   if (gamepad2.y);
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
