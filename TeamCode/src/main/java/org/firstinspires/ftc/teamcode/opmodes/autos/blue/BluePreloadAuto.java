package org.firstinspires.ftc.teamcode.opmodes.autos.blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.utils.pedroPathing.Constants;

@Autonomous(name = "Blue Preload Auto", group = "Blue")
public class BluePreloadAuto extends OpMode {

    private Follower follower;

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;

    private Path scorePreload;

    private final Pose startPose =
            new Pose(28.5, 128, Math.toRadians(90));

    private final Pose shootPose =
            new Pose(64, 22.5, Math.toRadians(125));

    // ===== AUTO STATE MACHINE =====

    private enum AutoState {
        DRIVE_TO_SHOOT,
        WAIT_FOR_READY,
        FIRE_PRELOAD,
        DONE
    }

    private AutoState currentState;

    // ===== TIMERS =====

    private long fireStartTime;

    // ===== CONSTANTS =====

    private static final double SHOOTER_RPM = 3300;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        shooter = new ShooterSubsystem();
        shooter.initialize(hardwareMap);

        intake = new IntakeSubsystem();
        intake.initialize(hardwareMap);

        buildPaths();

        currentState = AutoState.DRIVE_TO_SHOOT;
    }

    @Override
    public void start() {

        // Start driving
        follower.followPath(scorePreload);

        // Spin shooter up immediately while moving
        shooter.setRPM(SHOOTER_RPM);
    }

    @Override
    public void loop() {

        // Always update these
        follower.update();
        shooter.run();

        telemetry.addData("State", currentState);
        telemetry.addData("Shooter RPM", shooter.getCurrentRPM());
        telemetry.update();

        switch (currentState) {

            case DRIVE_TO_SHOOT:

                // Wait until robot reaches target
                if (!follower.isBusy()) {
                    currentState = AutoState.WAIT_FOR_READY;
                }

                break;

            case WAIT_FOR_READY:

                // Wait until shooter stabilizes
                if (shooter.isStableReadyToFire()) {

                    // Start feeding rings
                    intake.runIntake();
                    intake.runTransfer();

                    fireStartTime = System.currentTimeMillis();

                    currentState = AutoState.FIRE_PRELOAD;
                }

                break;

            case FIRE_PRELOAD:

                // Run feeder for fixed duration
                if (System.currentTimeMillis() - fireStartTime > 1500) {

                    intake.stopIntake();
                    intake.stopTransfer();

                    shooter.stop();

                    currentState = AutoState.DONE;
                }

                break;

            case DONE:
                break;
        }
    }

    public void buildPaths() {

        scorePreload = new Path(
                new BezierLine(startPose, shootPose)
        );

        scorePreload.setLinearHeadingInterpolation(
                startPose.getHeading(),
                shootPose.getHeading()
        );
    }
}