package org.firstinspires.ftc.teamcode.opmodes.autos.blue;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue 12 Ball Auto", group = "Blue")
public class Blue12BallAuto {
    private Follower follower;

    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of the robot.


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(34.378, 8.865),

                                    new Pose(63.700, 20.800)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(63.700, 20.800),

                                    new Pose(40.054, 35.189)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.054, 35.189),

                                    new Pose(17.432, 35.595)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.432, 35.595),

                                    new Pose(63.800, 20.865)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(63.800, 20.865),

                                    new Pose(40.541, 59.432)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.541, 59.432),

                                    new Pose(16.919, 59.270)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.919, 59.270),

                                    new Pose(63.800, 20.865)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(63.800, 20.865),

                                    new Pose(41.054, 83.676)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.054, 83.676),

                                    new Pose(16.486, 83.514)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.486, 83.514),

                                    new Pose(63.800, 20.800)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                    .build();
        }
    }
}
