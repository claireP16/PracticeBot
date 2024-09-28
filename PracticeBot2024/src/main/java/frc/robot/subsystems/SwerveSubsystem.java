package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

//add motor channel numbers later
public class SwerveSubsystem extends SubsystemBase {

        private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(2);
        private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(2);
        private final SlewRateLimiter rotRateLimiter = new SlewRateLimiter(2);

        private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
        private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
        private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
        private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

        private final SwerveModule fLSwerve = new SwerveModule(15, 14, 20, true, true, -0.137);
        private final SwerveModule fRSwerve = new SwerveModule(13, 12, 19, true, true, 0);
        private final SwerveModule bLSwerve = new SwerveModule(17, 16, 21, true, true, 0.172);
        private final SwerveModule bRSwerve = new SwerveModule(11, 10, 18, true, true, -0.429);

        private static AHRS gyro = new AHRS(SPI.Port.kMXP);

        private LinearFilter hitFilter = LinearFilter.movingAverage(30);

        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        public void drive(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
                drive(xPercent, yPercent, rotPercent, fieldRelative, 0, 0);
        }

        public void drive(double xPercent, double yPercent, double rotPercent, boolean fieldRelative, double a,
                        double b) {

                // System.out.println("DRIVE");
                // if (!AutoAlignTags.running) {
                // var st = Thread.currentThread().getStackTrace();
                // for (var s : st) {
                //         System.out.println(s + "\n");
                // }

                // }

                var xSpeed = xRateLimiter.calculate(xPercent) * Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var ySpeed = yRateLimiter.calculate(yPercent) * Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var rot = rotRateLimiter.calculate(rotPercent)
                                * Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond;

                // if (!primaryJoy.getTrigger()) {
                // xSpeed *= 0.5;
                // ySpeed *= 0.5;
                // rot *= 0.2;
                // } else {
                // rot *= 0.5;
                // }

                // while(primaryJoy.getRawButton(7)){
                // xSpeed *= 0.75;
                // ySpeed *= 0.75;
                // rot *= 0.2;
                // }
                ChassisSpeeds chasSpeed = fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot);

                // var swerveModuleStates = kinematics.toSwerveModuleStates(chasSpeed);

                // TODO: DEFINE MAX SPEED
                var swerveModuleStates2 = DriveConstants.kinematics.toSwerveModuleStates(
                                ChassisSpeeds.discretize(chasSpeed, 0.2),
                                new Translation2d(DriveConstants.kTrackBaseMeters * a * 1.5,
                                                DriveConstants.kTrackWidthMeters * b * 1.5));

                driveStates(swerveModuleStates2);

        }

        public double getDriveMotorVelocity(){
                return hitFilter.calculate(fLSwerve.driveMotor.getEncoder().getVelocity());
        }

        public boolean uh = false;

        public boolean hasHitSomething(){
                if(Math.abs(getDriveMotorVelocity()) > 1.3){
                        uh = true;
                }
                if(uh == true && Math.abs(getDriveMotorVelocity()) < 0.4){
                        return true;
                } else {
                        return false;
                }
        }

        SwerveDriveOdometry ometry = new SwerveDriveOdometry(
                        kinematics,
                        gyro.getRotation2d(),
                        new SwerveModulePosition[] {
                                        fLSwerve.getPosition(),
                                        fRSwerve.getPosition(),
                                        bLSwerve.getPosition(),
                                        bRSwerve.getPosition()
                        });

        public static void zeroYaw() {
                gyro.zeroYaw();
        }
        // Configure AutoBuilder last

        @Override
        public void periodic() {
                // TODO Auto-generated method stub
                ometry.update(
                                gyro.getRotation2d(),
                                new SwerveModulePosition[] {
                                                fLSwerve.getPosition(),
                                                fRSwerve.getPosition(),
                                                bLSwerve.getPosition(),
                                                bRSwerve.getPosition()
                                }

                );

        }

        public Pose2d getPose() {
                return ometry.getPoseMeters();
        }

        public double getYaw() {
                return gyro.getYaw();
        }

        public void resetOmetry(Pose2d pose) {
                ometry.resetPosition(
                                gyro.getRotation2d(),
                                new SwerveModulePosition[] {
                                                fLSwerve.getPosition(),
                                                fRSwerve.getPosition(),
                                                bLSwerve.getPosition(),
                                                bRSwerve.getPosition()
                                },
                                pose);
        }

        public SwerveModuleState[] getModuleStates() {
                SwerveModuleState[] states = {
                                fLSwerve.getState(),
                                fRSwerve.getState(),
                                bLSwerve.getState(),
                                bRSwerve.getState(),
                };
                return states;
        }

        public ChassisSpeeds getSpeeds() {
                return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
        }

        public void driveStates(SwerveModuleState[] swerveModuleStates) {
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                                Constants.DriveConstants.MaxVelocityMetersPerSecond);

                SwerveModuleState[] optimizedSwerveModuleStates = {
                                fLSwerve.optimizeModuleState(swerveModuleStates[0]),
                                fRSwerve.optimizeModuleState(swerveModuleStates[1]),
                                bLSwerve.optimizeModuleState(swerveModuleStates[2]),
                                bRSwerve.optimizeModuleState(swerveModuleStates[3]),
                };

                fLSwerve.setDesiredState(optimizedSwerveModuleStates[0]);
                fRSwerve.setDesiredState(optimizedSwerveModuleStates[1]);
                bLSwerve.setDesiredState(optimizedSwerveModuleStates[2]);
                bRSwerve.setDesiredState(optimizedSwerveModuleStates[3]);
        }

        public SwerveSubsystem() {
                
                Shuffleboard.getTab("Debug").addDouble("drive velocity", this::getDriveMotorVelocity);
                Shuffleboard.getTab("Debug").addDouble("drive velocity unfiltered", () -> fLSwerve.driveMotor.getEncoder().getVelocity());
        }
}
