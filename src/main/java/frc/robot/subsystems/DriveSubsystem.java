package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.lib.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.controllers.MAXSwerveModule;

public class DriveSubsystem extends SubsystemBase {
    public DriveSubsystem() {
        m_gyro.reset();
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(2, 0, 0.1), // Translation PID constants
                    new PIDConstants(0.95, 0.01, 0), // Rotation PID constants
                    8, // Max module speed, in m/s
                    0.646557, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {

              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red sidefget of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this// Reference to this subsystem to set requirements
        );
    }

    public Command angleToSpeakerFromStage() {
        return run(this::rotateToSpeakerFromStage);
    }
    public Command angleToSpeakerUsingAprilTag() {
        return run(this::rotateToSpeakerAprilTag).onlyWhile(RobotContainer.readyToShoot());
    }

    public void rotateToSpeakerFromStage() {
        this.drive(
                -MathUtil.applyDeadband(RobotContainer.m_operatorController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(RobotContainer.m_operatorController.getLeftX(), OIConstants.kDriveDeadband),
                (23 + this.getTrueHeading()) * -.08,
            true, .02
        );
    }
    public void rotateToSpeakerAprilTag() {
        this.drive(
                -MathUtil.applyDeadband(RobotContainer.m_operatorController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(RobotContainer.m_operatorController.getLeftX(), OIConstants.kDriveDeadband),
                this.limelight_aim_proportional(),
            true, .02
        );
    }
    public BooleanSupplier isRobotFacingSpeaker() {
        return (BooleanSupplier) () -> {
            if ((23 + this.getTrueHeading()) * -.08 < .1) return true;
            else return false;
        };
    }
    public static double limelight_aim_proportional() {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .0015;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= DriveConstants.kMaxSpeedMetersPerSecond;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;
        // System.out.println(targetingAngularVelocity);

        return targetingAngularVelocity;
    }

    public static final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    public static final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    public static final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    public static final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    public final TalonFX talonFrontLeft = new TalonFX(DriveConstants.kFrontLeftDrivingCanId);
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        talonFrontLeft, 
        DriveConstants.kFrontLeftTurningCanId, 
        DriveConstants.kFrontLeftChassisAngularOffset
    );
    public final TalonFX talonFrontRight = new TalonFX(DriveConstants.kFrontRightDrivingCanId);
    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        talonFrontRight, 
        DriveConstants.kFrontRightTurningCanId, 
        DriveConstants.kFrontRightChassisAngularOffset
    );
    public final TalonFX talonRearLeft = new TalonFX(DriveConstants.kRearLeftDrivingCanId);
    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
        talonRearLeft, 
        DriveConstants.kRearLeftTurningCanId, 
        DriveConstants.kRearLeftChassisAngularOffset
    );
    public final TalonFX talonRearRight = new TalonFX(DriveConstants.kRearRightDrivingCanId);
    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
        talonRearRight, 
        DriveConstants.kRearRightTurningCanId, 
        DriveConstants.kRearRightChassisAngularOffset
    );

    public final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    public double getTrueHeading() {
        return Math.IEEEremainder(-m_gyro.getAngle(), 360.0d);  
    }
    
    public final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    public final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          Rotation2d.fromDegrees(this.getTrueHeading()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });
    /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, Rotation2d.fromDegrees(this.getTrueHeading()))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(this.getTrueHeading()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(this.getTrueHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    );
  }
  public void driveRobotRelative(ChassisSpeeds speeds){
      System.out.println(speeds.vxMetersPerSecond + " : " + speeds.vyMetersPerSecond);
      this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false, .02);
  }
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(this.getTrueHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }
}
