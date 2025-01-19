// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LimelightHelpers;
import frc.robot.Constants.ConfigurationConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.sensors.IntakeIR;
import frc.robot.sensors.PivotDockingLI;
import frc.robot.sensors.ShooterIR;
import frc.robot.sensors.WristBumperLI;
import frc.robot.sensors.WristDockingLI;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LeftHangingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.RightHangingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

import java.util.ArrayList;
import java.util.Collection;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private String selectedAuto;
    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    public static final SendableChooser<String> songChooser = new SendableChooser<>();

    public static final CommandXboxController m_operatorController = new CommandXboxController(ConfigurationConstants.kOperatorControllerPort);
    public static final OBController m_secondaryController = new OBController(ConfigurationConstants.kSecondaryControllerPort);

    // public static final LimeLight D_LIMELIGHT = new LimeLight();
    public static final IntakeIR D_INTAKE_IR = new IntakeIR(Constants.WristConstants.intakeIRSensorId);
    public static final ShooterIR D_SHOOTER_IR = new ShooterIR(Constants.DigitalInput.shooterIRSensorId);
    public static final WristBumperLI D_WRIST_BUMPER_LI = new WristBumperLI(Constants.DigitalInput.wristBumperLimitSwitchId);
    public static final WristDockingLI D_WRIST_DOCKING_LI = new WristDockingLI(Constants.DigitalInput.wristDockingLimitSwitchId);
    public static final PivotDockingLI D_PIVOT_DOCKING_LI = new PivotDockingLI(Constants.DigitalInput.pivotDockingLimitSwitchId);

    private final static IntakeSubsystem s_intake = new IntakeSubsystem();
    private final WristSubsystem s_wrist = new WristSubsystem();
    private final ShooterSubsystem s_shooter = new ShooterSubsystem();
    private final ElevatorSubsystem s_elevator = new ElevatorSubsystem();
    private final PivotSubsystem s_pivot = new PivotSubsystem();
    private final DriveSubsystem s_drive = new DriveSubsystem();
    private final LeftHangingSubsystem s_leftHanging = new LeftHangingSubsystem();
    private final RightHangingSubsystem s_rightHanging = new RightHangingSubsystem();
    private final RGBSubsystem s_lights = new RGBSubsystem();
    public static Orchestra falconChoir = new Orchestra();

    public static BooleanSupplier readyToShoot() {
        return (BooleanSupplier) () -> {
            double pose = LimelightHelpers.getTY("limelight");
            // double id = LimelightHelpers.getFiducialID("limelight");
            if (DriverStation.getAlliance().isPresent()) {
                // int expectedId = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
                if (pose != 0.0 && RobotContainer.D_SHOOTER_IR.supplier.getAsBoolean()) return true;
                return false;
            } else return false;
        };
    }
    public BooleanSupplier isLimelightOnSpeaker() {
        return (BooleanSupplier) () -> {
            if (Math.abs(LimelightHelpers.getTX("limelight")) < 1.8) {
                return true;
            } else return false;
        };
    }
    public BooleanSupplier isAimedAtSpeaker() {
        return (BooleanSupplier) () -> {
            if (s_pivot.m_shooterPivot.encoder().getPosition() > (s_pivot.curveEstimateAdjusted - .008) && s_pivot.m_shooterPivot.encoder().getPosition() < (s_pivot.curveEstimateAdjusted + .008)) {
                return true;
            } else return false;
        };
    }
    public static BooleanSupplier readyToSource() {
        return (BooleanSupplier) ()-> {
            if (m_operatorController.povUp().getAsBoolean() && !D_INTAKE_IR.supplier.getAsBoolean()) {
                return true;
            } return false;
        };
    }
    public BooleanSupplier readyToSourceFling() {
        return (BooleanSupplier) ()-> {
            if (m_operatorController.povUp().getAsBoolean() && D_INTAKE_IR.supplier.getAsBoolean()) {
                return true;
            } return false;
        };
    }
    public static BooleanSupplier handoffReady() {
        return (BooleanSupplier) () -> {
            if (!m_operatorController.povUp().getAsBoolean() && DriverStation.isTeleopEnabled() && RobotContainer.D_INTAKE_IR.isNoteInIntake() && RobotContainer.D_WRIST_DOCKING_LI.isWristDocked()) return true;
            return false;
        };
    }

  // The robot's subsystems

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureBindings();

    // CameraServer.startAutomaticCapture();
    
    //auto chooser <25
    //autoChooser.addOption("D4Note", "Distance4NoteR");
    //autoChooser.addOption("4Note", "BlueSubwoofer");
    autoChooser.addOption("3NoteAMP", "3NoteAMP");
    autoChooser.addOption("Center2", "TestCenter");
    //autoChooser.addOption("3NoteAMPStage", "AMP3Note");
    autoChooser.addOption("3NoteStage", "3NoteStage");
    autoChooser.addOption("2NoteCenter", "2NoteNTX");
    autoChooser.addOption("1Note", "1Note");
    //autoChooser.addOption("1NoteAMPLeave", "");
    //autoChooser.addOption("No Auto", "None");
    //autoChooser.addOption("Side", "Long-Auto");
    autoChooser.addOption("4NoteAuto", "5NoteAutoTest");
    //autoChooser.addOption("3NoteAmpDistance", "AMPSideDistance");
    //autoChooser.addOption("Subwoofer3Note", "Non-StageSubwoofer");
    //autoChooser.addOption("Test", "TestGreg");
    //autoChooser.addOption("RylandTest", "RylandTest");
    SmartDashboard.putData("Autonomous Selector", autoChooser);
    //songChooser.addOption("He is a Pirate", "pirate.chrp");
    //songChooser.addOption("Stardust Crusaders", "stardust.chrp");
    //songChooser.addOption("Il Vento D'oro", "giorno.chrp");
    //songChooser.addOption("Megalovania", "sans.chrp");
    //songChooser.addOption("Rocket Man", "rocketman.chrp");
    //songChooser.addOption("Daft Punk", "lit.chrp");
    //songChooser.addOption("Starboy", "starboy.chrp");

    SmartDashboard.putData("Intro Music", songChooser);

    SmartDashboard.putNumber("Aiming Adjustment", 0);
    

    s_drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> s_drive.drive(
                -MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_operatorController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_operatorController.getRightX(), OIConstants.kDriveDeadband),
            true, .02),
            
        s_drive)
    ); 
    
    NamedCommands.registerCommand("autoShot", s_shooter.manualSpinShooters().andThen(new WaitCommand(.5)).andThen(s_intake.autoReleaseNote()));
    NamedCommands.registerCommand("stopShooter", s_shooter.manualStopShooters());
    NamedCommands.registerCommand("releaseNote", s_intake.autoReleaseNote());
    NamedCommands.registerCommand("groundIntakeNote", s_intake.groundPickup()
                .alongWith(s_wrist.restWrist())
                .andThen(s_wrist.dockWrist()));
    NamedCommands.registerCommand("handoffNote", s_intake.handoff()
        .andThen(
            s_shooter.achieveClearance()
        ));
    NamedCommands.registerCommand("distanceShot", s_drive.angleToSpeakerUsingAprilTag()
        .alongWith(s_pivot.aimWithLimelight())
        .alongWith(s_shooter.manualSpinShooters())
    );
    // );
    // NamedCommands.registerCommand("distanceShot", s_shooter.manualSpinShooters()
    // .alongWith(s_pivot.aimWithLimelight())
    // .alongWith(s_drive.angleToSpeakerUsingAprilTag())
    // .alongWith(new WaitCommand(1)).alongWith(s_intake.autoReleaseNote()));

    NamedCommands.registerCommand("releaseWhenAimed", s_intake.autoReleaseNote());
    

    falconChoir.addInstrument(s_drive.talonFrontLeft);
    falconChoir.addInstrument(s_drive.talonFrontRight);
    falconChoir.addInstrument(s_drive.talonRearLeft);
    falconChoir.addInstrument(s_drive.talonRearRight);
    falconChoir.addInstrument(s_intake.m_shooterIntake);
    falconChoir.addInstrument(s_shooter.m_topShooter);
    falconChoir.addInstrument(s_shooter.m_bottomShooter);

  }
 /* Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureBindings() {
    // new Trigger(readyToShoot())
    //     .onTrue(s_shooter.spinShooters());
    // m_operatorController.rightBumper()
    //     .onTrue(
    //         s_pivot.aimWithLimelight()
    //     .alongWith(
    //         s_drive.angleToSpeakerUsingAprilTag()
    //     )
    //     .alongWith(
    //         s_intake.clearNote()
    //     ));
    
    m_operatorController.rightBumper()
        .onTrue(
            s_shooter.spinShooters()
        .alongWith(
            s_pivot.aimWithLimelight()
        )
        .alongWith(
            s_drive.angleToSpeakerUsingAprilTag()
        )
        );
    

    // new Trigger(RobotContainer.readyToShoot())
    //     .onTrue(
    //         s_pivot.aimWithLimelight()
    //     .alongWith(
    //         s_drive.angleToSpeakerUsingAprilTag()
    //     ));
    // new Trigger(D_INTAKE_IR.supplier)
    //     .whileTrue(s_wrist.dockWrist());
    //Handoff trigger
    // new Trigger(readyToSource())
    //     .onTrue(
    //         s_wrist.sourceFling()
    //     .alongWith(s_intake.groundPickup())
    //     .alongWith(s_shooter.controlledShot())
    //     );
    // new Trigger(readyToSourceFling())
    //     .onTrue(s_intake.handoff()
    //     .alongWith(s_shooter.controlledShot())
    //     );
    
    new Trigger(handoffReady())
        .onTrue(
            s_intake.handoff()
        .andThen(
            s_shooter.achieveClearance()
        )
        // .andThen(s_shooter.controlledShot())
    );

    /*
    below was an idea for the handoff system to correct itself if the note
    got stuck between the wrist and pivot. Unfortunately, I gave up because
    as an independent trigger of any other commands,
    in order for this to work it would require the robot to track if the note
    was released properly since it was intook. No thanks, this was all in one night.

    new Trigger(s_intake.noteUnknown())
         .onTrue(
             s_intake.correctTransfer()
         );
    */

    m_secondaryController.yButton
        .onTrue(s_intake.run(s_intake::outwardGroundIntake));
    // m_secondaryController.aButton
    //     .onTrue(s_elevator.run(s_elevator::lowerElevator));


    m_operatorController.povLeft()
        .onTrue(s_leftHanging.manualRaiseLeft().alongWith(s_rightHanging.manualRaiseRight()))
        .onFalse(s_leftHanging.stopLeft().alongWith(s_rightHanging.stopRight()));
    m_operatorController.povRight()
        .onTrue(s_leftHanging.manualLowerLeft().alongWith(s_rightHanging.manualLowerRight()))
        .onFalse(s_leftHanging.stopLeft().alongWith(s_rightHanging.stopRight()));
    m_operatorController.start()
        .onTrue(
            s_shooter.manualSpinShooters()
        );
    // m_operatorController.x()
    //     .whileTrue(
    //         s_pivot.run(s_pivot::raisePivot)
    //     ).onFalse(s_pivot.run(s_pivot::holdPivot));
    // m_operatorController.b()
    //     .whileTrue(
    //         s_pivot.run(s_pivot::lowerPivot)
    //     )
    //     .onFalse(s_pivot.run(s_pivot::holdPivot));
    m_operatorController.b()
        .whileTrue(s_shooter.controlledShot());

    m_operatorController.y()
        .onTrue(
            s_wrist.dockWrist()
        );
    //score AMP
    m_operatorController.leftBumper()
        .whileTrue(
            s_elevator.raiseToAmp()
        .alongWith(
            s_shooter.shootIntoAMP())
        .alongWith(
            s_pivot.pivotAtAMP())
        .alongWith(
            s_intake.clearNote()
        ));
        //left trigger is used to reshuffle
    // m_operatorController.leftTrigger()
    //     .onTrue(
    //         s_shooter.manualSpinShooters()
    //     .alongWith(
    //         s_pivot.pivotAtStage()
    //     ))
    //     .onFalse(s_intake.releaseNote());
    //release note/shoot notes
    m_operatorController.rightTrigger()
        .whileTrue(s_intake.releaseNote());
    //ground intake
    m_operatorController.a()
        .onTrue(
            s_wrist.restWrist()
                .alongWith(s_intake.groundPickup())
        .andThen(
            s_wrist.dockWrist()
                .alongWith(s_intake.holdNote())
        ));
    //reset handoff button
    // used to be .povdown
    m_operatorController.leftTrigger()
        .onTrue(s_intake.reverseHandoff());

    m_secondaryController.rtButton
        .whileTrue(s_rightHanging.manualRaiseRight())
        .onFalse(s_rightHanging.stopRight());
    m_secondaryController.ltButton
        .whileTrue(s_leftHanging.manualRaiseLeft())
        .onFalse(s_leftHanging.stopLeft());
    m_secondaryController.lbButton
        .whileTrue(s_leftHanging.manualLowerLeft())
        .onFalse(s_leftHanging.stopLeft());
    m_secondaryController.rbButton
        .whileTrue(s_rightHanging.manualLowerRight())
        .onFalse(s_rightHanging.stopRight());

    m_secondaryController.aButton
        .whileTrue(s_shooter.feedShot());
    m_secondaryController.bButton
        .whileTrue(s_shooter.yeetShot());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    selectedAuto = autoChooser.getSelected();
    if (selectedAuto == null) {
        selectedAuto = "1Note";
    }
    System.out.println("Selected Auto: " + selectedAuto);
    
    return new PathPlannerAuto(selectedAuto);
  }
}

