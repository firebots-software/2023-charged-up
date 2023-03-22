// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DockingConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commandGroups.ArmToGround;
import frc.robot.commandGroups.ChargeStation;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.ArmToDegree;
import frc.robot.commands.JankArmToTicks;
import frc.robot.commands.LimpArm;
import frc.robot.commands.MoveToTag;
import frc.robot.commands.RetractArmCmd;
import frc.robot.commands.ToggleClaw;
import frc.robot.commandGroups.MoveAndScore;
import frc.robot.commandGroups.MoveToCubeAndExtend;
import frc.robot.commandGroups.MoveToTargetAndExtend;
import frc.robot.commandGroups.PickupFromGround;
import frc.robot.commandGroups.PickupObjectFromHeight;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroHeadingCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ResetAutoBuilder;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ResetAutoBuilder.ChargeStationOptions;
import frc.robot.subsystems.ResetAutoBuilder.PiecesScoredOptions;
import frc.robot.subsystems.ResetAutoBuilder.StartingPositionOptions;

public class RobotContainer {

  // OI
  private Joystick driverPS4;
  private Joystick armJoystick;
  private Joystick numpad;

  private static SendableChooser<ResetAutoBuilder.StartingPositionOptions> startPosChooser = new SendableChooser<>();
  private static SendableChooser<ResetAutoBuilder.ChargeStationOptions> chargeStationChooser = new SendableChooser<>();
  private static SendableChooser<ResetAutoBuilder.PiecesScoredOptions> piecesChooser = new SendableChooser<>();

  // Subsystems
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final ArmSubsystem arm = ArmSubsystem.getInstance();
  private final ClawSubsystem claw = ClawSubsystem.getInstance();
  // PathPlanner
  private final Map<String, Command> eventMap = new HashMap<String, Command>() {{
      //DOCKING
      put("ChargeStationForward", new ChargeStation(swerveSubsystem, DockingConstants.DOCKING_SPEED));
      put("ChargeStationBackward", new ChargeStation(swerveSubsystem, -DockingConstants.DOCKING_SPEED));
      //VISION
      put("MoveToTarget", new MoveToTag(swerveSubsystem));
      put("MoveToCubeAndExtend", new MoveToCubeAndExtend(swerveSubsystem, arm));
      put("MoveToTargetAndExtend", new MoveToTargetAndExtend(swerveSubsystem, arm));
      // SCORING
      put("ScoreMidCube", 
        new MoveAndScore(MoveAndScore.MIDDLE_POS, MoveAndScore.MID_LEVEL, swerveSubsystem, arm, claw, true));
      put("ScoreHighCube", 
        new MoveAndScore(MoveAndScore.MIDDLE_POS, MoveAndScore.HIGH_LEVEL, swerveSubsystem, arm, claw, true));
      put("ScoreHighCone", new MoveAndScore(MoveAndScore.RIGHT_POS, MoveAndScore.HIGH_LEVEL, swerveSubsystem, arm, claw, true));
      // ARM
      put("RetractArmToMin", new RetractArmCmd(arm));
      put("PickupCube", new PickupFromGround(() -> true, arm, claw, true));
      put("ArmToGroundBack", new ArmToGround(() -> true, arm, true));
      put("ExtendArmToMax", new JankArmToTicks(304433, arm));
      put("ArmToHighCube", new ArmToDegree(arm, ArmConstants.HIGH_CUBE_FRONT_DEG));
      put("ArmToMidCube", new ArmToDegree(arm, ArmConstants.MID_CONE_FRONT_DEG));
      put("TuckArm", new ArmToDegree(arm, ArmConstants.MAX_RETRACTED_DEG));
      // CLAW
      put("ToggleClaw", new ToggleClaw(claw));
      put("CloseClaw", new ToggleClaw(false, claw));
  }};
  

  private final ResetAutoBuilder autoBuilder = new ResetAutoBuilder(
      swerveSubsystem::getPose,
      swerveSubsystem::resetOdometry,
      DriveConstants.kDriveKinematics,
      new PIDConstants(Constants.AutonConstants.kPDriving, Constants.AutonConstants.kIDriving,
          Constants.AutonConstants.kDDriving),
      new PIDConstants(Constants.AutonConstants.kPTurning, Constants.AutonConstants.kITurning,
          Constants.AutonConstants.kDTurning),
      swerveSubsystem::setModuleStates,
      eventMap,
      true,
      swerveSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.driverPS4 = new Joystick(Constants.OI.DRIVER_PS4_PORT);
    this.armJoystick = new Joystick(Constants.OI.ARM_JOYSTICK_PORT);
    this.numpad = new Joystick(Constants.OI.NUMPAD_PORT);

    swerveSubsystem.resetEncoders();
    swerveSubsystem.zeroHeading();

    // Configure the button bindings
    configureButtonBindings();

    initializeAuton();
  }

  private void configureButtonBindings() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverPS4.getRawAxis(1),
        () -> -driverPS4.getRawAxis(0),
        () -> -driverPS4.getRawAxis(2),
        () -> driverPS4.getRawAxis(4) > -0.75 ? 1 : (driverPS4.getRawAxis(3) > -0.75 ? 0.15 : 0.50),

        () -> !driverPS4.getRawButton(Constants.OI.SQUARE_BUTTON_PORT)));

    arm.setDefaultCommand(new ArmJoystickCmd(
      arm,
        () -> armJoystick.getRawAxis(0) * (armJoystick.getRawButton(3) ? 0.2 : 0.5),
        () -> -armJoystick.getRawAxis(1) * (armJoystick.getRawButton(3) ? 0.3 : 0.5)));
    
    for (int button = 1; button < 10; button++) {
      new JoystickButton(numpad, button).whileTrue(new MoveAndScore(((button-1) % 3) - 1, (button-1) / 3, swerveSubsystem, arm, claw));
    }

    

    final Trigger armToDegree = new JoystickButton(driverPS4, Constants.OI.X_BUTTON_PORT);
    armToDegree.whileTrue(new PickupFromGround(() -> arm.getRotationDegrees() > 0, arm, claw));
    
    final Trigger tri = new JoystickButton(driverPS4, Constants.OI.TRIANGLE_BUTTON_PORT);
    tri.whileTrue(new PickupObjectFromHeight(swerveSubsystem, arm, claw));

    final Trigger to90 = new JoystickButton(driverPS4, Constants.OI.R1_BUTTON_PORT);
    to90.whileTrue(new ArmToDegree(arm, () -> 90d));

    final Trigger damageControl = new JoystickButton(driverPS4, Constants.OI.CIRCLE_BUTTON_PORT);
    damageControl.onTrue(new ZeroHeadingCmd(swerveSubsystem));

    final Trigger thing = new JoystickButton(driverPS4, Constants.OI.SQUARE_BUTTON_PORT);
    thing.whileTrue(new ChargeStation(swerveSubsystem, -2.0));

    final Trigger limpArm = new JoystickButton(driverPS4, Constants.OI.PS_SHARE_BUTTON_PORT);
    limpArm.whileTrue(new LimpArm(arm));

    final Trigger clawToggle = new JoystickButton(armJoystick, 1);
    clawToggle.onTrue(new ToggleClaw(claw));

    final Trigger goUp = new JoystickButton(armJoystick, 6);
    goUp.onTrue(new RetractArmCmd(arm).andThen(new ArmToDegree(arm, 0)));

    final Trigger shelfPickupAngle = new JoystickButton(armJoystick, 4);
    shelfPickupAngle.whileTrue(new ArmToDegree(arm, ArmConstants.SHELF_PICKUP_DEG));
  }
  // Changing the R2 axis range from [-1, 1] to [0, 1] because we are using
  // this value as a decimal to multiply and control the speed of the robot.

  public static void initializeAuton() {
    startPosChooser.setDefaultOption("Just Score", StartingPositionOptions.JUST_SCORE);
    startPosChooser.addOption("Top", StartingPositionOptions.TOP);
    startPosChooser.addOption("Middle", StartingPositionOptions.MIDDLE);
    startPosChooser.addOption("Bottom", StartingPositionOptions.BOTTOM);

    piecesChooser.setDefaultOption("1 Piece Scoring", PiecesScoredOptions.ONE);
    piecesChooser.addOption("2 Piece Scoring", PiecesScoredOptions.TWO);

    chargeStationChooser.setDefaultOption("Charge Station", ChargeStationOptions.CHARGE);
    chargeStationChooser.addOption("No Charge Station", ChargeStationOptions.NO_CHARGE);

    SmartDashboard.putData("Auton: Starting Position Options", startPosChooser);
    SmartDashboard.putData("Auton: Number of Pieces Options", piecesChooser);
    SmartDashboard.putData("Auton: Charge Station Options", chargeStationChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AUTONS_MAP.get(ResetAutoBuilder.hashAuton(startPosChooser.getSelected(), piecesChooser.getSelected(),
        chargeStationChooser.getSelected()));
  }

  int count = 0;
  public void printAuton() {
    count++;
    SmartDashboard.putString("selected auton", getAutonomousCommand().getName());
    SmartDashboard.putNumber("count", count);
  }

  public final Map<Integer, Command> AUTONS_MAP = new HashMap<>() {
    {
      // JUST SCORE
      put(0, new MoveAndScore(0, 2, swerveSubsystem, arm, claw, true));

      // TOP
      put(ResetAutoBuilder.hashAuton(StartingPositionOptions.TOP, PiecesScoredOptions.ONE,
          ChargeStationOptions.NO_CHARGE),
          autoBuilder.autoFromPath("Top 1"));
      put(ResetAutoBuilder.hashAuton(StartingPositionOptions.TOP, PiecesScoredOptions.ONE, ChargeStationOptions.CHARGE),
          autoBuilder.autoFromPath("Top Charge 1"));
      put(ResetAutoBuilder.hashAuton(StartingPositionOptions.TOP, PiecesScoredOptions.TWO,
          ChargeStationOptions.NO_CHARGE),
          autoBuilder.autoFromPath("Top 2"));
      put(ResetAutoBuilder.hashAuton(StartingPositionOptions.TOP, PiecesScoredOptions.TWO, ChargeStationOptions.CHARGE),
          autoBuilder.autoFromPath("Top Charge 2"));

      // MIDDLE
      // "Middle 1" does not exist (charge station obstructs leaving and then coming back w/pathplanner)
      put(ResetAutoBuilder.hashAuton(StartingPositionOptions.MIDDLE, PiecesScoredOptions.ONE,
          ChargeStationOptions.CHARGE), // Middle Charge 1
          new MoveAndScore(0, 1, swerveSubsystem, arm, claw, true).andThen(new ChargeStation(swerveSubsystem, -2.5)));
      // "Middle 2" does not exist (charge station gets in the way of accurate pickup)
      // "Middle Charge 2" does not exist (charge station gets in the way of accurate pickup)

      // BOTTOM
      put(ResetAutoBuilder.hashAuton(StartingPositionOptions.BOTTOM, PiecesScoredOptions.ONE,
          ChargeStationOptions.NO_CHARGE),
          autoBuilder.autoFromPath("Bottom 1"));
      put(ResetAutoBuilder.hashAuton(StartingPositionOptions.BOTTOM, PiecesScoredOptions.ONE,
          ChargeStationOptions.CHARGE),
          autoBuilder.autoFromPath("Bottom Charge 1"));
      // "Bottom 2" does not exist (wire gets in the way of accurate pickup)
      // "Bottom Charge 2" does not exist (wire gets in the way of accurate pickup)
    }
  };
}