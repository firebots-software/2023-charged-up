package frc.robot.commandGroups;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmToDegree;
import frc.robot.commands.JankArmToTicks;
import frc.robot.commands.MoveToTag;
import frc.robot.commands.RetractArmCmd;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.ArmConstants;

public class MoveAndScore extends SequentialCommandGroup {
    public static int LEFT_POS = -1;
    public static int RIGHT_POS = 1;
    public static int MIDDLE_POS = 0;

    public static int LOW_LEVEL = 0;
    public static int MID_LEVEL = 1;
    public static int HIGH_LEVEL = 2;

    private static final HashMap<Integer, double[]> cubeLevelToDegTickies = new HashMap<>() {
        {
            put(0, new double[] { ArmConstants.LOW_GOAL_FRONT_DEG, ArmConstants.LOW_GOAL_DIST_TICKIES });
            put(1, new double[] { ArmConstants.MID_CUBE_FRONT_DEG, ArmConstants.MID_CUBE_DIST_TICKIES });
            put(2, new double[] { ArmConstants.HIGH_CUBE_FRONT_DEG, ArmConstants.HIGH_CUBE_DIST_TICKIES });
        }
    };

    private static final HashMap<Integer, double[]> coneLevelToDegTickies = new HashMap<>() {
        {
            put(0, new double[] { ArmConstants.LOW_GOAL_FRONT_DEG, ArmConstants.LOW_GOAL_DIST_TICKIES });
            put(1, new double[] { ArmConstants.MID_CONE_FRONT_DEG, ArmConstants.MID_CONE_DIST_TICKIES });
            put(2, new double[] { ArmConstants.HIGH_CONE_FRONT_DEG, ArmConstants.HIGH_CONE_DIST_TICKIES });
        }
    };

    public MoveAndScore(int pos, int level, SwerveSubsystem swerveSubsystem, ArmSubsystem arm, ClawSubsystem claw,
            boolean auton) {
        boolean cone = pos != 0;
        double[] deginches = cone ? coneLevelToDegTickies.get(level) : cubeLevelToDegTickies.get(level);
        addCommands(
            new RetractArmCmd(arm),
            new ArmToDegree(arm, deginches[0]),
            new JankArmToTicks(deginches[1], arm),
            new ToggleClaw(true, claw),
            new WaitCommand(0.2),
            new RetractArmCmd(arm)
        );
    }

    /**
     * Move and score a loaded pieces
     * 
     * @param pos             -1 if to the left, 0 if in the middle, 1 if to the
     *                        right
     * @param level           0 for bottom, 1 for middle, 2 for top
     * @param cone            true if a cone is loaded, false if a cube is loaded
     * @param swerveSubsystem
     * @param arm
     * @param claw
     */
    public MoveAndScore(int pos, int level, SwerveSubsystem swerveSubsystem, ArmSubsystem arm, ClawSubsystem claw) {
        boolean cone = pos != 0;

        MoveToTag mtt = new MoveToTag(pos, swerveSubsystem);
        double[] deginches = cone ? coneLevelToDegTickies.get(level) : cubeLevelToDegTickies.get(level);
        addCommands(
                new RetractArmCmd(arm),
                mtt,
                new ArmToDegree(arm, -1 * Math.abs(deginches[0])),
                new JankArmToTicks(deginches[1], arm),
                new ToggleClaw(claw)
        // new ToggleClaw(true, claw)
        );
    }

}
