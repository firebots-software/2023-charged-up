package frc.robot.commandGroups;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmToDegree;
import frc.robot.commands.ExtendToCmd;
import frc.robot.commands.MoveToTag;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.ArmConstants;

public class MoveAndScore extends SequentialCommandGroup {

    private static final HashMap<Integer, double[]> cubeLevelToDegInches = new HashMap<>(){{
        put(0, new double[]{ArmConstants.LOW_GOAL_FRONT_DEG, ArmConstants.LOW_GOAL_DIST_IN});
        put(1, new double[]{ArmConstants.MID_CUBE_FRONT_DEG, ArmConstants.MID_CUBE_DIST_IN});
        put(2, new double[]{ArmConstants.HIGH_CUBE_FRONT_DEG, ArmConstants.HIGH_GOAL_DIST_IN});
    }};

    private static final HashMap<Integer, double[]> coneLevelToDegInches = new HashMap<>(){{
        put(0, new double[]{ArmConstants.LOW_GOAL_FRONT_DEG, ArmConstants.LOW_GOAL_DIST_IN});
        put(1, new double[]{ArmConstants.MID_CONE_FRONT_DEG, ArmConstants.MID_CONE_DIST_IN});
        put(2, new double[]{ArmConstants.HIGH_CONE_FRONT_DEG, ArmConstants.HIGH_GOAL_DIST_IN});
    }};

    /**
     * Move and score a loaded pieces
     * @param pos -1 if to the left, 0 if in the middle, 1 if to the right
     * @param level 0 for bottom, 1 for middle, 2 for top 
     * @param cone true if a cone is loaded, false if a cube is loaded
     * @param swerveSubsystem
     * @param arm
     * @param claw
     */
    public MoveAndScore(int pos, int level, boolean cone, SwerveSubsystem swerveSubsystem, ArmSubsystem arm, ClawSubsystem claw) {
        double[] deginches = cone ? coneLevelToDegInches.get(level) : cubeLevelToDegInches.get(level);
        addCommands(
            new ParallelCommandGroup(
                new MoveToTag(pos, swerveSubsystem),
                new ArmToDegree(arm, deginches[0]),
                new ExtendToCmd(arm, deginches[1])
            ),
            new ToggleClaw(true, claw)
        );
    }
    
}
