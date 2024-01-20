package frc.robot.commands.clawjoint;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawJoint;

public class HomeClawPosition extends CommandBase {
    private final ClawJoint clawJoint;

    private static final double HOME_ANGLE = .31; // Rotations

    public HomeClawPosition(ClawJoint clawJoint) {
        this.clawJoint = clawJoint;
        addRequirements(clawJoint);
    }

    @Override
    public void execute() {
        clawJoint.setAngle(HOME_ANGLE, false);
    }
}
