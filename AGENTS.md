# ECE 598 Humanoid Bodyguard

## Environment and scope
- Host: Ubuntu 24.04. ROS 2 Jazzy runs in Grove-G1's Docker development container.
- Simulation: MuJoCo with Unitree G1. Navigation: Nav2. Custom bodyguard ROS packages are planned.
- Week 1 covers simulator validation, fixed-goal navigation, ROS interface inspection, and diagnostics only. Do not implement person tracking or the bodyguard Behavior Tree.

## Safety and architecture
- Preserve Grove-G1's existing locomotion, safety, hardware, and DDS architecture.
- Never modify low-level G1 motor control, `rt/lowcmd`, or the balance policy without explicit approval.
- Never automatically start, stop, reset, or reconfigure running simulations or controllers.
- Never run Docker commands, `sudo`, destructive shell operations, `git reset`, `git clean`, commits, or pushes without explicit approval.

## Implementation
- Prefer new bodyguard-specific ROS packages over changes to upstream Grove-G1 packages.
- Confirm actual ROS topics, message types, frame IDs, and installed APIs before writing integrations. If inspection requires a restricted action, request approval first.
- Clearly label ground-truth simulation inputs separately from perception outputs in code, interfaces, and documentation.
- Change only files relevant to the requested milestone.
- Add tests for newly implemented logic.

## Verification and reporting
- Report actual verification results, including failures and checks not run; never imply unperformed checks passed.
- Explain changes and provide manual ROS 2 verification commands appropriate to the milestone. Providing commands does not authorize running restricted actions.
