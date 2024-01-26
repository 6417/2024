package frc.fridolib;

public enum RobotPreset {
    TestChassisDrive(
        Tankdrive::FourFalcons(10, 11, 12, 13)
    ),
    TestChassisShooter(
        Tankdrive::FourFalcons(10, 11, 12, 13),
        Shooter::TwoFalcons(21, 22, inverted=22),
    ),
    Demogrogon(
        SwerveDrive::NeoSparkMax(ids..),
    )
}
