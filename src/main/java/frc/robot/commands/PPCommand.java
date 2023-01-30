

  /** Creates a new PPCommand. */
  public PPCommand(DrivetrainSubsystem drivetrainSubsystem, 
  PoseEstimationSubsystem poseEstimationSubsystem, 
  PathPlannerTrajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies

    m_trajectory = trajectory;
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimationSubsystem = poseEstimationSubsystem;
    m_pose = m_poseEstimationSubsystem::getPose;
    
    addRequirements(drivetrainSubsystem, poseEstimationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xController = new PIDController(3.5, 0, 0);
    m_yController = new PIDController(3.5, 0, 0);
    m_thetaController = new ProfiledPIDController(8, 0, 0,
        new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2));
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //Reset PID controller error and set initial pose.
    m_xController.reset();
    m_yController.reset();
    m_thetaController.reset(m_poseEstimationSubsystem.getPose().getRotation().getRadians(), 0.0);

    //Add PID controllers to the drive controller
    m_controller = new HolonomicDriveController(m_xController, m_yController, m_thetaController);
    m_controller.setEnabled(true);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Calculate our desired chassis speeds to follow the trajectory using our PID
    //and drive controllers
    double curTime = m_timer.get();
    var desiredState = (PathPlannerState) m_trajectory.sample(curTime);
    var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, desiredState.holonomicRotation);


    var updatedTargetChassisSpeeds = new ChassisSpeeds(targetChassisSpeeds.vxMetersPerSecond,
        targetChassisSpeeds.vyMetersPerSecond, targetChassisSpeeds.omegaRadiansPerSecond);

    m_drivetrainSubsystem.drive(updatedTargetChassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_drivetrainSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
