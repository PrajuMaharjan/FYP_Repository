// var cfg = BallPhysicsConfig.Default;
// var start  = new Vec3d(-10, 3, 0);
// var target = new Vec3d(5, cfg.Radius, 2);

// // by flight time (serve)
// LaunchSolution s1 = LaunchSolver.SolveByFlightTime(start, target, 1.2, cfg);
// // by speed (spike)
// LaunchSolution s2 = LaunchSolver.SolveByChordSpeed(start, target, 24.0, cfg);
// // by arc height (set)
// LaunchSolution s3 = LaunchSolver.SolveByApexRise(start, target, 3.0, cfg);

// if (!s1.Converged) { /* shot not reachable, handle it */ }