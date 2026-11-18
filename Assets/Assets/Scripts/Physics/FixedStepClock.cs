namespace BenchToBanner.Physics
{
    /// <summary>
    /// Turns variable frame times into a whole number of fixed simulation steps.
    /// The ball is always stepped with config.TimeStep, never with the frame delta, so the
    /// trajectory is the same at 30, 60 or 144 fps. Alpha is for render interpolation only.
    /// </summary>
    public sealed class FixedStepClock
    {
        public readonly double StepSeconds;
        public readonly double MaxFrameSeconds;
        private double _accumulator;

        public FixedStepClock(double stepSeconds, double maxFrameSeconds = 0.25)
        {
            StepSeconds = stepSeconds;
            MaxFrameSeconds = maxFrameSeconds;
        }

        /// <summary>Feed one frame's elapsed time; returns how many fixed steps to run now.</summary>
        public int Advance(double frameSeconds)
        {
            if (frameSeconds < 0.0) frameSeconds = 0.0;
            if (frameSeconds > MaxFrameSeconds) frameSeconds = MaxFrameSeconds; // avoid spiral of death
            _accumulator += frameSeconds;

            int steps = 0;
            while (_accumulator >= StepSeconds)
            {
                _accumulator -= StepSeconds;
                steps++;
            }
            return steps;
        }

        /// <summary>0..1 progress between the previous and current simulation state (render only).</summary>
        public double Alpha { get { return _accumulator / StepSeconds; } }

        public void Reset() { _accumulator = 0.0; }
    }
}
