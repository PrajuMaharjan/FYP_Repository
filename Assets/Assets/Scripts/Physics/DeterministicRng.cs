namespace BenchToBanner.Physics
{
    /// <summary>
    /// Small seeded random generator (xorshift64*). Used instead of System.Random / UnityEngine.Random
    /// so that "random" choices (like where a return shot is aimed) are identical on every run and
    /// every machine for the same seed. Same seed in, same sequence out.
    /// </summary>
    public sealed class DeterministicRng
    {
        private ulong _state;

        public DeterministicRng(ulong seed)
        {
            // One splitmix64 round so small seeds (1, 2, 3...) do not give similar early output.
            unchecked
            {
                ulong z = seed + 0x9E3779B97F4A7C15UL;
                z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
                z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
                z ^= z >> 31;
                _state = z != 0UL ? z : 0x9E3779B97F4A7C15UL;
            }
        }

        public ulong NextU64()
        {
            unchecked
            {
                _state ^= _state >> 12;
                _state ^= _state << 25;
                _state ^= _state >> 27;
                return _state * 2685821657736338717UL;
            }
        }

        /// <summary>Uniform in [0, 1).</summary>
        public double NextDouble()
        {
            return (NextU64() >> 11) * (1.0 / 9007199254740992.0);
        }

        /// <summary>Uniform in [min, max).</summary>
        public double Range(double min, double max)
        {
            return min + (max - min) * NextDouble();
        }
    }
}
