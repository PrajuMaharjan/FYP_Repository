namespace BenchToBanner.Physics
{
    /// <summary>
    /// FIVB indoor court. Net plane is X = 0; side A is X &lt; 0, side B is X &gt; 0.
    /// Ball centre is what the physics tracks, so floor contact is Y = ball radius.
    /// </summary>
    public sealed class CourtSpec
    {
        public readonly double Length;     // 18 m
        public readonly double Width;      // 9 m
        public readonly double NetHeight;  // 2.43 m men, 2.24 m women

        public double HalfLength { get { return Length * 0.5; } }
        public double HalfWidth { get { return Width * 0.5; } }

        public CourtSpec(double netHeight, double length = 18.0, double width = 9.0)
        {
            Length = length;
            Width = width;
            NetHeight = netHeight;
        }

        public static readonly CourtSpec Men = new CourtSpec(2.43);
        public static readonly CourtSpec Women = new CourtSpec(2.24);

        /// <summary>Boundary lines count as in (a ball touching the line is in).</summary>
        public bool IsInCourt(Vec3d p)
        {
            return System.Math.Abs(p.X) <= HalfLength && System.Math.Abs(p.Z) <= HalfWidth;
        }

        /// <summary>-1 for side A (X &lt; 0), +1 for side B.</summary>
        public int SideOf(Vec3d p) { return p.X < 0.0 ? -1 : 1; }
    }
}
