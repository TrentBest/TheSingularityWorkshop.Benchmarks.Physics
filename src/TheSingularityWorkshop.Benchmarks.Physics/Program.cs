using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;
using TheSingularityWorkshop.Physics.Collision;
using TheSingularityWorkshop.Physics.Numerics;
using TheSingularityWorkshop.Physics.Spatial;

BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly).Run(args);

[MemoryDiagnoser]
public class Vector3dBatchBenchmarks
{
    [Params(128, 4_096, 65_536)]
    public int Count { get; set; }

    private double[] _ax = [];
    private double[] _ay = [];
    private double[] _az = [];
    private double[] _bx = [];
    private double[] _by = [];
    private double[] _bz = [];
    private double[] _ox = [];
    private double[] _oy = [];
    private double[] _oz = [];
    private Vector3d[] _aosA = [];
    private Vector3d[] _aosB = [];
    private Vector3d[] _aosOutput = [];

    [GlobalSetup]
    public void Setup()
    {
        _ax = new double[Count]; _ay = new double[Count]; _az = new double[Count];
        _bx = new double[Count]; _by = new double[Count]; _bz = new double[Count];
        _ox = new double[Count]; _oy = new double[Count]; _oz = new double[Count];
        _aosA = new Vector3d[Count]; _aosB = new Vector3d[Count]; _aosOutput = new Vector3d[Count];

        for (var i = 0; i < Count; i++)
        {
            var n = i + 1d;
            _ax[i] = n * 0.25; _ay[i] = n * -0.5; _az[i] = n * 0.75;
            _bx[i] = n * -0.125; _by[i] = n * 0.375; _bz[i] = n * 0.625;
            _aosA[i] = new Vector3d(_ax[i], _ay[i], _az[i]);
            _aosB[i] = new Vector3d(_bx[i], _by[i], _bz[i]);
        }

        Vector3dBatch.Add(_ax, _ay, _az, _bx, _by, _bz, _ox, _oy, _oz);
        for (var i = 0; i < Count; i++)
            _aosOutput[i] = _aosA[i] + _aosB[i];
        for (var i = 0; i < Count; i++)
        {
            if (_ox[i] != _aosOutput[i].X || _oy[i] != _aosOutput[i].Y || _oz[i] != _aosOutput[i].Z)
                throw new InvalidOperationException("SoA and AoS addition results differ.");
        }
    }

    [Benchmark(Baseline = true)]
    public void PhysicsVector3dAoS()
    {
        for (var i = 0; i < Count; i++)
            _aosOutput[i] = _aosA[i] + _aosB[i];
    }

    [Benchmark]
    public void Vector3dBatchSoA() =>
        Vector3dBatch.Add(_ax, _ay, _az, _bx, _by, _bz, _ox, _oy, _oz);
}

public enum BroadphaseDistribution
{
    Sparse,
    Clustered,
    Dense
}

[MemoryDiagnoser]
public class SweepAndPruneBenchmarks
{
    [Params(128, 512, 2_048)]
    public int Count { get; set; }

    [Params(BroadphaseDistribution.Sparse, BroadphaseDistribution.Clustered, BroadphaseDistribution.Dense)]
    public BroadphaseDistribution Distribution { get; set; }

    private BroadphaseProxy[] _proxies = [];

    [GlobalSetup]
    public void Setup()
    {
        var random = new Random(1729);
        _proxies = new BroadphaseProxy[Count];
        for (var i = 0; i < Count; i++)
        {
            var (x, y, z, width) = Distribution switch
            {
                BroadphaseDistribution.Sparse => (i * 4d, random.NextDouble() * 100d, random.NextDouble() * 100d, 0.5d),
                BroadphaseDistribution.Clustered => (random.NextDouble() * 20d, random.NextDouble() * 20d, random.NextDouble() * 20d, 1.5d),
                _ => (random.NextDouble() * 2d, random.NextDouble() * 2d, random.NextDouble() * 2d, 2d)
            };
            _proxies[i] = new BroadphaseProxy(i, new AxisAlignedBounds(
                x, y, z, x + width, y + width, z + width));
        }

        var result = SweepAndPruneBroadphase.FindCandidates(_proxies);
        var expected = new List<BroadphasePair>();
        for (var i = 0; i < _proxies.Length; i++)
        for (var j = i + 1; j < _proxies.Length; j++)
        {
            if (!_proxies[i].Bounds.Intersects(_proxies[j].Bounds))
                continue;
            expected.Add(new BroadphasePair(
                Math.Min(_proxies[i].Id, _proxies[j].Id),
                Math.Max(_proxies[i].Id, _proxies[j].Id)));
        }
        expected.Sort(static (a, b) =>
        {
            var first = a.FirstId.CompareTo(b.FirstId);
            return first != 0 ? first : a.SecondId.CompareTo(b.SecondId);
        });
        if (!result.SequenceEqual(expected))
            throw new InvalidOperationException("Sweep-and-prune candidates differ from the brute-force AABB reference.");
    }

    [Benchmark]
    public int FindCandidates() => SweepAndPruneBroadphase.FindCandidates(_proxies).Count;
}
