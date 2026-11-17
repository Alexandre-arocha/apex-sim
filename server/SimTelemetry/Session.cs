namespace SimTelemetry;

/// <summary>Uma volta completa: instantes (s) em que cruzou a linha de largada.</summary>
public readonly record struct Lap(int Number, double StartS, double EndS)
{
    public double DurationS => EndS - StartS;
}

/// <summary>Um canal amostrado: tempo (s) e valor, do mesmo tamanho.</summary>
public sealed record Series(double[] Time, double[] Value);

/// <summary>
/// Sessao de telemetria: guarda as amostras recebidas (100 Hz; a amostra i e do instante
/// (i + 1) / 100 s) e responde consultas sobre elas. Seguro para uso concorrente.
/// </summary>
/// <remarks>
/// O quadro de telemetria nao leva posicao, entao as voltas saem da distancia integrada a partir
/// da velocidade (regra do trapezio) contra o comprimento da pista, informado na criacao.
/// </remarks>
public sealed class Session : ITelemetrySink
{
    public const double SampleRateHz = 100.0;

    private static readonly string[] ChannelNames = ["speed", "rpm", "throttle", "brake", "gear"];

    private readonly object gate = new();
    private readonly List<TelemetrySample> samples = [];

    public Session(double trackLengthM)
    {
        if (!double.IsFinite(trackLengthM) || trackLengthM <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(trackLengthM), "Comprimento invalido.");
        }

        TrackLengthM = trackLengthM;
    }

    public double TrackLengthM { get; }

    public int Count
    {
        get
        {
            lock (gate)
            {
                return samples.Count;
            }
        }
    }

    /// <summary>Guarda o quadro se for de telemetria; devolve false para qualquer outro.</summary>
    public bool Add(CanFrame frame)
    {
        if (!TelemetrySample.TryDecode(frame, out TelemetrySample sample))
        {
            return false;
        }

        AddSample(sample);
        return true;
    }

    /// <summary>Acrescenta uma amostra ja decodificada (a i-esima e do instante (i + 1) / 100 s).</summary>
    public void AddSample(TelemetrySample sample)
    {
        lock (gate)
        {
            samples.Add(sample);
        }
    }

    /// <summary>As amostras a partir do indice start (0 = todas), como copia.</summary>
    public TelemetrySample[] SamplesFrom(int start)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(start);
        lock (gate)
        {
            return start >= samples.Count ? [] : samples.GetRange(start, samples.Count - start).ToArray();
        }
    }

    /// <summary>Voltas completas, na ordem. A primeira conta desde o inicio da sessao.</summary>
    public IReadOnlyList<Lap> Laps()
    {
        TelemetrySample[] snapshot = Snapshot();
        var laps = new List<Lap>();
        double distance = 0;
        double previousTime = 0;
        double previousSpeed = 0;
        double lapStart = 0;
        double finishLine = TrackLengthM;

        for (int i = 0; i < snapshot.Length; i++)
        {
            double time = TimeOf(i);
            double speed = snapshot[i].SpeedMps;
            double before = distance;
            distance += 0.5 * (previousSpeed + speed) * (time - previousTime);

            while (distance >= finishLine)
            {
                double fraction = (finishLine - before) / (distance - before);
                double crossing = previousTime + fraction * (time - previousTime);
                laps.Add(new Lap(laps.Count + 1, lapStart, crossing));
                lapStart = crossing;
                before = finishLine;
                previousTime = crossing;
                finishLine += TrackLengthM;
            }

            previousTime = time;
            previousSpeed = speed;
        }

        return laps;
    }

    /// <summary>Um canal inteiro, nas amostras originais. Nomes: speed (m/s), rpm, throttle, brake, gear.</summary>
    public Series Channel(string name)
    {
        Func<TelemetrySample, double> pick = Picker(name);
        TelemetrySample[] snapshot = Snapshot();
        var time = new double[snapshot.Length];
        var value = new double[snapshot.Length];
        for (int i = 0; i < snapshot.Length; i++)
        {
            time[i] = TimeOf(i);
            value[i] = pick(snapshot[i]);
        }

        return new Series(time, value);
    }

    /// <summary>
    /// Reamostra um canal numa grade uniforme de rateHz, de from a to (por padrao a sessao inteira;
    /// os limites sao cortados ao que existe). Interpola linearmente, menos a marcha, que mantem o
    /// valor anterior.
    /// </summary>
    public Series Resample(string name, double rateHz, double? from = null, double? to = null)
    {
        if (!double.IsFinite(rateHz) || rateHz <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(rateHz));
        }

        Series source = Channel(name);
        if (source.Time.Length == 0)
        {
            return new Series([], []);
        }

        double first = source.Time[0];
        double last = source.Time[^1];
        double start = Math.Max(from ?? first, first);
        double end = Math.Min(to ?? last, last);
        if (end < start)
        {
            return new Series([], []);
        }

        bool step = name == "gear";
        int count = (int)Math.Floor((end - start) * rateHz + 1e-9) + 1;
        var time = new double[count];
        var value = new double[count];
        for (int k = 0; k < count; k++)
        {
            double t = start + k / rateHz;
            double position = (t - first) * SampleRateHz;
            int i = Math.Clamp((int)Math.Floor(position + 1e-9), 0, Math.Max(source.Value.Length - 2, 0));
            double fraction = Math.Clamp(position - i, 0, 1);
            time[k] = t;
            value[k] = source.Value.Length == 1 || step
                ? source.Value[i]
                : source.Value[i] + fraction * (source.Value[i + 1] - source.Value[i]);
        }

        return new Series(time, value);
    }

    /// <summary>Um canal de uma volta (1 = primeira), reamostrado; o tempo conta desde o inicio da volta.</summary>
    public Series LapChannel(string name, int lapNumber, double rateHz)
    {
        IReadOnlyList<Lap> laps = Laps();
        if (lapNumber < 1 || lapNumber > laps.Count)
        {
            throw new ArgumentOutOfRangeException(nameof(lapNumber), $"A sessao tem {laps.Count} volta(s) completa(s).");
        }

        Lap lap = laps[lapNumber - 1];
        Series series = Resample(name, rateHz, lap.StartS, lap.EndS);
        return series with { Time = series.Time.Select(t => t - lap.StartS).ToArray() };
    }

    private static double TimeOf(int index) => (index + 1) / SampleRateHz;

    private static Func<TelemetrySample, double> Picker(string name) => name switch
    {
        "speed" => s => s.SpeedMps,
        "rpm" => s => s.Rpm,
        "throttle" => s => s.Throttle,
        "brake" => s => s.Brake,
        "gear" => s => s.Gear,
        _ => throw new ArgumentException($"Canal desconhecido '{name}'. Use: {string.Join(", ", ChannelNames)}.", nameof(name)),
    };

    private TelemetrySample[] Snapshot()
    {
        lock (gate)
        {
            return [.. samples];
        }
    }
}
