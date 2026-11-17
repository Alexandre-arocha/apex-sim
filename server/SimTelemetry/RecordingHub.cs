namespace SimTelemetry;

/// <summary>
/// Recebe os quadros do TCP e os entrega a sessao ativa, que fica em memoria (consulta imediata)
/// e e gravada no banco em lotes: a cada 250 ms, ao trocar de sessao e ao fechar. Sem sessao ativa os
/// quadros sao descartados (e contados).
/// </summary>
public sealed class RecordingHub : ITelemetrySink, IDisposable
{
    private readonly SessionStore store;
    private readonly object gate = new();
    private readonly CancellationTokenSource stop = new();
    private readonly Task flusher;
    private Session? active;
    private long activeId;
    private int flushed;
    private long dropped;
    private long gapsFilled;
    private long staleSamples;

    public RecordingHub(SessionStore store, TimeSpan? flushEvery = null)
    {
        this.store = store ?? throw new ArgumentNullException(nameof(store));
        TimeSpan period = flushEvery ?? TimeSpan.FromMilliseconds(250);
        flusher = Task.Run(() => FlushLoop(period, stop.Token));
    }

    /// <summary>Id da sessao que esta gravando, ou null.</summary>
    public long? ActiveId
    {
        get
        {
            lock (gate)
            {
                return active is null ? null : activeId;
            }
        }
    }

    /// <summary>Quadros de telemetria que chegaram sem sessao ativa.</summary>
    public long DroppedFrames => Interlocked.Read(ref dropped);

    /// <summary>Amostras repetidas para tapar perdas (a sequencia pulou).</summary>
    public long GapsFilled => Interlocked.Read(ref gapsFilled);

    /// <summary>Amostras ignoradas por chegarem com numero de sequencia ja usado.</summary>
    public long StaleSamples => Interlocked.Read(ref staleSamples);

    /// <summary>
    /// Entrega a amostra de numero sequence a sessao ativa. Se a sequencia pulou, repete a ultima
    /// amostra para cada numero que faltou; se ja foi usada, ignora. Pensado para um unico emissor por
    /// vez (a ponte do broker); false se nao ha sessao ativa ou a amostra foi ignorada.
    /// </summary>
    public bool AddSample(long sequence, TelemetrySample sample)
    {
        Session? target;
        lock (gate)
        {
            target = active;
        }

        if (target is null)
        {
            Interlocked.Increment(ref dropped);
            return false;
        }

        int count = target.Count;
        if (sequence < count)
        {
            Interlocked.Increment(ref staleSamples);
            return false;
        }

        TelemetrySample held = count > 0 ? target.SamplesFrom(count - 1)[0] : sample;
        for (long missing = count; missing < sequence; missing++)
        {
            target.AddSample(held);
            Interlocked.Increment(ref gapsFilled);
        }

        target.AddSample(sample);
        return true;
    }

    /// <summary>Abre uma sessao nova e a torna a ativa (a anterior e gravada antes e deixa de gravar).</summary>
    public long Start(string name, double trackLengthM)
    {
        lock (gate)
        {
            FlushLocked();
            long id = store.CreateSession(name, trackLengthM);
            active = new Session(trackLengthM);
            activeId = id;
            flushed = 0;
            return id;
        }
    }

    public bool Add(CanFrame frame)
    {
        Session? target;
        lock (gate)
        {
            target = active;
        }

        if (target is null)
        {
            if (frame.Id == CanFrame.TelemetryId)
            {
                Interlocked.Increment(ref dropped);
            }

            return false;
        }

        return target.Add(frame);
    }

    /// <summary>A sessao em memoria se sessionId e a ativa; senao null (consulte o banco).</summary>
    public Session? ActiveSession(long sessionId)
    {
        lock (gate)
        {
            return active is not null && activeId == sessionId ? active : null;
        }
    }

    /// <summary>Grava no banco o que a sessao ativa recebeu desde a ultima gravacao.</summary>
    public void Flush()
    {
        lock (gate)
        {
            FlushLocked();
        }
    }

    public void Dispose()
    {
        stop.Cancel();
        try
        {
            flusher.Wait();
        }
        catch (AggregateException)
        {
            // Cancelamento esperado.
        }

        Flush();
        stop.Dispose();
    }

    private void FlushLocked()
    {
        if (active is null)
        {
            return;
        }

        TelemetrySample[] fresh = active.SamplesFrom(flushed);
        if (fresh.Length == 0)
        {
            return;
        }

        store.Append(activeId, flushed, fresh);
        flushed += fresh.Length;
    }

    private async Task FlushLoop(TimeSpan period, CancellationToken token)
    {
        using var timer = new PeriodicTimer(period);
        try
        {
            while (await timer.WaitForNextTickAsync(token))
            {
                Flush();
            }
        }
        catch (OperationCanceledException)
        {
            // Hub fechado.
        }
    }
}
