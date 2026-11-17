using System.Buffers.Binary;
using System.Net.Sockets;
using SimTelemetry;
using Xunit;

namespace SimTelemetry.Tests;

internal static class Synthetic
{
    /// <summary>Monta um quadro 0x400 pelo layout documentado (nao pelo codigo sob teste).</summary>
    public static CanFrame Telemetry(double speedMps, int rpm, int throttleRaw, int brakeRaw, int gear, int flags = 0)
    {
        var data = new byte[8];
        BinaryPrimitives.WriteUInt16LittleEndian(data, (ushort)Math.Round(speedMps / 0.01));
        BinaryPrimitives.WriteUInt16LittleEndian(data.AsSpan(2), (ushort)rpm);
        data[4] = (byte)throttleRaw;
        data[5] = (byte)brakeRaw;
        data[6] = (byte)gear;
        data[7] = (byte)flags;
        return new CanFrame(0x400, data);
    }

    /// <summary>Sessao a velocidade constante, com n amostras a 100 Hz.</summary>
    public static Session Constant(double trackLengthM, double speedMps, int samples)
    {
        var session = new Session(trackLengthM);
        for (int i = 0; i < samples; i++)
        {
            session.Add(Telemetry(speedMps, 9000, 100, 0, 3));
        }

        return session;
    }
}

public class CanFrameTests
{
    [Fact]
    public void DecodesTheDocumentedLayout()
    {
        // speed 123,45 m/s = 12345 = 0x3039; rpm 9876 = 0x2694; acelerador 125 * 0,4% = 50%.
        var frame = new CanFrame(0x400, [0x39, 0x30, 0x94, 0x26, 125, 50, 4, 2]);
        Assert.True(TelemetrySample.TryDecode(frame, out TelemetrySample s));
        Assert.Equal(123.45, s.SpeedMps, 1e-9);
        Assert.Equal(9876, s.Rpm);
        Assert.Equal(0.5, s.Throttle, 1e-9);
        Assert.Equal(0.2, s.Brake, 1e-9);
        Assert.Equal(4, s.Gear);
        Assert.Equal(2, s.Flags);
    }

    [Fact]
    public void WireFormatRoundTrips()
    {
        var frame = new CanFrame(0x400, [1, 2, 3, 4, 5, 6, 7, 8]);
        byte[] wire = frame.ToWire();
        Assert.Equal(13, wire.Length);
        Assert.Equal([0x00, 0x04, 0x00, 0x00, 8], wire[..5]);  // id 0x400 little-endian, dlc 8
        CanFrame back = CanFrame.FromWire(wire);
        Assert.Equal(frame.Id, back.Id);
        Assert.Equal(frame.Data, back.Data);
    }

    [Fact]
    public void ShortFramesKeepTheirDlcAndPadWithZeros()
    {
        var frame = new CanFrame(0x100, [9, 8, 7]);
        byte[] wire = frame.ToWire();
        Assert.Equal(3, wire[4]);
        Assert.Equal([9, 8, 7, 0, 0, 0, 0, 0], wire[5..]);
        Assert.Equal(frame.Data, CanFrame.FromWire(wire).Data);
    }

    [Fact]
    public void OnlyTelemetryFramesDecode()
    {
        Assert.False(TelemetrySample.TryDecode(new CanFrame(0x300, new byte[8]), out _));  // outro id
        Assert.False(TelemetrySample.TryDecode(new CanFrame(0x400, new byte[7]), out _));  // dlc errado
    }

    [Fact]
    public void RejectsMalformedRecordsAndFrames()
    {
        var invalidDlc = new byte[13];
        invalidDlc[4] = 9;
        Assert.Throws<FormatException>(() => CanFrame.FromWire(invalidDlc));
        Assert.Throws<ArgumentException>(() => CanFrame.FromWire(new byte[12]));
        Assert.Throws<ArgumentException>(() => new CanFrame(0x400, new byte[9]));
    }
}

public class SessionTests
{
    [Fact]
    public void IgnoresFramesThatAreNotTelemetry()
    {
        var session = new Session(1000);
        Assert.False(session.Add(new CanFrame(0x300, new byte[7])));
        Assert.True(session.Add(Synthetic.Telemetry(10, 8000, 0, 0, 1)));
        Assert.Equal(1, session.Count);
    }

    [Fact]
    public void DetectsLapsFromIntegratedDistance()
    {
        // 50 m/s numa pista de 1000 m: 20 s por volta; 65 s de dados = 3 voltas completas.
        Session session = Synthetic.Constant(1000, 50, 6500);
        IReadOnlyList<Lap> laps = session.Laps();
        Assert.Equal(3, laps.Count);
        Assert.Equal([1, 2, 3], laps.Select(l => l.Number));
        Assert.All(laps, lap => Assert.Equal(20.0, lap.DurationS, 0.02));
        Assert.Equal(laps[0].EndS, laps[1].StartS);
    }

    [Fact]
    public void InterpolatesTheCrossingInsideTheStep()
    {
        // 35 m/s * 0,01 s = 0,35 m por amostra: a linha de 100 m cai no meio de um passo.
        Session session = Synthetic.Constant(100, 35, 600);
        Lap first = Assert.Single(session.Laps().Take(1));
        Assert.Equal(100.0 / 35.0, first.EndS, 0.01);
    }

    [Fact]
    public void ReportsNoLapBeforeTheLineIsCrossed()
    {
        Assert.Empty(Synthetic.Constant(1000, 50, 1900).Laps());  // 19 s < 20 s
        Assert.Empty(new Session(1000).Laps());
    }

    [Fact]
    public void ExposesChannelsInPhysicalUnits()
    {
        var session = new Session(1000);
        session.Add(Synthetic.Telemetry(12.5, 9000, 250, 0, 3));
        session.Add(Synthetic.Telemetry(13.5, 9100, 125, 50, 4));
        Series speed = session.Channel("speed");
        Assert.Equal([0.01, 0.02], speed.Time);
        Assert.Equal([12.5, 13.5], speed.Value);
        Assert.Equal([1.0, 0.5], session.Channel("throttle").Value);
        Assert.Equal([0.0, 0.2], session.Channel("brake").Value);
        Assert.Equal([3.0, 4.0], session.Channel("gear").Value);
        Assert.Equal([9000.0, 9100.0], session.Channel("rpm").Value);
    }

    [Fact]
    public void ResamplesLinearlyOnAUniformGrid()
    {
        // Rampa: velocidade = 10 * t, nas amostras de 0,01 s a 6,00 s.
        var session = new Session(1_000_000);
        for (int i = 1; i <= 600; i++)
        {
            session.Add(Synthetic.Telemetry(10.0 * i / 100.0, 9000, 0, 0, 1));
        }

        Series slow = session.Resample("speed", 10, 1.0, 2.0);
        Assert.Equal(11, slow.Time.Length);
        for (int k = 0; k < slow.Time.Length; k++)
        {
            Assert.Equal(1.0 + k / 10.0, slow.Time[k], 1e-9);
            Assert.Equal(10.0 * slow.Time[k], slow.Value[k], 0.006);  // quantizacao de 0,01 m/s
        }

        Series fast = session.Resample("speed", 1000, 3.0, 3.01);  // entre amostras: interpola
        Assert.Equal(11, fast.Time.Length);
        Assert.Equal(30.05, fast.Value[5], 0.006);
    }

    [Fact]
    public void ResampleClipsToTheDataAndHoldsGearSteps()
    {
        var session = new Session(1000);
        for (int i = 0; i < 100; i++)
        {
            session.Add(Synthetic.Telemetry(20, 9000, 0, 0, i < 50 ? 2 : 3));
        }

        Series all = session.Resample("gear", 100, -5, 500);
        Assert.Equal(0.01, all.Time[0], 1e-9);
        Assert.Equal(1.0, all.Time[^1], 1e-9);
        Assert.Equal(100, all.Value.Length);
        Assert.All(all.Value, g => Assert.True(g == 2 || g == 3));  // nunca 2,5
        Assert.Empty(session.Resample("speed", 10, 5, 6).Value);   // fora da sessao
    }

    [Fact]
    public void ExtractsOneLapWithTimeCountedFromItsStart()
    {
        Session session = Synthetic.Constant(1000, 50, 4500);  // 2 voltas de 20 s e mais 5 s
        Series lap2 = session.LapChannel("speed", 2, 10);
        Assert.Equal(0.0, lap2.Time[0], 1e-9);
        Assert.Equal(20.0, lap2.Time[^1], 0.1);
        Assert.All(lap2.Value, v => Assert.Equal(50.0, v, 1e-9));
        Assert.Throws<ArgumentOutOfRangeException>(() => session.LapChannel("speed", 3, 10));
        Assert.Throws<ArgumentOutOfRangeException>(() => session.LapChannel("speed", 0, 10));
    }

    [Fact]
    public void RejectsInvalidArguments()
    {
        Assert.Throws<ArgumentException>(() => new Session(10).Channel("temperature"));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Session(10).Resample("speed", 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Session(10).Resample("speed", double.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Session(0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Session(double.PositiveInfinity));
    }
}

public class TelemetryServerTests
{
    private static async Task WaitForCount(Session session, int expected)
    {
        for (int i = 0; i < 500 && session.Count < expected; i++)
        {
            await Task.Delay(10);
        }
    }

    [Fact]
    public async Task ReceivesFramesOverTcpEvenWhenRecordsAreSplit()
    {
        var session = new Session(1000);
        using var server = new TelemetryServer(session);
        server.Start();

        var stream = new List<byte>();
        for (int i = 0; i < 300; i++)
        {
            stream.AddRange(Synthetic.Telemetry(i * 0.5, 9000, 100, 0, 3).ToWire());
            if (i == 100)
            {
                stream.AddRange(new CanFrame(0x300, new byte[7]).ToWire());  // outro quadro: ignorado
            }
        }

        using var client = new TcpClient();
        await client.ConnectAsync(System.Net.IPAddress.Loopback, server.Port);
        NetworkStream tcp = client.GetStream();
        byte[] bytes = [.. stream];
        await tcp.WriteAsync(bytes.AsMemory(0, 20));  // corta no meio do segundo registro
        await tcp.FlushAsync();
        await Task.Delay(50);
        await tcp.WriteAsync(bytes.AsMemory(20));
        await tcp.FlushAsync();

        await WaitForCount(session, 300);
        Assert.Equal(300, session.Count);
        Assert.Equal(149.5, session.Channel("speed").Value[^1], 1e-9);
    }

    [Fact]
    public async Task DropsAnInvalidRecordWithoutLosingTheStream()
    {
        var session = new Session(1000);
        using var server = new TelemetryServer(session);
        server.Start();

        var bad = new byte[13];
        bad[4] = 200;  // dlc invalido
        using var client = new TcpClient();
        await client.ConnectAsync(System.Net.IPAddress.Loopback, server.Port);
        NetworkStream tcp = client.GetStream();
        await tcp.WriteAsync(Synthetic.Telemetry(10, 8000, 0, 0, 1).ToWire());
        await tcp.WriteAsync(bad);
        await tcp.WriteAsync(Synthetic.Telemetry(20, 8000, 0, 0, 1).ToWire());
        await tcp.FlushAsync();

        await WaitForCount(session, 2);
        Assert.Equal([10.0, 20.0], session.Channel("speed").Value);
    }

    [Fact]
    public async Task ServesSeveralClientsAtOnce()
    {
        var session = new Session(1000);
        using var server = new TelemetryServer(session);
        server.Start();

        async Task Send(int count)
        {
            using var client = new TcpClient();
            await client.ConnectAsync(System.Net.IPAddress.Loopback, server.Port);
            NetworkStream tcp = client.GetStream();
            for (int i = 0; i < count; i++)
            {
                await tcp.WriteAsync(Synthetic.Telemetry(15, 8000, 0, 0, 2).ToWire());
            }

            await tcp.FlushAsync();
        }

        await Task.WhenAll(Send(200), Send(200), Send(200));
        await WaitForCount(session, 600);
        Assert.Equal(600, session.Count);
    }
}

/// <summary>
/// Telemetria real: duas voltas do piloto virtual com a ECU no laco, gravadas pelo firmware em C
/// (sim_telemetry_dump) no formato de fio. Prova que o servidor entende o que a ECU realmente emite.
/// </summary>
public class RealTelemetryTests
{
    // Valores impressos por sim_telemetry_dump ao gerar o fixture.
    private const double TrackLengthM = 2215.0147;
    private const int ExpectedFrames = 9431;
    private static readonly double[] CppLapTimesS = [49.000, 45.309];

    private static Session Load()
    {
        byte[] bytes = File.ReadAllBytes(Path.Combine(AppContext.BaseDirectory, "Fixtures", "reference_2laps.can"));
        Assert.Equal(ExpectedFrames * CanFrame.WireSize, bytes.Length);
        var session = new Session(TrackLengthM);
        for (int offset = 0; offset < bytes.Length; offset += CanFrame.WireSize)
        {
            Assert.True(session.Add(CanFrame.FromWire(bytes.AsSpan(offset, CanFrame.WireSize))));
        }

        return session;
    }

    [Fact]
    public async Task ServerReadsTheFirmwareTelemetryOverTcp()
    {
        byte[] bytes = File.ReadAllBytes(Path.Combine(AppContext.BaseDirectory, "Fixtures", "reference_2laps.can"));
        var session = new Session(TrackLengthM);
        using var server = new TelemetryServer(session);
        server.Start();
        using var client = new TcpClient();
        await client.ConnectAsync(System.Net.IPAddress.Loopback, server.Port);
        await client.GetStream().WriteAsync(bytes);
        for (int i = 0; i < 1000 && session.Count < ExpectedFrames; i++)
        {
            await Task.Delay(10);
        }

        Assert.Equal(ExpectedFrames, session.Count);
        Assert.Equal(2, session.Laps().Count);
    }

    [Fact]
    public void DetectsTheSameLapsTheSimulatorTimed()
    {
        IReadOnlyList<Lap> laps = Load().Laps();
        Assert.Equal(2, laps.Count);
        // A volta vem da distancia integrada do canal de velocidade, nao da posicao: concorda com o
        // cronometro do simulador dentro de uma fracao de segundo.
        Assert.Equal(CppLapTimesS[0], laps[0].DurationS, 0.05);
        Assert.Equal(CppLapTimesS[1], laps[1].DurationS, 0.05);
    }

    [Fact]
    public void ChannelsLookLikeARacingLap()
    {
        Session session = Load();
        Series speed = session.LapChannel("speed", 2, 10);
        Assert.InRange(speed.Value.Max() * 3.6, 295, 312);   // ~304 km/h
        Assert.InRange(speed.Value.Min() * 3.6, 80, 110);    // curva mais lenta
        Series gear = session.Channel("gear");
        Assert.Equal(1.0, gear.Value.Min());
        Assert.InRange(gear.Value.Max(), 6.0, 8.0);  // a ~304 km/h o carro chega a 6a ou 7a
        Assert.True(session.Channel("rpm").Value.Max() <= 15000);
        Assert.InRange(session.Channel("brake").Value.Max(), 0.3, 1.0);
        Assert.InRange(session.Channel("throttle").Value.Max(), 0.99, 1.0);
        // Resampleada a 10 Hz, a volta 2 tem 45,309 s -> ~454 pontos.
        Assert.InRange(speed.Time.Length, 445, 455);
    }
}
