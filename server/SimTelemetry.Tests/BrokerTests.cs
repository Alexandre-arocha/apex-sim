using System.Buffers.Binary;
using System.Net.Sockets;
using Google.Protobuf;
using SimTelemetry;
using SimTelemetry.Proto;
using Xunit;
using Sample = SimTelemetry.Proto.TelemetrySample;

namespace SimTelemetry.Tests;

internal static class Wait
{
    public static async Task Until(Func<bool> condition, int timeoutMs = 5000)
    {
        var deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs);
        while (!condition())
        {
            Assert.True(DateTime.UtcNow < deadline, "tempo esgotado esperando a condicao");
            await Task.Delay(5);
        }
    }
}

/// <summary>Cliente cru: fala o quadro do broker na mao, sem passar pelo codigo sob teste.</summary>
internal sealed class RawPeer : IDisposable
{
    private readonly TcpClient tcp = new();

    public static async Task<RawPeer> ConnectAsync(int port, int receiveBuffer = 0)
    {
        var peer = new RawPeer();
        if (receiveBuffer > 0)
        {
            peer.tcp.ReceiveBufferSize = receiveBuffer;
        }

        await peer.tcp.ConnectAsync(System.Net.IPAddress.Loopback, port);
        return peer;
    }

    public async Task SendFrame(byte[] message)
    {
        var frame = new byte[4 + message.Length];
        BinaryPrimitives.WriteUInt32BigEndian(frame, (uint)message.Length);
        message.CopyTo(frame, 4);
        await tcp.GetStream().WriteAsync(frame);
    }

    public Task Subscribe(string topic) =>
        SendFrame(new ClientMessage { Subscribe = new Subscribe { Topic = topic } }.ToByteArray());

    public async Task SendHeaderOnly(uint length)
    {
        var header = new byte[4];
        BinaryPrimitives.WriteUInt32BigEndian(header, length);
        await tcp.GetStream().WriteAsync(header);
    }

    /// <summary>True se o broker fechou a conexao (fim de fluxo ou reset) em ate 3 s.</summary>
    public async Task<bool> ClosedByBroker()
    {
        using var cts = new CancellationTokenSource(3000);
        var buffer = new byte[256];
        try
        {
            while (await tcp.GetStream().ReadAsync(buffer, cts.Token) > 0)
            {
                // Descarta o que ja estava a caminho (confirmacao de assinatura).
            }

            return true;
        }
        catch (IOException)
        {
            return true;
        }
        catch (OperationCanceledException)
        {
            return false;
        }
    }

    public void Dispose() => tcp.Dispose();
}

public class BrokerTests
{
    private static BrokerServer StartBroker(int queueCapacity = 1024)
    {
        var broker = new BrokerServer(queueCapacity: queueCapacity);
        broker.Start();
        return broker;
    }

    private sealed class Inbox
    {
        private readonly List<(string Topic, byte[] Payload)> items = [];

        public Task Add(string topic, byte[] payload)
        {
            lock (items)
            {
                items.Add((topic, payload));
            }

            return Task.CompletedTask;
        }

        public int Count
        {
            get
            {
                lock (items)
                {
                    return items.Count;
                }
            }
        }

        public (string Topic, byte[] Payload)[] Snapshot()
        {
            lock (items)
            {
                return [.. items];
            }
        }
    }

    [Fact]
    public async Task RoutesByExactTopicAndByPrefix()
    {
        using BrokerServer broker = StartBroker();
        var exact = new Inbox();
        var prefix = new Inbox();
        await using BrokerClient a = await BrokerClient.ConnectAsync("127.0.0.1", broker.Port, exact.Add);
        await using BrokerClient b = await BrokerClient.ConnectAsync("127.0.0.1", broker.Port, prefix.Add);
        await using BrokerClient pub = await BrokerClient.ConnectAsync("127.0.0.1", broker.Port);
        await a.SubscribeAsync("telemetry.sample");
        await b.SubscribeAsync("telemetry.*");

        await pub.PublishAsync("telemetry.sample", [1]);
        await pub.PublishAsync("telemetry.other", [2]);
        await pub.PublishAsync("session.start", [3]);
        await pub.PublishAsync("telemetry", [4]);   // sem o '.', o prefixo "telemetry." nao casa

        await Wait.Until(() => exact.Count == 1 && prefix.Count == 2);
        await Task.Delay(100);   // da tempo de uma entrega indevida aparecer
        Assert.Equal([("telemetry.sample", new byte[] { 1 })], exact.Snapshot());
        Assert.Equal(["telemetry.sample", "telemetry.other"], prefix.Snapshot().Select(m => m.Topic));
        Assert.Equal(4, broker.Stats.Published);
    }

    [Fact]
    public async Task DeliversInOrderAndToEverySubscriber()
    {
        using BrokerServer broker = StartBroker();
        var one = new Inbox();
        var two = new Inbox();
        await using BrokerClient a = await BrokerClient.ConnectAsync("127.0.0.1", broker.Port, one.Add);
        await using BrokerClient b = await BrokerClient.ConnectAsync("127.0.0.1", broker.Port, two.Add);
        await using BrokerClient pub = await BrokerClient.ConnectAsync("127.0.0.1", broker.Port);
        await a.SubscribeAsync("t");
        await b.SubscribeAsync("t");

        for (int i = 0; i < 1000; i++)
        {
            await pub.PublishAsync("t", BitConverter.GetBytes(i));
        }

        await Wait.Until(() => one.Count == 1000 && two.Count == 1000);
        int[] expected = Enumerable.Range(0, 1000).ToArray();
        Assert.Equal(expected, one.Snapshot().Select(m => BitConverter.ToInt32(m.Payload)));
        Assert.Equal(expected, two.Snapshot().Select(m => BitConverter.ToInt32(m.Payload)));
        Assert.Equal(0, broker.Stats.Dropped);
    }

    [Fact]
    public async Task OversizedFrameDropsOnlyThatConnection()
    {
        using BrokerServer broker = StartBroker();
        var inbox = new Inbox();
        await using BrokerClient good = await BrokerClient.ConnectAsync("127.0.0.1", broker.Port, inbox.Add);
        await good.SubscribeAsync("t");
        using RawPeer bad = await RawPeer.ConnectAsync(broker.Port);

        await bad.SendHeaderOnly(2u << 20);   // 2 MiB: acima do limite de 1 MiB
        Assert.True(await bad.ClosedByBroker());
        await Wait.Until(() => broker.Stats.ProtocolErrors == 1);

        await good.PublishAsync("t", [7]);   // o broker segue de pe para os outros
        await Wait.Until(() => inbox.Count == 1);
    }

    [Theory]
    [InlineData(new byte[] { 0xFF, 0xFF, 0xFF })]             // nao e Protobuf valido
    [InlineData(new byte[] { })]                              // mensagem vazia: tamanho 0 invalido
    public async Task GarbageDropsOnlyThatConnection(byte[] garbage)
    {
        using BrokerServer broker = StartBroker();
        var inbox = new Inbox();
        await using BrokerClient good = await BrokerClient.ConnectAsync("127.0.0.1", broker.Port, inbox.Add);
        await good.SubscribeAsync("t");
        using RawPeer bad = await RawPeer.ConnectAsync(broker.Port);

        if (garbage.Length == 0)
        {
            await bad.SendHeaderOnly(0);
        }
        else
        {
            await bad.SendFrame(garbage);
        }

        Assert.True(await bad.ClosedByBroker());
        await Wait.Until(() => broker.Stats.ProtocolErrors == 1);
        await good.PublishAsync("t", [9]);
        await Wait.Until(() => inbox.Count == 1);
    }

    [Fact]
    public async Task RejectsTopicsOutsideTheLimits()
    {
        using BrokerServer broker = StartBroker();
        using RawPeer bad = await RawPeer.ConnectAsync(broker.Port);
        await bad.Subscribe("");   // topico vazio
        Assert.True(await bad.ClosedByBroker());
        await Wait.Until(() => broker.Stats.ProtocolErrors == 1);
    }

    [Fact]
    public async Task SlowSubscriberLosesOldMessagesWithoutHoldingBackOthers()
    {
        using BrokerServer broker = StartBroker(queueCapacity: 16);
        using RawPeer slow = await RawPeer.ConnectAsync(broker.Port, receiveBuffer: 4096);
        await slow.Subscribe("t");   // assina e nunca le
        var fast = new Inbox();
        await using BrokerClient reader = await BrokerClient.ConnectAsync("127.0.0.1", broker.Port, fast.Add);
        await using BrokerClient pub = await BrokerClient.ConnectAsync("127.0.0.1", broker.Port);
        await reader.SubscribeAsync("t");
        await Wait.Until(() => broker.Stats.Clients == 3);

        byte[] payload = new byte[32 * 1024];
        const int count = 1000;   // ~32 MB: mais do que cabe nos buffers do soquete lento
        for (int i = 0; i < count; i++)
        {
            await pub.PublishAsync("t", payload);
            await Wait.Until(() => fast.Count > i);   // o rapido acompanha mensagem a mensagem
        }

        Assert.Equal(count, fast.Count);
        Assert.True(broker.Stats.Dropped > 0, "o assinante lento deveria ter perdido mensagens");
    }
}

public class TelemetryBridgeTests
{
    private sealed class Rig : IAsyncDisposable
    {
        public required BrokerServer Broker { get; init; }
        public required SessionStore Store { get; init; }
        public required RecordingHub Hub { get; init; }
        public required TelemetryBridge Bridge { get; init; }
        public required BrokerClient Publisher { get; init; }

        public static async Task<Rig> StartAsync(int queueCapacity = 1024)
        {
            var broker = new BrokerServer(queueCapacity: queueCapacity);
            broker.Start();
            var store = new SessionStore(":memory:");
            var hub = new RecordingHub(store);
            TelemetryBridge bridge = await TelemetryBridge.StartAsync("127.0.0.1", broker.Port, hub);
            BrokerClient publisher = await BrokerClient.ConnectAsync("127.0.0.1", broker.Port);
            return new Rig { Broker = broker, Store = store, Hub = hub, Bridge = bridge, Publisher = publisher };
        }

        public Task StartSession(string name = "pista", double length = 1000) =>
            Publisher.PublishAsync(Topics.SessionStart, new SessionStart { Name = name, TrackLengthM = length });

        public Task Send(ulong sequence, double speed = 10, uint gear = 1) =>
            Publisher.PublishAsync(Topics.Sample, new Sample { Sequence = sequence, SpeedMps = speed, Gear = gear });

        public Session Active => Hub.ActiveSession(Hub.ActiveId!.Value)!;

        public async ValueTask DisposeAsync()
        {
            await Publisher.DisposeAsync();
            await Bridge.DisposeAsync();
            Hub.Dispose();
            Store.Dispose();
            Broker.Dispose();
        }
    }

    [Fact]
    public async Task RecordsASessionFromSessionStartAndSamples()
    {
        await using Rig rig = await Rig.StartAsync();
        await rig.StartSession("volta rapida", 2215);
        await Wait.Until(() => rig.Hub.ActiveId is not null);
        for (ulong i = 0; i < 300; i++)
        {
            await rig.Send(i, speed: 1 + i, gear: 3);
        }

        await Wait.Until(() => rig.Active.Count == 300);
        rig.Hub.Flush();
        long id = rig.Hub.ActiveId!.Value;
        Assert.Equal("volta rapida", rig.Store.Find(id)!.Name);
        Assert.Equal(300, rig.Store.Find(id)!.SampleCount);
        TelemetrySample last = rig.Active.SamplesFrom(299)[0];
        Assert.Equal(300.0, last.SpeedMps);
        Assert.Equal(3, last.Gear);
        Assert.Equal(0, rig.Hub.GapsFilled);
    }

    [Fact]
    public async Task FillsGapsByRepeatingTheLastSample()
    {
        await using Rig rig = await Rig.StartAsync();
        await rig.StartSession();
        await Wait.Until(() => rig.Hub.ActiveId is not null);
        await rig.Send(0, speed: 10);
        await rig.Send(1, speed: 20);
        await rig.Send(5, speed: 60);   // 2, 3 e 4 se perderam no caminho
        await Wait.Until(() => rig.Active.Count == 6);

        TelemetrySample[] samples = rig.Active.SamplesFrom(0);
        Assert.Equal([10.0, 20.0, 20.0, 20.0, 20.0, 60.0], samples.Select(s => s.SpeedMps));
        Assert.Equal(3, rig.Hub.GapsFilled);
    }

    [Fact]
    public async Task IgnoresRepeatedAndLateSamples()
    {
        await using Rig rig = await Rig.StartAsync();
        await rig.StartSession();
        await Wait.Until(() => rig.Hub.ActiveId is not null);
        for (ulong i = 0; i < 5; i++)
        {
            await rig.Send(i, speed: i);
        }

        await rig.Send(4, speed: 999);   // repetida
        await rig.Send(2, speed: 999);   // atrasada
        await rig.Send(5, speed: 5);
        await Wait.Until(() => rig.Active.Count == 6);

        Assert.Equal(2, rig.Hub.StaleSamples);
        Assert.Equal([0.0, 1.0, 2.0, 3.0, 4.0, 5.0], rig.Active.SamplesFrom(0).Select(s => s.SpeedMps));
    }

    [Fact]
    public async Task DropsSamplesThatArriveBeforeAnySession()
    {
        await using Rig rig = await Rig.StartAsync();
        await rig.Send(0);
        await Wait.Until(() => rig.Hub.DroppedFrames == 1);
        Assert.Null(rig.Hub.ActiveId);
    }

    [Fact]
    public async Task RejectsInvalidMessagesAndKeepsGoing()
    {
        await using Rig rig = await Rig.StartAsync();
        await rig.Publisher.PublishAsync(Topics.SessionStart, new SessionStart { Name = "x", TrackLengthM = 0 });
        await rig.Publisher.PublishAsync(Topics.SessionStart, [0xFF, 0xFF, 0xFF]);   // nao e Protobuf
        await rig.Publisher.PublishAsync(Topics.Sample, [0xFF, 0xFF, 0xFF]);
        await Wait.Until(() => rig.Bridge.Rejected == 3);
        Assert.Null(rig.Hub.ActiveId);

        await rig.StartSession();   // a ponte continua de pe
        await Wait.Until(() => rig.Hub.ActiveId is not null);
    }

    [Fact]
    public void EncodesTheDocumentedBytes()
    {
        // Os mesmos bytes sao conferidos no lado C++ (tests/broker_publisher_test.cpp): os dois
        // lados concordam sobre o fio sem compartilhar codigo.
        var sample = new Sample { Sequence = 300, SpeedMps = 1.5, Gear = 4 };
        Assert.Equal(
            [0x08, 0xAC, 0x02, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x3F, 0x30, 0x04],
            sample.ToByteArray());
        Assert.Equal(
            [0x0A, 0x05, 0x70, 0x69, 0x73, 0x74, 0x61, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x8F, 0x40],
            new SessionStart { Name = "pista", TrackLengthM = 1000 }.ToByteArray());
        Assert.Equal(
            [0x12, 0x07, 0x0A, 0x01, 0x74, 0x12, 0x02, 0x01, 0x02],
            new ClientMessage { Publish = new Publish { Topic = "t", Payload = ByteString.CopyFrom(1, 2) } }.ToByteArray());
        Assert.Empty(new Sample().ToByteArray());   // valores padrao nao vao ao fio (o C++ confere o mesmo)
    }

    [Fact]
    public async Task TheFirmwareTelemetryTravelsThroughTheBrokerIntoTwoLaps()
    {
        byte[] bytes = File.ReadAllBytes(Path.Combine(AppContext.BaseDirectory, "Fixtures", "reference_2laps.can"));
        var direct = new Session(2215.0147);
        var messages = new List<Sample>();
        for (int offset = 0; offset < bytes.Length; offset += CanFrame.WireSize)
        {
            var frame = CanFrame.FromWire(bytes.AsSpan(offset, CanFrame.WireSize));
            Assert.True(direct.Add(frame));
            TelemetrySample.TryDecode(frame, out TelemetrySample s);
            messages.Add(new Sample
            {
                Sequence = (ulong)messages.Count, SpeedMps = s.SpeedMps, Rpm = s.Rpm,
                Throttle = s.Throttle, Brake = s.Brake, Gear = (uint)s.Gear, Flags = s.Flags,
            });
        }

        await using Rig rig = await Rig.StartAsync(queueCapacity: 16384);   // cabe a rajada inteira
        await rig.StartSession("referencia", 2215.0147);
        await Wait.Until(() => rig.Hub.ActiveId is not null);
        foreach (Sample message in messages)
        {
            await rig.Publisher.PublishAsync(Topics.Sample, message);
        }

        await Wait.Until(() => rig.Active.Count == messages.Count, timeoutMs: 20000);
        Assert.Equal(direct.SamplesFrom(0), rig.Active.SamplesFrom(0));   // o fio nao perde nem muda nada
        Assert.Equal(2, rig.Active.Laps().Count);
        Assert.Equal(0, rig.Hub.GapsFilled);
        Assert.Equal(0, rig.Broker.Stats.Dropped);
    }
}
