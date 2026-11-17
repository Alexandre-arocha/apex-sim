using System.Buffers.Binary;
using System.Net;
using System.Net.Sockets;
using System.Threading.Channels;
using Google.Protobuf;
using SimTelemetry.Proto;

namespace SimTelemetry;

/// <summary>Topicos usados entre o simulador e o servidor de telemetria.</summary>
public static class Topics
{
    public const string SessionStart = "session.start";
    public const string Sample = "telemetry.sample";
}

/// <summary>Quadros do broker no fio: 4 bytes de tamanho (uint32 big-endian) e a mensagem Protobuf.</summary>
internal static class Framing
{
    public const int MaxFrame = 1 << 20;

    public static byte[] Encode(byte[] message)
    {
        if (message.Length is 0 or > MaxFrame)
        {
            throw new ArgumentException("Tamanho de mensagem fora de 1 byte a 1 MiB.", nameof(message));
        }

        var frame = new byte[4 + message.Length];
        BinaryPrimitives.WriteUInt32BigEndian(frame, (uint)message.Length);
        message.CopyTo(frame, 4);
        return frame;
    }

    /// <summary>Le um quadro; null se a conexao fechou entre dois quadros.</summary>
    /// <exception cref="InvalidDataException">Tamanho fora de 1 byte a 1 MiB.</exception>
    /// <exception cref="EndOfStreamException">A conexao fechou no meio de um quadro.</exception>
    public static async Task<byte[]?> ReadAsync(Stream stream, CancellationToken token)
    {
        var header = new byte[4];
        int got = await stream.ReadAsync(header.AsMemory(), token);
        if (got == 0)
        {
            return null;
        }

        if (got < 4)
        {
            await stream.ReadExactlyAsync(header.AsMemory(got), token);
        }

        uint length = BinaryPrimitives.ReadUInt32BigEndian(header);
        if (length is 0 or > MaxFrame)
        {
            throw new InvalidDataException($"Tamanho de quadro invalido: {length}.");
        }

        var body = new byte[length];
        await stream.ReadExactlyAsync(body.AsMemory(), token);
        return body;
    }
}

public sealed record BrokerStats(long Published, long Delivered, long Dropped, long ProtocolErrors, int Clients);

/// <summary>
/// Broker pub/sub em TCP: clientes se inscrevem em topicos (exatos, ou prefixos terminados em '*') e
/// recebem o que outros publicam neles. Entrega no maximo uma vez e sem guardar nada: quem nao esta
/// inscrito no momento perde a mensagem. Cada assinante tem uma fila limitada; se ele nao acompanha,
/// as mensagens mais antigas dele sao descartadas (e contadas) sem atrasar os demais. Quadro invalido
/// ou mensagem que nao e Protobuf valida derruba so a conexao que a mandou.
/// </summary>
public sealed class BrokerServer : IDisposable
{
    private const int MaxTopicLength = 256;

    private readonly TcpListener listener;
    private readonly CancellationTokenSource stop = new();
    private readonly object gate = new();
    private readonly List<Peer> peers = [];
    private readonly int queueCapacity;
    private long published;
    private long delivered;
    private long dropped;
    private long protocolErrors;

    public BrokerServer(IPAddress? address = null, int port = 0, int queueCapacity = 1024)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(queueCapacity, 1);
        this.queueCapacity = queueCapacity;
        listener = new TcpListener(address ?? IPAddress.Loopback, port);
    }

    public int Port => ((IPEndPoint)listener.LocalEndpoint).Port;

    public BrokerStats Stats
    {
        get
        {
            lock (gate)
            {
                return new BrokerStats(
                    Interlocked.Read(ref published), Interlocked.Read(ref delivered),
                    Interlocked.Read(ref dropped), Interlocked.Read(ref protocolErrors), peers.Count);
            }
        }
    }

    public void Start()
    {
        listener.Start();
        _ = AcceptLoop(stop.Token);
    }

    public void Dispose()
    {
        stop.Cancel();
        listener.Stop();
        lock (gate)
        {
            foreach (Peer peer in peers)
            {
                peer.Close();
            }
        }

        stop.Dispose();
    }

    internal static bool Matches(string subscription, string topic) =>
        subscription.EndsWith('*') ? topic.StartsWith(subscription[..^1], StringComparison.Ordinal) : subscription == topic;

    private async Task AcceptLoop(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                TcpClient client = await listener.AcceptTcpClientAsync(token);
                _ = Serve(client, token);
            }
        }
        catch (Exception e) when (e is OperationCanceledException or ObjectDisposedException or SocketException)
        {
            // Broker parado.
        }
    }

    private async Task Serve(TcpClient client, CancellationToken token)
    {
        var peer = new Peer(client, queueCapacity, () => Interlocked.Increment(ref dropped));
        lock (gate)
        {
            peers.Add(peer);
        }

        Task writer = peer.RunWriter(token);
        try
        {
            NetworkStream stream = client.GetStream();
            while (await Framing.ReadAsync(stream, token) is { } bytes)
            {
                if (!Handle(peer, bytes))
                {
                    Interlocked.Increment(ref protocolErrors);
                    break;
                }
            }
        }
        catch (InvalidDataException)
        {
            Interlocked.Increment(ref protocolErrors);
        }
        catch (Exception e) when (e is EndOfStreamException or IOException or OperationCanceledException or ObjectDisposedException)
        {
            // Cliente saiu ou broker parado.
        }
        finally
        {
            lock (gate)
            {
                peers.Remove(peer);
            }

            peer.Close();
            await writer;
        }
    }

    /// <summary>Trata uma mensagem do cliente; false se ela e invalida.</summary>
    private bool Handle(Peer sender, byte[] bytes)
    {
        ClientMessage message;
        try
        {
            message = ClientMessage.Parser.ParseFrom(bytes);
        }
        catch (InvalidProtocolBufferException)
        {
            return false;
        }

        switch (message.KindCase)
        {
            case ClientMessage.KindOneofCase.Subscribe:
                string topic = message.Subscribe.Topic;
                if (topic.Length is 0 or > MaxTopicLength)
                {
                    return false;
                }

                sender.Subscribe(topic);
                sender.Enqueue(Framing.Encode(new ServerMessage { Ack = new Ack { Topic = topic } }.ToByteArray()));
                return true;

            case ClientMessage.KindOneofCase.Publish:
                Publish publish = message.Publish;
                if (publish.Topic.Length is 0 or > MaxTopicLength)
                {
                    return false;
                }

                Interlocked.Increment(ref published);
                byte[] frame = Framing.Encode(new ServerMessage { Publish = publish }.ToByteArray());
                Peer[] snapshot;
                lock (gate)
                {
                    snapshot = [.. peers];
                }

                foreach (Peer peer in snapshot)
                {
                    if (peer.IsSubscribedTo(publish.Topic) && peer.Enqueue(frame))
                    {
                        Interlocked.Increment(ref delivered);
                    }
                }

                return true;

            default:
                return false;
        }
    }

    private sealed class Peer
    {
        private readonly TcpClient client;
        private readonly Channel<byte[]> outbox;
        private readonly object subscriptionsGate = new();
        private readonly List<string> subscriptions = [];

        public Peer(TcpClient client, int capacity, Action onDropped)
        {
            this.client = client;
            outbox = Channel.CreateBounded<byte[]>(
                new BoundedChannelOptions(capacity) { FullMode = BoundedChannelFullMode.DropOldest, SingleReader = true },
                _ => onDropped());
        }

        public void Subscribe(string topic)
        {
            lock (subscriptionsGate)
            {
                if (!subscriptions.Contains(topic))
                {
                    subscriptions.Add(topic);
                }
            }
        }

        public bool IsSubscribedTo(string topic)
        {
            lock (subscriptionsGate)
            {
                return subscriptions.Any(s => Matches(s, topic));
            }
        }

        public bool Enqueue(byte[] frame) => outbox.Writer.TryWrite(frame);

        public async Task RunWriter(CancellationToken token)
        {
            try
            {
                NetworkStream stream = client.GetStream();
                await foreach (byte[] frame in outbox.Reader.ReadAllAsync(token))
                {
                    await stream.WriteAsync(frame, token);
                }
            }
            catch (Exception e) when (e is IOException or OperationCanceledException or ObjectDisposedException or InvalidOperationException)
            {
                // Conexao caiu ou broker parado.
            }
        }

        public void Close()
        {
            outbox.Writer.TryComplete();
            client.Close();
        }
    }
}

/// <summary>Cliente do broker: inscreve-se em topicos e publica mensagens.</summary>
public sealed class BrokerClient : IAsyncDisposable
{
    private readonly TcpClient tcp;
    private readonly NetworkStream stream;
    private readonly Func<string, byte[], Task>? onMessage;
    private readonly SemaphoreSlim writeGate = new(1, 1);
    private readonly CancellationTokenSource stop = new();
    private readonly object pendingGate = new();
    private readonly Dictionary<string, Queue<TaskCompletionSource>> pendingAcks = [];
    private Task reader = Task.CompletedTask;

    private BrokerClient(TcpClient tcp, Func<string, byte[], Task>? onMessage)
    {
        this.tcp = tcp;
        stream = tcp.GetStream();
        this.onMessage = onMessage;
    }

    /// <summary>onMessage recebe (topico, conteudo) das mensagens dos topicos assinados, uma de cada vez e em ordem.</summary>
    public static async Task<BrokerClient> ConnectAsync(string host, int port, Func<string, byte[], Task>? onMessage = null)
    {
        var tcp = new TcpClient();
        await tcp.ConnectAsync(host, port);
        var client = new BrokerClient(tcp, onMessage);
        client.reader = client.ReadLoop();
        return client;
    }

    /// <summary>Assina um topico (exato ou prefixo terminado em '*') e espera o broker confirmar.</summary>
    public async Task SubscribeAsync(string topic, TimeSpan? timeout = null)
    {
        var ack = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        lock (pendingGate)
        {
            if (!pendingAcks.TryGetValue(topic, out Queue<TaskCompletionSource>? queue))
            {
                pendingAcks[topic] = queue = new Queue<TaskCompletionSource>();
            }

            queue.Enqueue(ack);
        }

        await SendAsync(new ClientMessage { Subscribe = new Subscribe { Topic = topic } });
        await ack.Task.WaitAsync(timeout ?? TimeSpan.FromSeconds(5));
    }

    public Task PublishAsync(string topic, byte[] payload) =>
        SendAsync(new ClientMessage { Publish = new Publish { Topic = topic, Payload = ByteString.CopyFrom(payload) } });

    public Task PublishAsync(string topic, IMessage message) => PublishAsync(topic, message.ToByteArray());

    public async ValueTask DisposeAsync()
    {
        stop.Cancel();
        tcp.Close();
        try
        {
            await reader;
        }
        catch (Exception e) when (e is IOException or OperationCanceledException or ObjectDisposedException)
        {
            // Fechando.
        }

        stop.Dispose();
        writeGate.Dispose();
    }

    private async Task SendAsync(ClientMessage message)
    {
        byte[] frame = Framing.Encode(message.ToByteArray());
        await writeGate.WaitAsync(stop.Token);
        try
        {
            await stream.WriteAsync(frame, stop.Token);
        }
        finally
        {
            writeGate.Release();
        }
    }

    private async Task ReadLoop()
    {
        try
        {
            while (await Framing.ReadAsync(stream, stop.Token) is { } bytes)
            {
                ServerMessage message = ServerMessage.Parser.ParseFrom(bytes);
                if (message.KindCase == ServerMessage.KindOneofCase.Ack)
                {
                    CompleteAck(message.Ack.Topic);
                }
                else if (message.KindCase == ServerMessage.KindOneofCase.Publish && onMessage is not null)
                {
                    await onMessage(message.Publish.Topic, message.Publish.Payload.ToByteArray());
                }
            }
        }
        catch (Exception e) when (e is IOException or OperationCanceledException or ObjectDisposedException
                                      or EndOfStreamException or InvalidDataException or InvalidProtocolBufferException)
        {
            // Conexao fechou ou o broker mandou algo invalido: o cliente para de ouvir.
        }
    }

    private void CompleteAck(string topic)
    {
        TaskCompletionSource? waiting = null;
        lock (pendingGate)
        {
            if (pendingAcks.TryGetValue(topic, out Queue<TaskCompletionSource>? queue) && queue.Count > 0)
            {
                waiting = queue.Dequeue();
            }
        }

        waiting?.TrySetResult();
    }
}

/// <summary>
/// Liga o broker ao gravador: ouve "session.start" (abre uma sessao) e "telemetry.sample" (uma
/// amostra) e os entrega ao RecordingHub. Amostras perdidas no caminho (a sequencia pula) sao
/// preenchidas repetindo a ultima amostra, para o tempo da sessao continuar certo; repetidas ou
/// atrasadas sao ignoradas.
/// </summary>
public sealed class TelemetryBridge : IAsyncDisposable
{
    private readonly BrokerClient client;
    private readonly RecordingHub hub;
    private long rejected;

    private TelemetryBridge(BrokerClient client, RecordingHub hub)
    {
        this.client = client;
        this.hub = hub;
    }

    /// <summary>Mensagens que nao puderam ser usadas (Protobuf invalido ou sessao invalida).</summary>
    public long Rejected => Interlocked.Read(ref rejected);

    public static async Task<TelemetryBridge> StartAsync(string host, int port, RecordingHub hub)
    {
        TelemetryBridge? bridge = null;
        BrokerClient client = await BrokerClient.ConnectAsync(host, port, (topic, payload) =>
        {
            bridge?.Handle(topic, payload);
            return Task.CompletedTask;
        });
        bridge = new TelemetryBridge(client, hub);
        await client.SubscribeAsync(Topics.SessionStart);
        await client.SubscribeAsync(Topics.Sample);
        return bridge;
    }

    public ValueTask DisposeAsync() => client.DisposeAsync();

    private void Handle(string topic, byte[] payload)
    {
        try
        {
            if (topic == Topics.SessionStart)
            {
                SessionStart start = SessionStart.Parser.ParseFrom(payload);
                hub.Start(start.Name, start.TrackLengthM);
            }
            else if (topic == Topics.Sample)
            {
                Proto.TelemetrySample sample = Proto.TelemetrySample.Parser.ParseFrom(payload);
                hub.AddSample((long)sample.Sequence, new TelemetrySample(
                    sample.SpeedMps, sample.Rpm, sample.Throttle, sample.Brake, (int)sample.Gear, (byte)sample.Flags));
            }
        }
        catch (Exception e) when (e is InvalidProtocolBufferException or ArgumentException)
        {
            Interlocked.Increment(ref rejected);
        }
    }
}
