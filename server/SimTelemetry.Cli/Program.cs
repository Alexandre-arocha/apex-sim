// Servidor de telemetria de linha de comando: escuta uma porta TCP, recebe os quadros da ECU
// (por exemplo de sim_telemetry_dump <arquivo|-> <voltas> host:porta) e, quando a transmissao
// termina (1 s sem quadros novos), imprime quantos quadros chegaram e as voltas detectadas.
//   uso: SimTelemetry.Cli <porta> <comprimento-da-pista-m> [timeout-s]
//
// Modo serve: recebe a telemetria por TCP, grava as sessoes num banco SQLite e responde consultas
// por HTTP (so no loopback, sem autenticacao). Sem comprimento de pista nao ha sessao gravando ate
// um POST /sessions.
//   uso: SimTelemetry.Cli serve <banco.db> <porta-tcp> <porta-http> [comprimento-da-pista-m] [--broker host:porta]
// Com --broker, o servidor tambem assina o broker (session.start e telemetry.sample, em Protobuf) e
// grava o que o simulador publica la.
//
// Modo broker: broker pub/sub em TCP (so no loopback, sem autenticacao).
//   uso: SimTelemetry.Cli broker <porta> [tamanho-da-fila-por-assinante]
// Modo consume: assina um topico (exato ou prefixo terminado em '*') e imprime o que chega.
//   uso: SimTelemetry.Cli consume <host:porta> [topico] [maximo-de-mensagens]
using System.Diagnostics;
using System.Globalization;
using Microsoft.AspNetCore.Builder;
using SimTelemetry;
using SimTelemetry.Proto;

if (args.Length > 0 && args[0] == "serve")
{
    return await Serve(args[1..]);
}

if (args.Length > 0 && args[0] == "broker")
{
    return await RunBroker(args[1..]);
}

if (args.Length > 0 && args[0] == "consume")
{
    return await Consume(args[1..]);
}

if (args.Length < 2
    || !int.TryParse(args[0], NumberStyles.None, CultureInfo.InvariantCulture, out int port)
    || !double.TryParse(args[1], NumberStyles.Float, CultureInfo.InvariantCulture, out double trackLengthM))
{
    Console.Error.WriteLine("uso: SimTelemetry.Cli <porta> <comprimento-da-pista-m> [timeout-s]");
    return 2;
}

double timeoutS = 120;
if (args.Length > 2 && !double.TryParse(args[2], NumberStyles.Float, CultureInfo.InvariantCulture, out timeoutS))
{
    Console.Error.WriteLine("timeout invalido");
    return 2;
}

var session = new Session(trackLengthM);
using var server = new TelemetryServer(session, port: port);
server.Start();
Console.WriteLine(FormattableString.Invariant($"ouvindo porta {server.Port}"));

var clock = Stopwatch.StartNew();
int last = 0;
TimeSpan lastChange = clock.Elapsed;
while (clock.Elapsed.TotalSeconds < timeoutS)
{
    await Task.Delay(100);
    int count = session.Count;
    if (count != last)
    {
        last = count;
        lastChange = clock.Elapsed;
    }
    else if (count > 0 && (clock.Elapsed - lastChange).TotalSeconds >= 1.0)
    {
        break;
    }
}

IReadOnlyList<Lap> laps = session.Laps();
Console.WriteLine(FormattableString.Invariant($"quadros={session.Count}"));
Console.WriteLine(FormattableString.Invariant($"voltas={laps.Count}"));
foreach (Lap lap in laps)
{
    Console.WriteLine(FormattableString.Invariant($"volta {lap.Number} {lap.DurationS:F3}"));
}

return session.Count > 0 ? 0 : 1;

static bool TryParseEndpoint(string text, out string host, out int port)
{
    int colon = text.LastIndexOf(':');
    host = colon > 0 ? text[..colon] : string.Empty;
    port = 0;
    return colon > 0 && int.TryParse(text[(colon + 1)..], NumberStyles.None, CultureInfo.InvariantCulture, out port)
        && port is >= 1 and <= 65535;
}

static Task WaitForCtrlC()
{
    var done = new TaskCompletionSource();
    Console.CancelKeyPress += (_, e) =>
    {
        e.Cancel = true;
        done.TrySetResult();
    };
    return done.Task;
}

static async Task<int> RunBroker(string[] brokerArgs)
{
    int queue = 65536;   // cabe uma volta inteira de amostras a 100 Hz (rajada do simulador sem pacing)
    if (brokerArgs.Length < 1
        || !int.TryParse(brokerArgs[0], NumberStyles.None, CultureInfo.InvariantCulture, out int port)
        || (brokerArgs.Length > 1
            && !int.TryParse(brokerArgs[1], NumberStyles.None, CultureInfo.InvariantCulture, out queue))
        || queue < 1)
    {
        Console.Error.WriteLine("uso: SimTelemetry.Cli broker <porta> [tamanho-da-fila-por-assinante]");
        return 2;
    }

    using var broker = new BrokerServer(port: port, queueCapacity: queue);
    broker.Start();
    Console.WriteLine(FormattableString.Invariant($"ouvindo broker {broker.Port}"));
    await WaitForCtrlC();
    BrokerStats stats = broker.Stats;
    Console.WriteLine(FormattableString.Invariant(
        $"publicadas={stats.Published} entregues={stats.Delivered} descartadas={stats.Dropped} erros={stats.ProtocolErrors}"));
    return 0;
}

static async Task<int> Consume(string[] consumeArgs)
{
    int max = int.MaxValue;
    if (consumeArgs.Length < 1
        || !TryParseEndpoint(consumeArgs[0], out string host, out int port)
        || (consumeArgs.Length > 2
            && (!int.TryParse(consumeArgs[2], NumberStyles.None, CultureInfo.InvariantCulture, out max) || max < 1)))
    {
        Console.Error.WriteLine("uso: SimTelemetry.Cli consume <host:porta> [topico] [maximo-de-mensagens]");
        return 2;
    }

    string topic = consumeArgs.Length > 1 ? consumeArgs[1] : "*";
    int received = 0;
    var finished = new TaskCompletionSource();
    await using BrokerClient client = await BrokerClient.ConnectAsync(host, port, (t, payload) =>
    {
        Console.WriteLine(Describe(t, payload));
        if (Interlocked.Increment(ref received) >= max)
        {
            finished.TrySetResult();
        }

        return Task.CompletedTask;
    });
    await client.SubscribeAsync(topic);
    Console.WriteLine($"assinado {topic}");
    await Task.WhenAny(finished.Task, WaitForCtrlC());
    return 0;
}

static string Describe(string topic, byte[] payload)
{
    try
    {
        if (topic == Topics.Sample)
        {
            SimTelemetry.Proto.TelemetrySample s = SimTelemetry.Proto.TelemetrySample.Parser.ParseFrom(payload);
            return FormattableString.Invariant(
                $"{topic} seq={s.Sequence} v={s.SpeedMps * 3.6:F1}km/h rpm={s.Rpm:F0} marcha={s.Gear} flags={s.Flags}");
        }

        if (topic == Topics.SessionStart)
        {
            SessionStart s = SessionStart.Parser.ParseFrom(payload);
            return FormattableString.Invariant($"{topic} nome={s.Name} pista={s.TrackLengthM:F1}m");
        }
    }
    catch (Google.Protobuf.InvalidProtocolBufferException)
    {
        return FormattableString.Invariant($"{topic} <Protobuf invalido, {payload.Length} bytes>");
    }

    return FormattableString.Invariant($"{topic} {payload.Length} bytes");
}

static async Task<int> Serve(string[] allArgs)
{
    string? brokerTarget = null;
    var serveArgs = new List<string>();
    for (int i = 0; i < allArgs.Length; i++)
    {
        if (allArgs[i] == "--broker" && i + 1 < allArgs.Length)
        {
            brokerTarget = allArgs[++i];
        }
        else
        {
            serveArgs.Add(allArgs[i]);
        }
    }

    string brokerHost = string.Empty;
    int brokerPort = 0;
    if (brokerTarget is not null && !TryParseEndpoint(brokerTarget, out brokerHost, out brokerPort))
    {
        Console.Error.WriteLine("--broker espera host:porta");
        return 2;
    }

    if (serveArgs.Count < 3
        || !int.TryParse(serveArgs[1], NumberStyles.None, CultureInfo.InvariantCulture, out int tcpPort)
        || !int.TryParse(serveArgs[2], NumberStyles.None, CultureInfo.InvariantCulture, out int httpPort))
    {
        Console.Error.WriteLine(
            "uso: SimTelemetry.Cli serve <banco.db> <porta-tcp> <porta-http> [comprimento-da-pista-m] [--broker host:porta]");
        return 2;
    }

    double? trackLength = null;
    if (serveArgs.Count > 3)
    {
        if (!double.TryParse(serveArgs[3], NumberStyles.Float, CultureInfo.InvariantCulture, out double length))
        {
            Console.Error.WriteLine("comprimento da pista invalido");
            return 2;
        }

        trackLength = length;
    }

    using var store = new SessionStore(serveArgs[0]);
    using var hub = new RecordingHub(store);
    using var tcp = new TelemetryServer(hub, port: tcpPort);
    tcp.Start();
    if (trackLength is double first)
    {
        hub.Start("sessao-inicial", first);
    }

    TelemetryBridge? bridge = null;
    if (brokerTarget is not null)
    {
        try
        {
            bridge = await TelemetryBridge.StartAsync(brokerHost, brokerPort, hub);
        }
        catch (System.Net.Sockets.SocketException e)
        {
            Console.Error.WriteLine($"nao consegui conectar ao broker {brokerTarget}: {e.Message}");
            return 1;
        }
    }

    await using WebApplication app = TelemetryApi.CreateApp(
        store, hub, FormattableString.Invariant($"http://127.0.0.1:{httpPort}"));
    await app.StartAsync();
    Console.WriteLine(FormattableString.Invariant($"ouvindo tcp {tcp.Port} http {app.Urls.First()}")
        + (brokerTarget is null ? string.Empty : $" broker {brokerTarget}"));

    await WaitForCtrlC();
    await app.StopAsync();
    if (bridge is not null)
    {
        await bridge.DisposeAsync();
    }

    return 0;
}
