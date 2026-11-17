using System.Net;
using System.Net.Sockets;

namespace SimTelemetry;

/// <summary>
/// Servidor TCP que recebe a telemetria da ECU: um fluxo de registros de 13 bytes (ver CanFrame) e
/// guarda os quadros de telemetria na sessao. Aceita varias conexoes; um registro com dlc invalido
/// e descartado sem derrubar a conexao.
/// </summary>
public sealed class TelemetryServer : IDisposable
{
    private readonly ITelemetrySink session;
    private readonly TcpListener listener;
    private readonly CancellationTokenSource stop = new();

    /// <param name="port">0 deixa o sistema escolher; a porta usada fica em <see cref="Port"/>.</param>
    public TelemetryServer(ITelemetrySink session, IPAddress? address = null, int port = 0)
    {
        this.session = session ?? throw new ArgumentNullException(nameof(session));
        listener = new TcpListener(address ?? IPAddress.Loopback, port);
    }

    public int Port => ((IPEndPoint)listener.LocalEndpoint).Port;

    public void Start()
    {
        listener.Start();
        _ = AcceptLoop(stop.Token);
    }

    public void Dispose()
    {
        stop.Cancel();
        listener.Stop();
        stop.Dispose();
    }

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
            // Servidor parado.
        }
    }

    private async Task Serve(TcpClient client, CancellationToken token)
    {
        using (client)
        {
            var record = new byte[CanFrame.WireSize];
            try
            {
                NetworkStream stream = client.GetStream();
                while (true)
                {
                    await stream.ReadExactlyAsync(record, token);
                    try
                    {
                        session.Add(CanFrame.FromWire(record));
                    }
                    catch (FormatException)
                    {
                        // Registro com dlc invalido: o fluxo continua alinhado, so esse quadro cai.
                    }
                }
            }
            catch (Exception e) when (e is EndOfStreamException or IOException or OperationCanceledException or ObjectDisposedException)
            {
                // Cliente saiu (inclusive no meio de um registro) ou servidor parado.
            }
        }
    }
}
