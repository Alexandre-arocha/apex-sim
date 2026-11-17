using System.Net;
using System.Net.Http.Json;
using System.Net.Sockets;
using System.Text.Json;
using Microsoft.AspNetCore.Builder;
using SimTelemetry;
using Xunit;

namespace SimTelemetry.Tests;

public class SessionStoreTests
{
    private static Session Ramp(int count)
    {
        var session = new Session(1000);
        for (int i = 1; i <= count; i++)
        {
            session.Add(Synthetic.Telemetry(i * 0.37 % 90, 8000 + i, i % 250, (i * 7) % 250, 1 + i % 8, i % 4));
        }

        return session;
    }

    [Fact]
    public void RoundTripsEverySampleExactly()
    {
        Session original = Ramp(250);
        TelemetrySample[] samples = original.SamplesFrom(0);
        using var store = new SessionStore(":memory:");
        long id = store.CreateSession("teste", 1000);
        store.Append(id, 0, samples[..100]);
        store.Append(id, 100, samples[100..]);

        Session loaded = store.Load(id)!;
        Assert.Equal(250, loaded.Count);
        Assert.Equal(samples, loaded.SamplesFrom(0));
        Assert.Equal(original.Laps().Count, loaded.Laps().Count);
        Assert.Equal(original.Channel("rpm").Value, loaded.Channel("rpm").Value);
        Assert.Equal(250, store.Find(id)!.SampleCount);
    }

    [Fact]
    public void KeepsSessionsApartAndListsThemInOrder()
    {
        using var store = new SessionStore(":memory:");
        long a = store.CreateSession("primeira", 1000);
        long b = store.CreateSession("  segunda  ", 2000);
        store.Append(a, 0, Ramp(10).SamplesFrom(0));
        store.Append(b, 0, Ramp(30).SamplesFrom(0));

        IReadOnlyList<SessionInfo> all = store.List();
        Assert.Equal([a, b], all.Select(s => s.Id));
        Assert.Equal(["primeira", "segunda"], all.Select(s => s.Name));
        Assert.Equal([10, 30], all.Select(s => s.SampleCount));
        Assert.Equal(2000, store.Find(b)!.TrackLengthM);
    }

    [Fact]
    public void WritingTheSameIndexesAgainDoesNotDuplicate()
    {
        using var store = new SessionStore(":memory:");
        long id = store.CreateSession("x", 1000);
        TelemetrySample[] samples = Ramp(20).SamplesFrom(0);
        store.Append(id, 0, samples);
        store.Append(id, 0, samples);
        Assert.Equal(20, store.Find(id)!.SampleCount);
    }

    [Fact]
    public void RejectsMissingSessionsAndInvalidInput()
    {
        using var store = new SessionStore(":memory:");
        Assert.Null(store.Find(99));
        Assert.Null(store.Load(99));
        Assert.Throws<KeyNotFoundException>(() => store.Append(99, 0, Ramp(1).SamplesFrom(0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => store.CreateSession("x", 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => store.CreateSession("x", double.NaN));
        long id = store.CreateSession("", 1000);
        Assert.Equal("sessao", store.Find(id)!.Name);
        Assert.Throws<ArgumentOutOfRangeException>(() => store.Append(id, -1, []));
    }

    [Fact]
    public void SurvivesClosingAndReopeningTheFile()
    {
        string path = Path.Combine(Path.GetTempPath(), $"sim-telemetry-{Guid.NewGuid():N}.db");
        try
        {
            long id;
            using (var store = new SessionStore(path))
            {
                id = store.CreateSession("em arquivo", 1000);
                store.Append(id, 0, Ramp(120).SamplesFrom(0));
            }

            using var reopened = new SessionStore(path);
            Assert.Equal(120, reopened.Find(id)!.SampleCount);
            Assert.Equal("em arquivo", reopened.Find(id)!.Name);
            Assert.Equal(120, reopened.Load(id)!.Count);
        }
        finally
        {
            Microsoft.Data.Sqlite.SqliteConnection.ClearAllPools();
            foreach (string file in new[] { path, path + "-wal", path + "-shm" })
            {
                try
                {
                    File.Delete(file);
                }
                catch (IOException)
                {
                    // Arquivo temporario: o sistema limpa depois.
                }
            }
        }
    }
}

public class RecordingHubTests
{
    [Fact]
    public void DropsTelemetryUntilASessionIsStartedThenRecordsIt()
    {
        using var store = new SessionStore(":memory:");
        using var hub = new RecordingHub(store);
        Assert.False(hub.Add(Synthetic.Telemetry(10, 8000, 0, 0, 1)));
        Assert.False(hub.Add(new CanFrame(0x300, new byte[7])));  // outro id nao conta como perda
        Assert.Equal(1, hub.DroppedFrames);
        Assert.Null(hub.ActiveId);

        long id = hub.Start("pista", 1000);
        Assert.Equal(id, hub.ActiveId);
        for (int i = 0; i < 300; i++)
        {
            Assert.True(hub.Add(Synthetic.Telemetry(20, 9000, 0, 0, 2)));
        }

        hub.Flush();
        Assert.Equal(300, store.Find(id)!.SampleCount);
        for (int i = 0; i < 50; i++)
        {
            hub.Add(Synthetic.Telemetry(20, 9000, 0, 0, 2));
        }

        hub.Flush();
        Assert.Equal(350, store.Find(id)!.SampleCount);  // so o que faltava, sem repetir
    }

    [Fact]
    public async Task FlushesInTheBackgroundAndWhenAnotherSessionStarts()
    {
        using var store = new SessionStore(":memory:");
        using var hub = new RecordingHub(store, TimeSpan.FromMilliseconds(20));
        long first = hub.Start("a", 1000);
        for (int i = 0; i < 40; i++)
        {
            hub.Add(Synthetic.Telemetry(20, 9000, 0, 0, 2));
        }

        for (int i = 0; i < 200 && store.Find(first)!.SampleCount < 40; i++)
        {
            await Task.Delay(10);
        }

        Assert.Equal(40, store.Find(first)!.SampleCount);  // sem chamar Flush

        hub.Add(Synthetic.Telemetry(20, 9000, 0, 0, 2));
        long second = hub.Start("b", 500);  // grava o que sobrou da primeira antes de trocar
        Assert.Equal(41, store.Find(first)!.SampleCount);
        Assert.Null(hub.ActiveSession(first));
        Assert.NotNull(hub.ActiveSession(second));
        Assert.Equal(0, hub.ActiveSession(second)!.Count);
    }

    [Fact]
    public void ClosingTheHubFlushesWhatIsPending()
    {
        using var store = new SessionStore(":memory:");
        long id;
        using (var hub = new RecordingHub(store, TimeSpan.FromHours(1)))
        {
            id = hub.Start("a", 1000);
            for (int i = 0; i < 25; i++)
            {
                hub.Add(Synthetic.Telemetry(20, 9000, 0, 0, 2));
            }
        }

        Assert.Equal(25, store.Find(id)!.SampleCount);
    }
}

public sealed class ApiHost : IAsyncDisposable
{
    private ApiHost(SessionStore store, RecordingHub hub, WebApplication app, HttpClient http)
    {
        Store = store;
        Hub = hub;
        App = app;
        Http = http;
    }

    public SessionStore Store { get; }

    public RecordingHub Hub { get; }

    public WebApplication App { get; }

    public HttpClient Http { get; }

    public static async Task<ApiHost> StartAsync()
    {
        var store = new SessionStore(":memory:");
        var hub = new RecordingHub(store, TimeSpan.FromMilliseconds(50));
        WebApplication app = TelemetryApi.CreateApp(store, hub, "http://127.0.0.1:0");
        await app.StartAsync();
        var http = new HttpClient { BaseAddress = new Uri(app.Urls.First()) };
        return new ApiHost(store, hub, app, http);
    }

    public async ValueTask DisposeAsync()
    {
        Http.Dispose();
        await App.StopAsync();
        await App.DisposeAsync();
        Hub.Dispose();
        Store.Dispose();
    }
}

public class HttpApiTests
{
    private static async Task<JsonElement> Json(HttpResponseMessage response) =>
        await response.Content.ReadFromJsonAsync<JsonElement>();

    private static async Task<long> StartSession(ApiHost api, double trackLengthM, string name = "teste")
    {
        HttpResponseMessage created = await api.Http.PostAsJsonAsync("/sessions", new { name, trackLengthM });
        Assert.Equal(HttpStatusCode.Created, created.StatusCode);
        return (await Json(created)).GetProperty("id").GetInt64();
    }

    [Fact]
    public async Task CreatesListsAndDescribesSessions()
    {
        await using ApiHost api = await ApiHost.StartAsync();
        long id = await StartSession(api, 1000, "treino");

        JsonElement list = await Json(await api.Http.GetAsync("/sessions"));
        Assert.Equal(1, list.GetArrayLength());
        Assert.Equal("treino", list[0].GetProperty("name").GetString());
        Assert.True(list[0].GetProperty("recording").GetBoolean());

        for (int i = 0; i < 6500; i++)  // 50 m/s numa pista de 1000 m: 3 voltas de 20 s
        {
            api.Hub.Add(Synthetic.Telemetry(50, 9000, 100, 0, 3));
        }

        JsonElement details = await Json(await api.Http.GetAsync($"/sessions/{id}"));
        Assert.Equal(6500, details.GetProperty("samples").GetInt32());
        Assert.Equal(65.0, details.GetProperty("durationS").GetDouble(), 1e-9);
        Assert.Equal(3, details.GetProperty("laps").GetArrayLength());
        Assert.Equal(20.0, details.GetProperty("laps")[1].GetProperty("durationS").GetDouble(), 0.02);

        JsonElement laps = await Json(await api.Http.GetAsync($"/sessions/{id}/laps"));
        Assert.Equal(3, laps.GetArrayLength());
        Assert.Equal(1, laps[0].GetProperty("number").GetInt32());
    }

    [Fact]
    public async Task ServesResampledChannelsAndSingleLaps()
    {
        await using ApiHost api = await ApiHost.StartAsync();
        long id = await StartSession(api, 1000);
        for (int i = 0; i < 4500; i++)
        {
            api.Hub.Add(Synthetic.Telemetry(50, 9000, 100, 0, 3));
        }

        JsonElement speed = await Json(await api.Http.GetAsync($"/sessions/{id}/channels/speed?rate=10&from=1&to=2"));
        Assert.Equal(11, speed.GetProperty("time").GetArrayLength());
        Assert.Equal(1.0, speed.GetProperty("time")[0].GetDouble(), 1e-9);
        Assert.Equal(50.0, speed.GetProperty("value")[5].GetDouble(), 1e-9);

        JsonElement lap = await Json(await api.Http.GetAsync($"/sessions/{id}/channels/rpm?lap=2&rate=5"));
        Assert.Equal(0.0, lap.GetProperty("time")[0].GetDouble(), 1e-9);
        Assert.InRange(lap.GetProperty("time").GetArrayLength(), 99, 101);  // 20 s a 5 Hz
        Assert.Equal(9000.0, lap.GetProperty("value")[10].GetDouble(), 1e-9);

        JsonElement defaulted = await Json(await api.Http.GetAsync($"/sessions/{id}/channels/gear"));
        Assert.Equal(450, defaulted.GetProperty("time").GetArrayLength());  // de 0,01 s a 45,00 s a 10 Hz
    }

    [Fact]
    public async Task AnswersBadRequestsAndMissingThings()
    {
        await using ApiHost api = await ApiHost.StartAsync();
        long id = await StartSession(api, 1000);
        for (int i = 0; i < 2500; i++)
        {
            api.Hub.Add(Synthetic.Telemetry(50, 9000, 100, 0, 3));
        }

        Assert.Equal(HttpStatusCode.NotFound, (await api.Http.GetAsync("/sessions/999")).StatusCode);
        Assert.Equal(HttpStatusCode.NotFound, (await api.Http.GetAsync("/sessions/999/laps")).StatusCode);
        Assert.Equal(HttpStatusCode.NotFound, (await api.Http.GetAsync("/sessions/999/channels/speed")).StatusCode);
        Assert.Equal(HttpStatusCode.NotFound, (await api.Http.GetAsync("/sessions/abc")).StatusCode);
        foreach (string bad in new[]
        {
            $"/sessions/{id}/channels/temperature",        // canal desconhecido
            $"/sessions/{id}/channels/speed?rate=0",
            $"/sessions/{id}/channels/speed?rate=-3",
            $"/sessions/{id}/channels/speed?rate=abc",
            $"/sessions/{id}/channels/speed?from=x",
            $"/sessions/{id}/channels/speed?lap=9",         // a sessao tem 1 volta
            $"/sessions/{id}/channels/speed?lap=0",
            $"/sessions/{id}/channels/speed?rate=1e9",      // pedido grande demais
        })
        {
            Assert.Equal(HttpStatusCode.BadRequest, (await api.Http.GetAsync(bad)).StatusCode);
        }

        Assert.Equal(HttpStatusCode.OK, (await api.Http.GetAsync($"/sessions/{id}/channels/speed?lap=1")).StatusCode);
        Assert.Equal(HttpStatusCode.BadRequest,
            (await api.Http.PostAsJsonAsync("/sessions", new { name = "x", trackLengthM = 0 })).StatusCode);
        Assert.Equal(HttpStatusCode.BadRequest,
            (await api.Http.PostAsJsonAsync("/sessions", new { name = "x", trackLengthM = -5 })).StatusCode);
    }

    [Fact]
    public async Task OlderSessionsAreServedFromTheDatabaseAfterANewOneStarts()
    {
        await using ApiHost api = await ApiHost.StartAsync();
        long first = await StartSession(api, 1000, "antiga");
        for (int i = 0; i < 4500; i++)
        {
            api.Hub.Add(Synthetic.Telemetry(50, 9000, 100, 0, 3));
        }

        long second = await StartSession(api, 500, "nova");
        Assert.NotEqual(first, second);

        JsonElement old = await Json(await api.Http.GetAsync($"/sessions/{first}"));
        Assert.False(old.GetProperty("recording").GetBoolean());
        Assert.Equal(4500, old.GetProperty("samples").GetInt32());
        Assert.Equal(2, old.GetProperty("laps").GetArrayLength());
        JsonElement fresh = await Json(await api.Http.GetAsync($"/sessions/{second}"));
        Assert.True(fresh.GetProperty("recording").GetBoolean());
        Assert.Equal(0, fresh.GetProperty("samples").GetInt32());
        Assert.Equal(2, (await Json(await api.Http.GetAsync("/sessions"))).GetArrayLength());
    }
}

/// <summary>
/// Ponta a ponta com a telemetria real do firmware: TCP -> gravador -> banco -> HTTP.
/// </summary>
public class EndToEndTests
{
    [Fact]
    public async Task RealTelemetryFlowsFromTcpToTheDatabaseAndTheApi()
    {
        byte[] bytes = File.ReadAllBytes(Path.Combine(AppContext.BaseDirectory, "Fixtures", "reference_2laps.can"));
        await using ApiHost api = await ApiHost.StartAsync();
        long id = await StartSessionFor(api, 2215.0147);
        using var tcp = new TelemetryServer(api.Hub);
        tcp.Start();

        using var client = new TcpClient();
        await client.ConnectAsync(IPAddress.Loopback, tcp.Port);
        await client.GetStream().WriteAsync(bytes);
        int expected = bytes.Length / CanFrame.WireSize;
        for (int i = 0; i < 1000 && api.Hub.ActiveSession(id)!.Count < expected; i++)
        {
            await Task.Delay(10);
        }

        JsonElement laps = await api.Http.GetFromJsonAsync<JsonElement>($"/sessions/{id}/laps");
        Assert.Equal(2, laps.GetArrayLength());
        Assert.Equal(49.000, laps[0].GetProperty("durationS").GetDouble(), 0.05);
        Assert.Equal(45.309, laps[1].GetProperty("durationS").GetDouble(), 0.05);

        api.Hub.Flush();
        Assert.Equal(expected, api.Store.Find(id)!.SampleCount);
        Session fromDatabase = api.Store.Load(id)!;
        Assert.Equal(laps.GetArrayLength(), fromDatabase.Laps().Count);
        Assert.Equal(laps[1].GetProperty("durationS").GetDouble(), fromDatabase.Laps()[1].DurationS, 1e-9);

        JsonElement lapTwoSpeed = await api.Http.GetFromJsonAsync<JsonElement>($"/sessions/{id}/channels/speed?lap=2&rate=10");
        double top = lapTwoSpeed.GetProperty("value").EnumerateArray().Max(v => v.GetDouble());
        Assert.InRange(top * 3.6, 295, 312);
    }

    private static async Task<long> StartSessionFor(ApiHost api, double trackLengthM)
    {
        HttpResponseMessage created = await api.Http.PostAsJsonAsync("/sessions", new { name = "real", trackLengthM });
        Assert.Equal(HttpStatusCode.Created, created.StatusCode);
        return (await created.Content.ReadFromJsonAsync<JsonElement>()).GetProperty("id").GetInt64();
    }
}
