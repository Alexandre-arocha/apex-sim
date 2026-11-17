using System.Globalization;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Routing;
using Microsoft.Extensions.Logging;

namespace SimTelemetry;

/// <summary>Corpo de POST /sessions.</summary>
public sealed record NewSession(string? Name, double TrackLengthM);

public sealed record SessionSummary(
    long Id, string Name, double TrackLengthM, DateTime CreatedUtc, int Samples, bool Recording);

public sealed record SessionDetails(
    long Id, string Name, double TrackLengthM, DateTime CreatedUtc, int Samples, double DurationS,
    bool Recording, IReadOnlyList<Lap> Laps);

/// <summary>
/// API HTTP (JSON) de consulta das sessoes de telemetria. Sem autenticacao: por padrao escuta so no
/// loopback.
///   GET  /sessions                           lista
///   POST /sessions {name, trackLengthM}      abre uma sessao nova e a torna a que esta gravando
///   GET  /sessions/{id}                      resumo e voltas
///   GET  /sessions/{id}/laps                 voltas
///   GET  /sessions/{id}/channels/{name}      canal (speed, rpm, throttle, brake, gear); ?rate=Hz
///                                            &amp;from=s&amp;to=s, ou ?lap=N (tempo contado da largada da volta)
/// </summary>
public static class TelemetryApi
{
    private const double MaxPoints = 2_000_000;

    public static WebApplication CreateApp(SessionStore store, RecordingHub hub, string url)
    {
        WebApplicationBuilder builder = WebApplication.CreateSlimBuilder();
        builder.WebHost.UseUrls(url);
        builder.Logging.ClearProviders();
        WebApplication app = builder.Build();
        Map(app, store, hub);
        return app;
    }

    public static void Map(IEndpointRouteBuilder app, SessionStore store, RecordingHub hub)
    {
        Session? Open(long id) => hub.ActiveSession(id) ?? store.Load(id);

        SessionSummary Summarize(SessionInfo info)
        {
            Session? live = hub.ActiveSession(info.Id);
            return new SessionSummary(info.Id, info.Name, info.TrackLengthM, info.CreatedUtc,
                live?.Count ?? info.SampleCount, live is not null);
        }

        app.MapGet("/sessions", () => store.List().Select(Summarize));

        app.MapPost("/sessions", (NewSession body) =>
        {
            try
            {
                long id = hub.Start(body.Name ?? "sessao", body.TrackLengthM);
                return Results.Created($"/sessions/{id}", Summarize(store.Find(id)!));
            }
            catch (ArgumentException e)
            {
                return Results.BadRequest(new { error = e.Message });
            }
        });

        app.MapGet("/sessions/{id:long}", (long id) =>
        {
            SessionInfo? info = store.Find(id);
            Session? session = info is null ? null : Open(id);
            if (info is null || session is null)
            {
                return Results.NotFound(new { error = $"Sessao {id} inexistente." });
            }

            SessionSummary summary = Summarize(info);
            return Results.Ok(new SessionDetails(info.Id, info.Name, info.TrackLengthM, info.CreatedUtc,
                session.Count, session.Count / Session.SampleRateHz, summary.Recording, session.Laps()));
        });

        app.MapGet("/sessions/{id:long}/laps", (long id) =>
        {
            Session? session = store.Find(id) is null ? null : Open(id);
            return session is null
                ? Results.NotFound(new { error = $"Sessao {id} inexistente." })
                : Results.Ok(session.Laps());
        });

        app.MapGet("/sessions/{id:long}/channels/{name}", (long id, string name, HttpRequest request) =>
        {
            Session? session = store.Find(id) is null ? null : Open(id);
            if (session is null)
            {
                return Results.NotFound(new { error = $"Sessao {id} inexistente." });
            }

            if (!TryDouble(request.Query, "rate", out double? rate)
                || !TryDouble(request.Query, "from", out double? from)
                || !TryDouble(request.Query, "to", out double? to)
                || !TryInt(request.Query, "lap", out int? lap))
            {
                return Results.BadRequest(new { error = "Parametro numerico invalido (rate, from, to ou lap)." });
            }

            double rateHz = rate ?? 10;
            if (!(rateHz > 0) || session.Count / Session.SampleRateHz * rateHz > MaxPoints)
            {
                return Results.BadRequest(new { error = "rate invalido ou pedido grande demais." });
            }

            try
            {
                return Results.Ok(lap is int number
                    ? session.LapChannel(name, number, rateHz)
                    : session.Resample(name, rateHz, from, to));
            }
            catch (ArgumentException e)
            {
                return Results.BadRequest(new { error = e.Message });
            }
        });
    }

    private static bool TryDouble(IQueryCollection query, string key, out double? value)
    {
        value = null;
        if (!query.TryGetValue(key, out var raw) || raw.Count == 0)
        {
            return true;
        }

        if (double.TryParse(raw[0], NumberStyles.Float, CultureInfo.InvariantCulture, out double parsed)
            && double.IsFinite(parsed))
        {
            value = parsed;
            return true;
        }

        return false;
    }

    private static bool TryInt(IQueryCollection query, string key, out int? value)
    {
        value = null;
        if (!query.TryGetValue(key, out var raw) || raw.Count == 0)
        {
            return true;
        }

        if (int.TryParse(raw[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsed))
        {
            value = parsed;
            return true;
        }

        return false;
    }
}
