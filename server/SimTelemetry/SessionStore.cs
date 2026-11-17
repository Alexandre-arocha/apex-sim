using System.Globalization;
using Microsoft.Data.Sqlite;

namespace SimTelemetry;

/// <summary>Quem recebe quadros de telemetria: uma sessao, ou o gravador que escolhe a sessao ativa.</summary>
public interface ITelemetrySink
{
    /// <summary>Devolve true se o quadro foi guardado.</summary>
    bool Add(CanFrame frame);
}

/// <summary>Uma sessao guardada no banco.</summary>
public sealed record SessionInfo(long Id, string Name, double TrackLengthM, DateTime CreatedUtc, int SampleCount);

/// <summary>
/// Sessoes de telemetria em SQLite (um arquivo, ou ":memory:"). Uma tabela de sessoes e uma de
/// amostras (sessao, indice, canais); a amostra i de uma sessao e do instante (i + 1) / 100 s, entao os
/// indices de uma sessao sao sempre 0..n-1 sem buracos. Seguro para uso concorrente (uma conexao
/// serializada por um lock).
/// </summary>
public sealed class SessionStore : IDisposable
{
    private readonly object gate = new();
    private readonly SqliteConnection connection;

    public SessionStore(string path)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(path);
        connection = new SqliteConnection(new SqliteConnectionStringBuilder { DataSource = path }.ToString());
        connection.Open();
        Execute("PRAGMA journal_mode=WAL;");
        Execute("""
            CREATE TABLE IF NOT EXISTS sessions(
              id INTEGER PRIMARY KEY AUTOINCREMENT,
              name TEXT NOT NULL,
              track_length_m REAL NOT NULL,
              created_utc TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS samples(
              session_id INTEGER NOT NULL REFERENCES sessions(id),
              idx INTEGER NOT NULL,
              speed REAL NOT NULL,
              rpm REAL NOT NULL,
              throttle REAL NOT NULL,
              brake REAL NOT NULL,
              gear INTEGER NOT NULL,
              flags INTEGER NOT NULL,
              PRIMARY KEY(session_id, idx)) WITHOUT ROWID;
            """);
    }

    public long CreateSession(string name, double trackLengthM)
    {
        if (!double.IsFinite(trackLengthM) || trackLengthM <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(trackLengthM), "Comprimento invalido.");
        }

        string label = string.IsNullOrWhiteSpace(name) ? "sessao" : name.Trim();
        lock (gate)
        {
            using SqliteCommand command = connection.CreateCommand();
            command.CommandText =
                "INSERT INTO sessions(name, track_length_m, created_utc) VALUES($name, $length, $created); SELECT last_insert_rowid();";
            command.Parameters.AddWithValue("$name", label);
            command.Parameters.AddWithValue("$length", trackLengthM);
            command.Parameters.AddWithValue("$created", DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture));
            return (long)command.ExecuteScalar()!;
        }
    }

    /// <summary>
    /// Grava as amostras com os indices firstIndex, firstIndex + 1, ... (regravar um indice o substitui).
    /// </summary>
    /// <exception cref="KeyNotFoundException">A sessao nao existe.</exception>
    public void Append(long sessionId, int firstIndex, IReadOnlyList<TelemetrySample> batch)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(firstIndex);
        ArgumentNullException.ThrowIfNull(batch);
        lock (gate)
        {
            if (FindLocked(sessionId) is null)
            {
                throw new KeyNotFoundException($"Sessao {sessionId} inexistente.");
            }

            using SqliteTransaction transaction = connection.BeginTransaction();
            using SqliteCommand command = connection.CreateCommand();
            command.Transaction = transaction;
            command.CommandText = """
                INSERT OR REPLACE INTO samples(session_id, idx, speed, rpm, throttle, brake, gear, flags)
                VALUES($session, $idx, $speed, $rpm, $throttle, $brake, $gear, $flags)
                """;
            SqliteParameter idx = command.Parameters.Add("$idx", SqliteType.Integer);
            SqliteParameter speed = command.Parameters.Add("$speed", SqliteType.Real);
            SqliteParameter rpm = command.Parameters.Add("$rpm", SqliteType.Real);
            SqliteParameter throttle = command.Parameters.Add("$throttle", SqliteType.Real);
            SqliteParameter brake = command.Parameters.Add("$brake", SqliteType.Real);
            SqliteParameter gear = command.Parameters.Add("$gear", SqliteType.Integer);
            SqliteParameter flags = command.Parameters.Add("$flags", SqliteType.Integer);
            command.Parameters.AddWithValue("$session", sessionId);
            for (int i = 0; i < batch.Count; i++)
            {
                TelemetrySample s = batch[i];
                idx.Value = firstIndex + i;
                speed.Value = s.SpeedMps;
                rpm.Value = s.Rpm;
                throttle.Value = s.Throttle;
                brake.Value = s.Brake;
                gear.Value = s.Gear;
                flags.Value = (int)s.Flags;
                command.ExecuteNonQuery();
            }

            transaction.Commit();
        }
    }

    public SessionInfo? Find(long sessionId)
    {
        lock (gate)
        {
            return FindLocked(sessionId);
        }
    }

    public IReadOnlyList<SessionInfo> List()
    {
        lock (gate)
        {
            using SqliteCommand command = connection.CreateCommand();
            command.CommandText = SelectInfo + " ORDER BY s.id";
            using SqliteDataReader reader = command.ExecuteReader();
            var result = new List<SessionInfo>();
            while (reader.Read())
            {
                result.Add(ReadInfo(reader));
            }

            return result;
        }
    }

    /// <summary>Carrega a sessao inteira para consulta; null se nao existe.</summary>
    public Session? Load(long sessionId)
    {
        lock (gate)
        {
            SessionInfo? info = FindLocked(sessionId);
            if (info is null)
            {
                return null;
            }

            var session = new Session(info.TrackLengthM);
            using SqliteCommand command = connection.CreateCommand();
            command.CommandText =
                "SELECT speed, rpm, throttle, brake, gear, flags FROM samples WHERE session_id = $id ORDER BY idx";
            command.Parameters.AddWithValue("$id", sessionId);
            using SqliteDataReader reader = command.ExecuteReader();
            while (reader.Read())
            {
                session.AddSample(new TelemetrySample(
                    reader.GetDouble(0), reader.GetDouble(1), reader.GetDouble(2), reader.GetDouble(3),
                    reader.GetInt32(4), (byte)reader.GetInt32(5)));
            }

            return session;
        }
    }

    public void Dispose()
    {
        lock (gate)
        {
            connection.Dispose();
        }
    }

    private const string SelectInfo = """
        SELECT s.id, s.name, s.track_length_m, s.created_utc,
               (SELECT COUNT(*) FROM samples WHERE session_id = s.id) FROM sessions s
        """;

    private static SessionInfo ReadInfo(SqliteDataReader reader) => new(
        reader.GetInt64(0),
        reader.GetString(1),
        reader.GetDouble(2),
        DateTime.Parse(reader.GetString(3), CultureInfo.InvariantCulture, DateTimeStyles.RoundtripKind),
        reader.GetInt32(4));

    private SessionInfo? FindLocked(long sessionId)
    {
        using SqliteCommand command = connection.CreateCommand();
        command.CommandText = SelectInfo + " WHERE s.id = $id";
        command.Parameters.AddWithValue("$id", sessionId);
        using SqliteDataReader reader = command.ExecuteReader();
        return reader.Read() ? ReadInfo(reader) : null;
    }

    private void Execute(string sql)
    {
        using SqliteCommand command = connection.CreateCommand();
        command.CommandText = sql;
        command.ExecuteNonQuery();
    }
}
