using System.Buffers.Binary;

namespace SimTelemetry;

/// <summary>
/// Quadro CAN simulado, igual ao can_frame_t do firmware. No fio cada quadro ocupa 13 bytes:
/// id (uint32 little-endian), dlc (1 byte) e 8 bytes de dados (os que passam do dlc valem 0).
/// </summary>
public sealed class CanFrame
{
    public const int WireSize = 13;
    public const uint TelemetryId = 0x400;

    public CanFrame(uint id, byte[] data)
    {
        ArgumentNullException.ThrowIfNull(data);
        if (data.Length > 8)
        {
            throw new ArgumentException("CAN carrega no maximo 8 bytes.", nameof(data));
        }

        Id = id;
        Data = data;
    }

    public uint Id { get; }

    /// <summary>Os bytes de dados; o dlc e o tamanho do vetor.</summary>
    public byte[] Data { get; }

    public byte Dlc => (byte)Data.Length;

    /// <exception cref="FormatException">O dlc do registro passa de 8.</exception>
    public static CanFrame FromWire(ReadOnlySpan<byte> record)
    {
        if (record.Length != WireSize)
        {
            throw new ArgumentException($"Um registro tem {WireSize} bytes.", nameof(record));
        }

        int dlc = record[4];
        if (dlc > 8)
        {
            throw new FormatException($"dlc invalido: {dlc}.");
        }

        return new CanFrame(BinaryPrimitives.ReadUInt32LittleEndian(record), record.Slice(5, dlc).ToArray());
    }

    public byte[] ToWire()
    {
        var record = new byte[WireSize];
        BinaryPrimitives.WriteUInt32LittleEndian(record, Id);
        record[4] = Dlc;
        Data.CopyTo(record, 5);
        return record;
    }
}

/// <summary>Uma amostra de telemetria (quadro 0x400, 100 Hz), ja nas unidades fisicas.</summary>
public readonly record struct TelemetrySample(
    double SpeedMps, double Rpm, double Throttle, double Brake, int Gear, byte Flags)
{
    /// <summary>
    /// Decodifica um quadro 0x400 (dlc 8): velocidade u16 @0,01 m/s, giro u16 @1 rpm, acelerador e
    /// freio u8 @0,4%, marcha u8 e flags u8. Devolve false para qualquer outro quadro.
    /// </summary>
    public static bool TryDecode(CanFrame frame, out TelemetrySample sample)
    {
        sample = default;
        if (frame.Id != CanFrame.TelemetryId || frame.Dlc != 8)
        {
            return false;
        }

        ReadOnlySpan<byte> d = frame.Data;
        sample = new TelemetrySample(
            SpeedMps: BinaryPrimitives.ReadUInt16LittleEndian(d) * 0.01,
            Rpm: BinaryPrimitives.ReadUInt16LittleEndian(d[2..]),
            Throttle: d[4] * 0.004,
            Brake: d[5] * 0.004,
            Gear: d[6],
            Flags: d[7]);
        return true;
    }
}
