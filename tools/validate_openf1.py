#!/usr/bin/env python3
"""Compara o envelope do carro simulado com telemetria real da OpenF1 (api.openf1.org).

A pista do simulador e ficticia, entao so da para comparar o que nao depende dela: velocidade
maxima, giro, marchas, aceleracao a partir da largada e frenagem de pico. Uso:
    python tools/validate_openf1.py [ano] [circuito] [numero-do-piloto]
Os dados vem da API publica, sem chave; nada e gravado em disco. So biblioteca padrao.
"""
import json
import sys
import urllib.parse
import urllib.request
from datetime import datetime, timedelta

API = "https://api.openf1.org/v1/"
YEAR = int(sys.argv[1]) if len(sys.argv) > 1 else 2024
CIRCUIT = sys.argv[2] if len(sys.argv) > 2 else "Monza"
DRIVER = int(sys.argv[3]) if len(sys.argv) > 3 else 1


def get(endpoint, **query):
    url = API + endpoint + "?" + urllib.parse.urlencode(query, safe=">=<:+")
    with urllib.request.urlopen(url, timeout=60) as response:
        return json.load(response)


def when(text):
    return datetime.fromisoformat(text)


def crossing_time(samples, kmh):
    """Primeiro instante (s desde samples[0]) em que a velocidade chega a kmh, por interpolacao."""
    t0 = when(samples[0]["date"])
    for a, b in zip(samples, samples[1:]):
        if a["speed"] < kmh <= b["speed"]:
            ta, tb = (when(a["date"]) - t0).total_seconds(), (when(b["date"]) - t0).total_seconds()
            return ta + (tb - ta) * (kmh - a["speed"]) / (b["speed"] - a["speed"])
    return None


def peak_braking_g(samples):
    """Maior desaceleracao media entre amostras seguidas com o freio acionado, de mais de 200 km/h."""
    best = 0.0
    for a, b in zip(samples, samples[1:]):
        dt = (when(b["date"]) - when(a["date"])).total_seconds()
        if a["brake"] and b["brake"] and a["speed"] > 200 and 0 < dt < 0.5:
            best = max(best, (a["speed"] - b["speed"]) / 3.6 / dt / 9.80665)
    return best


def main():
    session = get("sessions", year=YEAR, session_name="Race", circuit_short_name=CIRCUIT)[0]
    key = session["session_key"]
    print(f"{CIRCUIT} {YEAR}, corrida, piloto {DRIVER} (session_key {key})")

    # Largada: da luz verde (date_start da sessao) a 25 s depois.
    start = when(session["date_start"])
    launch = get("car_data", session_key=key, driver_number=DRIVER,
                 **{"date>": start.isoformat(), "date<": (start + timedelta(seconds=25)).isoformat()})
    # Volta mais rapida da corrida.
    laps = [l for l in get("laps", session_key=key, driver_number=DRIVER) if l.get("lap_duration")]
    fast = min(laps, key=lambda l: l["lap_duration"])
    lap_start = when(fast["date_start"])
    lap = get("car_data", session_key=key, driver_number=DRIVER,
              **{"date>": lap_start.isoformat(),
                 "date<": (lap_start + timedelta(seconds=fast["lap_duration"])).isoformat()})

    # A largada e a ultima amostra parada antes de o carro cruzar 100 km/h (a janela pode pegar o
    # carro ainda se mexendo na grade).
    fast_at = next(i for i, s in enumerate(launch) if s["speed"] >= 100)
    standing = max(i for i in range(fast_at) if launch[i]["speed"] <= 2)
    run = launch[standing:]
    print(f"amostras: largada={len(launch)} volta rapida (volta {fast['lap_number']}, "
          f"{fast['lap_duration']:.3f} s)={len(lap)}, ~{len(lap) / fast['lap_duration']:.1f} Hz")
    for kmh in (100, 200, 300):
        t = crossing_time(run, kmh)
        print(f"0-{kmh} km/h na largada: " + (f"{t:.2f} s" if t is not None else "nao chegou na janela"))
    print(f"velocidade maxima na volta rapida: {max(s['speed'] for s in lap)} km/h")
    print(f"giro maximo: {max(s['rpm'] for s in lap)} rpm; marcha maxima: {max(s['n_gear'] for s in lap)}")
    print(f"frenagem de pico (media entre amostras, >200 km/h): {peak_braking_g(lap):.1f} g")


if __name__ == "__main__":
    main()
