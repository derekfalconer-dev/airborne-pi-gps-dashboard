from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class CrtnStation:
    code: str
    station_name: str
    latitude: float
    longitude: float
    mountpoint: str
    host: str
    port: int
    data_format: str
    constellation: str
    zone: str
    distance_km: float = 0.0


def haversine_km(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
) -> float:
    radius_km = 6371.0088

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)

    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = (
        math.sin(delta_phi / 2) ** 2
        + math.cos(phi1)
        * math.cos(phi2)
        * math.sin(delta_lambda / 2) ** 2
    )

    return 2 * radius_km * math.asin(math.sqrt(a))


def load_crtn_stations(csv_path: str | Path) -> list[CrtnStation]:
    stations: list[CrtnStation] = []

    with open(csv_path, newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)

        for row in reader:
            try:
                station = CrtnStation(
                    code=row["code"].strip(),
                    station_name=row["station_name"].strip(),
                    latitude=float(row["latitude_deg"]),
                    longitude=float(row["longitude_deg"]),
                    mountpoint=row["mountpoint"].strip(),
                    host=row["ntrip_host"].strip(),
                    port=int(row["ntrip_port"]),
                    data_format=row["data_format"].strip(),
                    constellation=row["constellation"].strip(),
                    zone=row["ntrip_zone"].strip(),
                )
            except (KeyError, TypeError, ValueError):
                continue

            if not station.mountpoint:
                continue

            if "RTCM3" not in station.mountpoint.upper():
                continue

            stations.append(station)

    return stations


def nearest_crtn_stations(
    rover_latitude: float,
    rover_longitude: float,
    stations: list[CrtnStation],
    limit: int = 5,
    max_distance_km: float = 100.0,
) -> list[CrtnStation]:
    candidates: list[CrtnStation] = []

    for station in stations:
        distance = haversine_km(
            rover_latitude,
            rover_longitude,
            station.latitude,
            station.longitude,
        )

        if distance > max_distance_km:
            continue

        candidates.append(
            CrtnStation(
                code=station.code,
                station_name=station.station_name,
                latitude=station.latitude,
                longitude=station.longitude,
                mountpoint=station.mountpoint,
                host=station.host,
                port=station.port,
                data_format=station.data_format,
                constellation=station.constellation,
                zone=station.zone,
                distance_km=distance,
            )
        )

    candidates.sort(
        key=lambda station: (
            station.distance_km,
            "GNSS" not in station.constellation.upper(),
        )
    )

    return candidates[:limit]
