import os
from py_gpmf_parser.gopro_telemetry_extractor import GoProTelemetryExtractor
import json
import numpy as np

SECS_TO_MS = 1e3


def convert_to_gopro_telemetry(payload: tuple, skip_seconds=0.):
    return {
        "samples": [{"value": data.tolist(), "cts": (ts*SECS_TO_MS).tolist()} for data, ts in zip(*payload)]
    }


if __name__ == "__main__":
    streams = ["ACCL", "GYRO", "GPS5", "GPSP", "GPSU", "GPSF", "GRAV", "MAGN", "CORI", "IORI", "TMPC"]
    filepath = 'raw_video.mp4'
    raw_telemetry_fp = "imu_raw.json"
    output_json_fp = "imu_data_2.json"
    output = {
        "1": {
            "streams": {},
        },
        "frames/second": 0.0
    }

    extractor = GoProTelemetryExtractor(filepath)
    extractor.open_source()
    for stream in streams:
        payload = extractor.extract_data(stream)
        data = convert_to_gopro_telemetry(payload)
        output["1"]["streams"][stream] = data

    with open(output_json_fp, "w") as f:
        json.dump(output, f, indent=2)

    extractor.close_source()
