"""
python scripts_slam_pipeline/01_extract_gopro_imu.py data_workspace/cup_in_the_wild/20240105_zhenjia_packard_2nd_conference_room
"""
# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import pathlib
import click
import multiprocessing
import concurrent.futures
import json
from tqdm import tqdm
from py_gpmf_parser.gopro_telemetry_extractor import GoProTelemetryExtractor

SECS_TO_MS = 1e3
def convert_to_gopro_telemetry(payload: tuple, skip_seconds=0.):
    return {
        "samples": [{"value": data.tolist(), "cts": (ts*SECS_TO_MS).tolist()} for data, ts in zip(*payload)]
    }

def extract_imu_from_video(video_dir):
    """Extract IMU data from a single video directory using py_gpmf_parser."""
    video_dir = pathlib.Path(video_dir).absolute()
    video_path = video_dir / 'raw_video.mp4'
    output_path = video_dir / 'imu_data.json'
    
    if not video_path.exists():
        raise FileNotFoundError(f"raw_video.mp4 not found in {video_dir}")
    
    streams = ["ACCL", "GYRO", "GPS5", "GPSP", "GPSU", "GPSF", "GRAV", "MAGN", "CORI", "IORI", "TMPC"]
    
    extractor = GoProTelemetryExtractor(str(video_path))
    try:
        extractor.open_source()
        
        output = {
            "1": {
                "streams": {},
            },
            "frames/second": 0.0
        }
        
        for stream in streams:
            payload = extractor.extract_data(stream)
            if payload and len(payload[0]) > 0:
                data = convert_to_gopro_telemetry(payload)
                output["1"]["streams"][stream] = data
        
        with open(output_path, 'w') as f:
            json.dump(output, f, indent=2)
            
        return True
    except Exception as e:
        print(f"Error processing {video_dir}: {str(e)}")
        return False
    finally:
        extractor.close_source()

# %%
@click.command()
@click.option('-n', '--num_workers', type=int, default=None)
@click.argument('session_dir', nargs=-1)
def main(num_workers, session_dir):
    if num_workers is None:
        num_workers = multiprocessing.cpu_count()

    for session in session_dir:
        input_dir = pathlib.Path(os.path.expanduser(session)).joinpath('demos')
        input_video_dirs = [x.parent for x in input_dir.glob('*/raw_video.mp4')]
        print(f'Found {len(input_video_dirs)} video dirs')

        with tqdm(total=len(input_video_dirs)) as pbar:
            with concurrent.futures.ThreadPoolExecutor(max_workers=num_workers) as executor:
                futures = set()
                for video_dir in input_video_dirs:
                    video_dir = video_dir.absolute()
                    if video_dir.joinpath('imu_data.json').is_file():
                        print(f"imu_data.json already exists, skipping {video_dir.name}")
                        pbar.update(1)
                        continue

                    if len(futures) >= num_workers:
                        completed, futures = concurrent.futures.wait(futures, 
                            return_when=concurrent.futures.FIRST_COMPLETED)
                        pbar.update(len(completed))

                    futures.add(executor.submit(extract_imu_from_video, video_dir))

                completed, futures = concurrent.futures.wait(futures)
                pbar.update(len(completed))

        results = [x.result() for x in completed]
        successful = sum(results)
        total = len(results)
        print(f"Done! Successfully processed {successful}/{total} videos")

# %%
if __name__ == "__main__":
    main()
