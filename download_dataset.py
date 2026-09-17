from urbaning.data import download_one_sequence
from pathlib import Path
datasets=["20241126_0024_crossing1_09", "20241126_0008_crossing1_01", "20241127_0000_crossing1_00"]
path = Path("./data")
path.mkdir(parents=True, exist_ok=True)
for name in datasets:
	download_one_sequence(download_dir=path.resolve(), sequence_name=name)
