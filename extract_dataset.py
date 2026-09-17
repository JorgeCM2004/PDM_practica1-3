import multivolumefile
import py7zr
from pathlib import Path

path = Path("./data/dataset")
for archive in sorted(path.glob("*.7z.001")):
	base = archive.with_suffix("")
	out_dir = path / base.stem
	out_dir.mkdir(exist_ok=True)
	with multivolumefile.open(base, mode="rb") as vol:
		with py7zr.SevenZipFile(vol, mode="r") as z:
			z.extractall(path=out_dir)
