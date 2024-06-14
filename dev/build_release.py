import os
import sys
from pathlib import Path

import mpy_cross

files_to_mpy = {
    Path("hexdrive.py"),
}

files_to_keep = {
    Path("app.py"),
    Path("tildagon.toml"),
    Path("metadata.json")
}

def _cosntruct_filepaths(dirname, filenames):
    return [Path(dirname, filename) for filename in filenames]

def find_files(top_level_dir):
    walkerator = iter(os.walk(top_level_dir))
    dirname, dirnames, filenames = next(walkerator)

    found_files = _cosntruct_filepaths(dirname, filenames)

    for dirname, dirnames, filenames  in walkerator:
        # if dirname not in dirs_to_keep:
        if dirname != "./.git" and ".git/" not in dirname:
            found_files.extend(_cosntruct_filepaths(dirname, filenames))

    return found_files


if __name__ == "__main__":

    force_mode = False

    if len(sys.argv) >= 2:
        force_mode = sys.argv[1] == "-f"

    found_files = set(find_files("."))

    if not files_to_keep.issubset(found_files):
        raise FileNotFoundError(f"Some of {files_to_keep} are not found so assuming wrong directory. "
                                "Please run this script from BadgeBot dir.")
    
    for file in files_to_mpy:
        print(f"Mpy-ing file: {file}")
        mpy_cross.run(file, "-v")

    files_to_remove = found_files.difference(files_to_keep)
    if not force_mode:
        if input(f"About to remove {len(files_to_remove)} files from {os.getcwd()}, continue? y/n") != "y":
            print("Aborting file removal")
            exit(0)

    for file in files_to_remove:
        print(f"Removing file: {file}")
        os.remove(file)
