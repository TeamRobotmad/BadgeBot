import os
from pathlib import Path

files_to_keep = {
    Path("app.py"),
    Path("hexdrive.py"),
    Path("tildagon.toml")
}

def _cosntruct_filepaths(dirname, filenames):
    return [Path(dirname, filename) for filename in filenames]

def find_files(top_level_dir):
    walkerator = iter(os.walk(top_level_dir))
    dirname, dirnames, filenames = next(walkerator)

    found_files = _cosntruct_filepaths(dirname, filenames)

    for dirname, dirnames, filenames  in walkerator:
        # if dirname not in dirs_to_keep:
         if ".git" not in dirname:
            found_files.extend(_cosntruct_filepaths(dirname, filenames))
            found_files.append(Path(dirname))

    print(found_files)
    return found_files


if __name__ == "__main__":

    found_files = set(find_files("."))

    if not files_to_keep.issubset(found_files):
        raise FileNotFoundError(f"Some of {files_to_keep} are not found so assuming wrong directory. "
                                "Please run this script from BadgeBot dir.")
    
    for file in found_files.difference(files_to_keep):
        print(file)
