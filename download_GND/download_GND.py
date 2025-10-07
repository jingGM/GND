import argparse
import os, sys, time, json, pathlib, requests
from urllib.parse import urljoin

SERVER = "https://dataverse.orc.gmu.edu"
PERSISTENT_ID = "doi:10.13021/ORC2020/JUIW5F"   # <-- your dataset DOI
API_TOKEN = os.getenv("DATAVERSE_API_TOKEN")     # optional

S = requests.Session()
if API_TOKEN:
    S.headers.update({"X-Dataverse-key": API_TOKEN})

def api_get(path, params=None, stream=False):
    url = urljoin(SERVER, path)
    for attempt in range(5):
        try:
            r = S.get(url, params=params, stream=stream, timeout=60)
            if r.status_code in (429, 500, 502, 503, 504):
                time.sleep(2**attempt)
                continue
            r.raise_for_status()
            return r
        except requests.RequestException as e:
            if attempt == 4:
                raise
            time.sleep(2**attempt)

def list_files(persistent_id):
    """Return list of dicts with file id, name, and subdir (directoryLabel)."""
    r = api_get("/api/datasets/:persistentId/versions/:latest/files",
                params={"persistentId": persistent_id})
    files = r.json().get("data", [])
    out = []
    for f in files:
        data = f.get("dataFile", {})
        file_id = data.get("id")
        label = f.get("label") or data.get("filename")
        subdir = f.get("directoryLabel") or ""
        if file_id and label:
            out.append({"id": file_id, "name": label, "subdir": subdir})
    return out

def download_file(fileinfo, root_dir):
    fid, name, subdir = fileinfo["id"], fileinfo["name"], fileinfo["subdir"]
    dest_dir = pathlib.Path(root_dir) / subdir
    dest_dir.mkdir(parents=True, exist_ok=True)
    dest_path = dest_dir / name

    # skip if already downloaded
    if dest_path.exists() and dest_path.stat().st_size > 0:
        return

    # original format (not tabular TSV conversion)
    r = api_get(f"/api/access/datafile/{fid}", params={"format": "original"}, stream=True)
    tmp = dest_path.with_suffix(dest_path.suffix + ".part")
    with open(tmp, "wb") as f:
        for chunk in r.iter_content(chunk_size=1024*1024):
            if chunk:
                f.write(chunk)
    tmp.rename(dest_path)

def main(directory):
    files = list_files(PERSISTENT_ID)
    if not files:
        print("No files visible. If dataset has restricted files, set DATAVERSE_API_TOKEN.")
        return
    print(f"Found {len(files)} files. Downloading to {directory} …")
    for i, finfo in enumerate(files, 1):
        rel = (finfo['subdir'] + '/' if finfo['subdir'] else '') + finfo['name']
        print(f"[{i}/{len(files)}] {rel}")
        download_file(finfo, directory)
    print("Done.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='process rosbags')
    parser.add_argument('--path', type=str, required=True, default="")
    arguments = parser.parse_args()
    main(arguments.path)
