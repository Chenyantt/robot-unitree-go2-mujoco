#!/usr/bin/env python3
"""Install pinned optional named-action ONNX files with digest verification."""
from __future__ import annotations

import hashlib
import json
import os
import tempfile
import time
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LOCK = ROOT / "stunt-assets.lock.json"
DESTINATION = ROOT / ".runtime/stunt-assets"


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def main() -> None:
    manifest = json.loads(LOCK.read_text(encoding="utf-8"))
    for item in manifest["assets"]:
        target = DESTINATION / item["path"]
        if target.is_file() and digest(target) == item["sha256"]:
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        errors = []
        # Prefer deployment mirrors; the canonical URL remains the provenance
        # source and every byte is accepted only after the pinned SHA-256 check.
        for url in [*item.get("mirrors", []), item["url"]]:
            for attempt in range(2):
                descriptor, temporary_name = tempfile.mkstemp(dir=target.parent, prefix=".download-")
                os.close(descriptor)
                temporary = Path(temporary_name)
                try:
                    request = urllib.request.Request(url, headers={"User-Agent": "robonix-go2-assets/1"})
                    with urllib.request.urlopen(request, timeout=600) as response, temporary.open("wb") as output:
                        while block := response.read(1024 * 1024):
                            output.write(block)
                    actual = digest(temporary)
                    if actual != item["sha256"]:
                        raise RuntimeError(f"digest mismatch: {actual}")
                    temporary.replace(target)
                    break
                except Exception as error:  # noqa: BLE001
                    errors.append(f"{url} attempt {attempt+1}: {error}")
                    time.sleep(attempt+1)
                finally:
                    temporary.unlink(missing_ok=True)
            if target.is_file() and digest(target) == item["sha256"]:
                break
        else:
            raise RuntimeError(f"failed to install {item['path']}: {'; '.join(errors)}")
        print(f"installed {item['path']}")


if __name__ == "__main__":
    main()
