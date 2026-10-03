"""Explicit download of a frozen, checksum-pinned CC BY-NC-SA research subset."""
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT=Path(__file__).resolve().parents[1]


def main():
    sources=json.loads((ROOT/'evaluation'/'sources.json').read_text(encoding='utf8'))
    destination=ROOT/'data'
    destination.mkdir(exist_ok=True)
    for source in sources:
        path=destination/f"{source['split']}-{Path(source['key']).name}"
        if path.exists() and hashlib.sha256(path.read_bytes()).hexdigest()==source['sha256']:
            continue
        if not source['url'].startswith('https://argoverse.s3.amazonaws.com/') or source['bytes']>500000:
            raise ValueError('unexpected source or size')
        with urllib.request.urlopen(source['url'],timeout=30) as response:
            data=response.read(500001)
        if len(data)!=source['bytes'] or hashlib.sha256(data).hexdigest()!=source['sha256']:
            raise RuntimeError('trajectory checksum mismatch')
        path.write_bytes(data)
    print('Verified 12 fixed Argoverse scenarios under CC BY-NC-SA 4.0; no account used.')


if __name__=='__main__':
    main()
