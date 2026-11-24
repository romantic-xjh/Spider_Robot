import urllib.request, os, time, sys, tarfile, shutil

URL = "https://codeload.github.com/espressif/esp-sr/tar.gz/refs/tags/v1.2.0"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "esp-sr-v1.2.0.tar.gz")
BASE = os.path.dirname(os.path.abspath(__file__))


def extract():
    """把 esp-sr-v1.2.0.tar.gz 解压到 components/esp-sr"""
    dst = os.path.join(BASE, "components", "esp-sr")
    os.makedirs(os.path.join(BASE, "components"), exist_ok=True)
    with tarfile.open(OUT, "r:gz") as tar:
        top = tar.getnames()[0].split("/")[0]      # esp-sr-1.2.0
        tar.extractall(path=BASE)                  # 解压出 esp-sr-1.2.0/
    src = os.path.join(BASE, top)
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.move(src, dst)
    print("EXTRACTED -> %s" % dst, flush=True)


def main():
    for attempt in range(1, 51):
        try:
            existing = os.path.getsize(OUT) if os.path.exists(OUT) else 0
            req = urllib.request.Request(URL, headers={"User-Agent": "Mozilla/5.0"})
            if existing:
                req.add_header("Range", "bytes=%d-" % existing)
            with urllib.request.urlopen(req, timeout=60) as r:
                status = getattr(r, "status", 200)
                mode = "ab" if (existing and status == 206) else "wb"
                total = r.headers.get("Content-Length")
                if total:
                    print("attempt %d: resuming=%d status=%d content-length=%s" %
                          (attempt, existing, status, total), flush=True)
                got = 0
                with open(OUT, mode) as f:
                    while True:
                        chunk = r.read(1 << 16)
                        if not chunk:
                            break
                        f.write(chunk)
                        got += len(chunk)
                size = os.path.getsize(OUT)
                print("attempt %d: downloaded this round=%d total=%d" % (attempt, got, size), flush=True)
                # success if we got a non-trivial file and stream ended cleanly
                if got == 0 or (total and size >= int(total)):
                    print("DONE %d bytes" % size, flush=True)
                    extract()
                    return
                # partial (no content-length / chunked), try continue once more
        except Exception as e:
            print("attempt %d failed: %r" % (attempt, e), flush=True)
            time.sleep(3)
    print("GIVEUP", flush=True)
    sys.exit(1)


if __name__ == "__main__":
    main()
