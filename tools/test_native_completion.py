"""No hardware: exercise controller shutdown delay and malformed completion."""
import json
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time

root = Path(__file__).resolve().parents[1]
binary = root / "build/policy_native_stage2/test_policy_completion"
subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                str(root / "native/tests/test_policy_completion.cpp"), "-o", str(binary)], check=True)
results = []
for case in ("delayed_close", "extra_data", "timeout", "unread_final_target", "qualified_terminal_reset"):
    with tempfile.TemporaryDirectory(prefix="sprite_completion_fake_") as temp:
        path = str(Path(temp)/"state.sock")
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as server:
            server.bind(path)
            server.listen(1)
            server.settimeout(5)
            failures = []
            def peer():
                try:
                    conn, _ = server.accept()
                    with conn:
                        if case in ("unread_final_target","qualified_terminal_reset"):
                            conn.sendall(b'SPRT\x01\x00\x01\x00'+struct.pack('<QQQ',1,1000000000,1)+bytes(62*8))
                            time.sleep(.35)  # Deliberately close without consuming the last target.
                        elif case == "delayed_close":
                            time.sleep(.35)
                        elif case == "extra_data":
                            conn.sendall(b"unexpected")
                        else:
                            time.sleep(1.2)
                except Exception as exc:
                    failures.append(repr(exc))
            thread = threading.Thread(target=peer)
            thread.start()
            try:
                extra=['queued','qualified'] if case=='qualified_terminal_reset' else ['queued'] if case=='unread_final_target' else []
                result = subprocess.run([str(binary), path]+extra, capture_output=True, text=True, timeout=4)
            finally:
                thread.join(6)
            assert not failures and not thread.is_alive(), failures
            if case in ("delayed_close","qualified_terminal_reset"):
                assert result.returncode == 0, result.stderr
            else:
                assert result.returncode != 0
                assert ("recv=1" if case == "extra_data" else "errno=104" if case=='unread_final_target' else "timeout") in result.stderr, result.stderr
            results.append({"case": case, "passed": True, "stderr":result.stderr})
print(json.dumps({"hardware_tx": 0, "tests": results}, indent=2))
