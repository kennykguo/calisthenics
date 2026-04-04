# ABOUTME: Provides shared fake arm-service fixtures for Python tests without real arm hardware.
# ABOUTME: Runs a tiny newline-delimited JSON socket server and records every request it receives.

from __future__ import annotations

import json
import socket
import threading
import time


def _split_response(entry: dict | tuple[float, dict]) -> tuple[float, dict]:
    if isinstance(entry, tuple):
        return entry
    return 0.0, entry


class FakeArmService:
    def __init__(self, responses: list[dict | tuple[float, dict]]) -> None:
        self._responses = list(responses)
        self.requests: list[dict] = []
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self.address: tuple[str, int] | None = None

    def start(self) -> None:
        self._thread.start()
        self._ready.wait(timeout=2.0)
        if self.address is None:
            raise RuntimeError("fake arm service did not publish an address")

    def stop(self) -> None:
        self._thread.join(timeout=2.0)

    def _serve(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            self.address = server.getsockname()
            self._ready.set()

            connection, _ = server.accept()
            with connection:
                reader = connection.makefile("r", encoding="utf-8", newline="\n")
                writer = connection.makefile("w", encoding="utf-8", newline="\n")

                for response_entry in self._responses:
                    request_line = reader.readline()
                    if not request_line:
                        break

                    self.requests.append(json.loads(request_line))
                    delay_s, response = _split_response(response_entry)
                    if delay_s > 0:
                        time.sleep(delay_s)
                    writer.write(f"{json.dumps(response)}\n")
                    writer.flush()
