import socket
import threading
from typing import Callable, Optional


class UDPServer:
    """Simple UDP text-command server for future robot-controller integration."""

    def __init__(self, bind_ip: str, port: int, on_command: Callable[[str, tuple], None]):
        self.bind_ip = bind_ip
        self.port = port
        self.on_command = on_command
        self._socket: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self):
        if self._running:
            return

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((self.bind_ip, self.port))
        self._socket.settimeout(0.2)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None

        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def _loop(self):
        while self._running and self._socket is not None:
            try:
                data, address = self._socket.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            command = data.decode("utf-8", errors="replace").strip()
            if command:
                self.on_command(command, address)
