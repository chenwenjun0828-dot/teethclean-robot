"""Linux TTY input adapter; never communicates with the robot."""
import os
import sys
import threading


class KeyboardControl:
    def __init__(self, stream=None):
        self.stream = sys.stdin if stream is None else stream
        self.lock = threading.RLock()
        self.started = threading.Event()
        self.closed = threading.Event()
        self.armed = False
        self.reason = None
        self.fd = self.attributes = self.thread = None

    def handle_key(self, key):
        with self.lock:
            if key == ' ':
                self.fail('operator_space')
            elif key == '\x04':
                self.fail('keyboard_input_eof')
            elif key in ('s', 'S') and self.armed and self.reason is None:
                self.started.set()

    def fail(self, reason):
        with self.lock:
            if self.reason is None:
                self.reason = reason

    def stop_reason(self):
        with self.lock:
            return self.reason

    def start_requested(self):
        return self.started.is_set()

    def arm(self):
        with self.lock:
            self.armed = False
            if self.fd is not None:
                while self.select.select([self.fd], [], [], 0)[0]:
                    self.read_keys()
                    if self.reason is not None:
                        break
            self.started.clear()
            self.armed = True
        print('Ready. Press S to start; Space to stop this run (no resume).',
              file=sys.stderr, flush=True)

    def __enter__(self):
        if os.name != 'posix' or not self.stream.isatty():
            raise ValueError('--keyboard requires a Linux foreground interactive TTY')
        import select
        import termios
        import tty
        self.select, self.termios = select, termios
        self.fd = self.stream.fileno()
        if os.tcgetpgrp(self.fd) != os.getpgrp():
            self.fd = None
            raise ValueError('--keyboard requires the foreground terminal process group')
        try:
            self.attributes = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd, when=termios.TCSANOW)
            self.thread = threading.Thread(target=self.listen, daemon=True)
            self.thread.start()
            print('Keyboard enabled; checking readiness. Space cancels or stops.',
                  file=sys.stderr, flush=True)
            return self
        except BaseException:
            self.close()
            raise

    def listen(self):
        try:
            while not self.closed.is_set():
                if not self.select.select([self.fd], [], [], .05)[0]:
                    continue
                with self.lock:
                    # arm() may have consumed pending input after select returned.
                    if not self.select.select([self.fd], [], [], 0)[0]:
                        continue
                    self.read_keys()
                    if self.reason is not None:
                        return
        except Exception as error:
            self.fail('keyboard_input_error: '+str(error))

    def read_keys(self):
        data = os.read(self.fd, 64)
        if not data:
            self.fail('keyboard_input_eof')
        for key in data.decode('ascii', errors='ignore'):
            self.handle_key(key)

    def close(self):
        self.closed.set()
        try:
            if self.thread is not None and self.thread.ident is not None:
                self.thread.join(timeout=.2)
        finally:
            try:
                if self.fd is not None and self.attributes is not None:
                    self.termios.tcsetattr(self.fd, self.termios.TCSANOW, self.attributes)
            finally:
                self.fd = self.attributes = None

    def __exit__(self, *_):
        self.close()
