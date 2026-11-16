"""The PC end of netring.irx's fast path (iop/netring/netring.c): a stream sent as UDP datagrams the PS2 takes straight
from its Ethernet driver. Go back N over UDP, one client at a time.

The PS2 opens with the request text ("fast <KB/s> <what>"), then every few ms sends {'F', u32 next sequence wanted,
u16 how many more may be sent, 2 x 0, u32 flags (1: it saw a gap)}; 'X' ends. We send {'P', 'S', u32 sequence, up to
1408 bytes}, paced at the asked rate and never past what it allows, and after the last one {'P', 'E', u32 sequence}.
A gap, or 100 ms without it moving on while datagrams are out, sends again from the one it wants.
"""
import socket
import struct
import threading
import time

PAY = 1408


def serve(port, open_stream, name="fast"):
    """open_stream(what) -> the stream, or None to refuse: a function read(n), or an object with read(n) and, if
    it likes, close(). read gives the next bytes: n of them; fewer when it wants the PS2 to have them at once (a live
    picture's end: keep those on multiples of 16 bytes of the stream); None for nothing yet; b"" at the end."""
    threading.Thread(target=_run, args=(port, open_stream, name), daemon=True).start()


def _run(port, open_stream, name):
    u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    u.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
    u.bind(("", port))
    u.setblocking(False)
    client = None  # the stream being sent
    while True:
        try:
            data, addr = u.recvfrom(64)
        except (BlockingIOError, ConnectionResetError):
            data = None
        now = time.perf_counter()
        if data:
            if data[:4] == b"fast":
                # the PS2 says its request again until our first datagram reaches it, and a live source can take
                # seconds to give one. The same request is the same stream: starting another under a PS2 that had
                # taken datagrams of the first (console, 2026-11-15) fed it two streams spliced, and it froze
                if client and client.addr == addr and client.req == data:
                    client.heard = now
                    continue
                if client:
                    client.report(name)
                parts = data.split()
                read = open_stream(parts[2].decode() if len(parts) > 2 else "")
                client = _Client(addr, max(1, int(parts[1])) * 1024, read, now) if read else None
                if client:
                    client.req = data
            elif client and addr == client.addr:
                if data[:1] == b"F" and len(data) >= 13:
                    client.feedback(*struct.unpack_from("<IHxxI", data, 1), now)
                elif data[:1] == b"X":
                    client.report(name)
                    client = None
            continue
        if client and now - client.heard > 5:  # gone (switched off: no 'X'). Checked whether or not there is
            client.report(name)                # something to send: there always was, the same window again and again
            client = None
        elif client:
            if not client.pump(u, now):
                time.sleep(0.0005)
        else:
            time.sleep(0.01)


class _Client:
    def __init__(self, addr, rate, read, now):
        self.addr, self.rate = addr, rate
        self.read, self.close = getattr(read, "read", read), getattr(read, "close", None)
        self.base = self.next = 0  # what it wants; what we send next
        self.window, self.out, self.first = 32, {}, 0  # the datagrams from base on, kept to send again
        self.end = None  # the end marker's sequence, once the stream ran out
        self.t0, self.sent, self.moved, self.heard = now, 0, now, now
        self.again, self.total = 0, 0
        self.pace, self.eased, self.due, self.last = rate, now, 0.0, now  # the pace now: eased off when datagrams are lost
        self.mark = (0, 0)
        self.back_at, self.back_when, self.last_tx = -1, now, now
        self.said, self.said_at = (0, 0, 0), now  # base, total, again at the last line printed

    def go_back(self, now):
        self.next, self.moved, self.again = self.base, now, self.again + 1

    def feedback(self, expect, window, flags, now):
        self.heard = now
        if expect > self.base:
            for k in range(self.base, expect):
                self.out.pop(k, None)
            self.base, self.moved = expect, now
            if self.next < expect:
                self.next = expect
        self.window = window
        # a gap: again from what it wants, at once (everything sent past the gap is thrown away there: on the console,
        # 2026-11-15, waiting 20 ms here cost ~40 datagrams a gap and a storm of gaps halved the rate). Once per
        # place: the datagrams already on their way make it say "gap" again
        # ...or datagrams are out, it still wants the first of them and nothing was sent after to show it the gap
        # (a live picture's last datagram lost): again without waiting for the 100 ms below
        quiet = self.next > self.base and expect == self.base and now - self.last_tx > 0.015
        if (flags & 1 or quiet) and self.next > self.base and (self.base != self.back_at or now - self.back_when > 0.03):
            self.back_at, self.back_when = self.base, now
            self.go_back(now)

    def packet(self, seq):
        if seq not in self.out:
            if self.end is not None:
                return None
            data = self.read(PAY)
            if data is None:
                return None  # live: nothing to send yet
            if not data:
                self.end = seq
                self.out[seq] = b"PE" + struct.pack("<I", seq)
            else:
                self.out[seq] = b"PS" + struct.pack("<I", seq) + data
        return self.out[seq]

    def pump(self, u, now):
        """Sends what is due; False when there was nothing to do."""
        if self.next > self.base and now - self.moved > 0.1:  # out and unanswered: again from what it wants
            self.go_back(now)
        if now - self.eased > 0.5:  # the pace follows what the last half second cost in datagrams sent twice
            sent, took = self.total - self.mark[0], self.base - self.mark[1]
            if sent > 20:
                waste = 1 - took / sent
                if waste > 0.10:  # never under 2/3 of what was asked: on the console (2026-11-15) a burst of gaps
                    # took a x0.9 pace from 3000 to 1100 KB/s in seconds and the video buffer ran dry before it was back
                    self.pace = max(self.rate * 2 / 3, self.pace * 0.95)
                elif waste < 0.03:
                    self.pace = min(self.rate, self.pace * 1.05)
            self.eased, self.mark = now, (self.total, self.base)
        if now - self.said_at > 5:  # how it goes, every 5 s
            b, t, a = self.said
            print(f"{self.addr[0]}: took {(self.base - b) * PAY / 1024 / (now - self.said_at):.0f} KB/s, "
                  f"{self.total - t} datagrams sent for {self.base - b} taken, again from a gap {self.again - a} times, "
                  f"pace {self.pace / 1024:.0f} KB/s, window {self.window}", flush=True)
            self.said, self.said_at = (self.base, self.total, self.again), now
        did = False
        self.due += (now - self.last) * self.pace
        self.last, due = now, self.due
        if self.sent < due - 3 * PAY:  # behind: no catching up in a burst (the PS2's receive FIFO is 16 KB)
            self.sent = due - 3 * PAY
        while self.sent < due and self.next < self.base + self.window:
            if self.end is not None and self.next > self.end:
                break
            p = self.packet(self.next)
            if p is None:
                break
            try:
                u.sendto(p, self.addr)
            except OSError:
                break
            self.next, self.sent, self.total, did = self.next + 1, self.sent + PAY, self.total + 1, True
            self.last_tx = now
        return did

    def report(self, name):
        if self.close:
            self.close()
        dt = max(time.perf_counter() - self.t0, 1e-3)
        print(f"{self.addr[0]}: {name} asked {self.rate >> 10} KB/s: it took {self.base} datagrams in {dt:.1f} s = "
              f"{self.base * PAY / 1024 / dt:.0f} KB/s; {self.total} sent, sent again from a gap {self.again} times, "
              f"pace at the end {self.pace / 1024:.0f} KB/s", flush=True)
