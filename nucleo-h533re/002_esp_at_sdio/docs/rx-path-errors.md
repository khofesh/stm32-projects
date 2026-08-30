# rx path errors (fixed)

Original report - AT+HTTPCGET / AT+HTTPCLIENT / AT+CIPSNTPCFG on the console:

```shell
E sdio_transport: Invalid size:6195
E sdio_transport: Invalid size:1048574
E sdio_driver: CMD53 read error, addr 0x1f7b9 count 512, err 0x80000000 TIMEOUT
E sdio_driver: DPSM still active after abort, STA=0x00001004 DCOUNT=224
rx packet error: -1
```

## what each one was

**`Invalid size:6195`** - not invalid at all. 6195 bytes really were queued: the
slave holds several packets and an HTTP GET leaves kilobytes pending, while the
old check treated anything over the 4096-byte read buffer as corrupt, threw the
length away and logged it again every millisecond. A backlog is now truncated to
the buffer and reported with `ERR_NOT_FINISHED`, which `AtDrainRx()` already
loops on.

**`Invalid size:1048574`** - that one is a fault, but the number says which:
0x100000 - 2, i.e. `rx_got_bytes` two bytes _ahead_ of the slave's counter. The
difference of two free-running 20-bit counters wraps rather than going negative,
so a host that has credited itself bytes the slave never sent sees a length just
under RX_BYTE_MAX and never sees it come down. Anything past half the counter
range is now treated as a desync and re-anchored on the slave's own counter.

**`CMD53 read error ... count 512 ... TIMEOUT`** - the packet was being read as a
whole-block transfer followed by a sub-block tail. The slave takes the length it
is to serve from the CMD53 address (0x1f800 - addr) and is finished with the
packet once a read has started on it: 512 bytes of a 583-byte packet leaves the
last 71 still counted by the length register but no longer available, and the
CMD53 that goes looking for them times out. Every response longer than one block
lost its tail this way. A packet now goes out as a single CMD53, its transfer
rounded up to the block the slave pads to.

**`DPSM still active after abort`** - the CCCR I/O abort was sent once, and the
log shows the slave answering it with CTIMEOUT while DPSMACT stayed set. DCTRL
and DLEN are read-only while it is, so every later transfer inherited stale
programming. The abort is now retried up to three times.

Also: the CMD53 timeout budget was 3x the theoretical transfer time, which fails
a 1024-byte read at 1033 ms; it is now 6x plus a 100 ms floor.

## verified on the bench

`AT`, `AT+GMR`, `AT+CIPSTA?`, `AT+CIPSNTPCFG`, `AT+HTTPCGET="https://example.com"`
and `AT+HTTPCLIENT` all return their full responses, page body included, where
the log above lost them. Clean runs (zero errors) are now normal.

## what is still open

- A CMD53 read still fails now and then with nothing received at all
  (`STA=0x00081000`, `DCOUNT` unchanged from the requested length) - at any
  packet size, roughly once or twice per session of HTTP commands. The packet is
  dropped, the counters stay in step and the console carries on. The link runs at
  25 kHz on one data line with `SDIO_PORT_GPIO_SPEED` at LOW, so this is most
  likely the wiring rather than the protocol.
- Large downloads (`AT+HTTPCGET` of the 24 kB hardware.png, 2048-byte chunks)
  still fail repeatedly and trigger the reinit path. Reads are capped at
  `SDIO_HOST_MAX_READ` (2048) because a 4096-byte read of a big backlog is never
  answered.
- `CMD53 attempt N failed, STA=... DCOUNT=...` is logged at debug level on every
  failed attempt; DCOUNT is what separates a slave that stopped part way from one
  that never started.
