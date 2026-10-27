# m2ppp

Minimal mgetty-like PPP answer daemon for a serial modem line.

It opens and locks a serial line, initialises the modem, waits for a call,
answers it, waits for `CONNECT`, and either hands the stream to `pppd`
(detected by a PPP LCP frame) or falls back to a login prompt.

## Build

```sh
make
```

## Install

```sh
sudo make install
sudo systemctl daemon-reload
```

`make install` copies the binary to `$(PREFIX)/sbin` (default
`/usr/local/sbin`) and the unit to `/etc/systemd/system`; override with
`PREFIX=`, `SBINDIR=`, `SYSTEMD_DIR=`, or stage with `DESTDIR=`. `make
uninstall` removes both. The manual equivalent is:

```sh
sudo install -m 755 m2ppp /usr/local/sbin/
sudo cp m2ppp@.service /etc/systemd/system/
sudo systemctl daemon-reload
```

## Usage

```text
m2ppp [options] [-- pppd-args...]
```

Arguments after `--` are passed through to `pppd`.

## Options

```text
-d device    serial device                           (default /dev/ttyds01)
-b baud      initial line speed                      (default 38400)
-i string    init chat (without CR)                  (default AT&F14+IF5+IM4;S0=0;E0;V1#CID=9)
-a string    answer chat                             (default ATA)
-t seconds   answer timeout                          (default 30)
-L dir       lock directory                          (default /var/lock)
-p path      pppd binary                             (default /usr/sbin/pppd)
-g path      login program                           (default /bin/login)
-u user      run pppd as user; if it cannot be changed, quit
-w seconds   PPP frame detection window              (default 60)
-P seconds   show login prompt after N sec           (default 6)
-c seconds   pause before waiting again              (default 0)
-I minutes   hang up an idle PPP session after N min (default 10)
-r seconds   modem-reinit/select interval            (default 900; 0 disables)
-o seconds   wait for the modem OK after init        (default 2)
-T string    TERM for login sessions                 (default vt100)
-v           trace every step of the call flow to stdout
-F           do not add /etc/ppp/options.<device> to pppd
-h           show this help and exit
```

## Run as a systemd service

The instance name after `@` is the device name without `/dev/`:

```sh
sudo systemctl enable --now m2ppp@ttyds01.service
journalctl -u m2ppp@ttyds01.service -f
```

Stop and disable:

```sh
sudo systemctl disable --now m2ppp@ttyds01.service
```

## Run manually

```sh
./m2ppp -d /dev/ttyds01 -v
```
