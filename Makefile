CC      ?= cc
CFLAGS  ?= -Wall -Wextra -O2
LDFLAGS ?=
STRIP   ?= -s

# extra strict warnings for `make lint` (GCC). The two suppressed classes are
# the termios `&= ~FLAG` sign conversions and the const->char* argv casts.
LINTFLAGS ?= -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wformat=2 \
	-Wcast-qual -Wwrite-strings -Wstrict-prototypes -Wmissing-prototypes \
	-Wold-style-definition -Wredundant-decls -Wundef -Wpointer-arith \
	-Wcast-align -Wswitch-enum -Wswitch-default -Wfloat-equal \
	-Wnull-dereference -Wdouble-promotion -Wlogical-op -Wduplicated-cond \
	-Wduplicated-branches -Wrestrict -Wnested-externs -Wbad-function-cast \
	-Wno-sign-conversion -Wno-cast-qual

# install locations (override e.g. make install PREFIX=/usr DESTDIR=/pkg)
PREFIX      ?= /usr/local
SBINDIR     ?= $(PREFIX)/sbin
SYSTEMD_DIR ?= /etc/systemd/system
DESTDIR     ?=

all: m2ppp

m2ppp: m2ppp.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS) $(STRIP)

lint:
	clang-format --dry-run --Werror m2ppp.c
	$(CC) $(CFLAGS) -fsyntax-only $(LINTFLAGS) m2ppp.c

install: m2ppp
	install -d $(DESTDIR)$(SBINDIR)
	install -m 755 m2ppp $(DESTDIR)$(SBINDIR)/m2ppp
	install -d $(DESTDIR)$(SYSTEMD_DIR)
	install -m 644 m2ppp@.service $(DESTDIR)$(SYSTEMD_DIR)/m2ppp@.service
	@echo "installed; run 'systemctl daemon-reload' to pick up the unit"

uninstall:
	rm -f $(DESTDIR)$(SBINDIR)/m2ppp
	rm -f $(DESTDIR)$(SYSTEMD_DIR)/m2ppp@.service
	@echo "removed; run 'systemctl daemon-reload' to drop the unit"

clean:
	rm -f m2ppp

.PHONY: all clean lint install uninstall
