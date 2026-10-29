/*
 * m2ppp.c - minimal mgetty-like PPP answer daemon.
 *
 * Attaches a serial line, initialises the modem, waits for RING, answers
 * the call, waits for CONNECT, watches for the start of a PPP LCP frame
 * (PPP frame detection), then runs pppd on a PTY with the environment variables
 * CALLER_ID / CONNECT / DEVICE set.
 *
 * The bytes already consumed while detecting the PPP frame are replayed into
 * pppd's input over the PTY, and the real modem line is then relayed to
 * and from the PTY for the rest of the call, so pppd sees the complete
 * PPP stream (nothing read by the detector is lost).
 *
 * The line is driven through termios: raw mode, speed, hardware flow
 * control (CRTSCTS), carrier handling (CLOCAL) and hangup (HUPCL / B0)
 * are all termios settings.
 *
 * Behaviour is modelled on mgetty 1.2.1 by Gert Doering; all functions
 * here are written from scratch.
 *
 * usage: m2ppp [options] [-- pppd-args...]
 *   -d device    serial device                           (default /dev/ttyds1)
 *   -b baud      initial line speed                      (default 38400)
 *   -i string    init chat (without CR)                  (default below)
 *   -a string    answer chat                             (default ATA)
 *   -t seconds   answer timeout                          (default 30)
 *   -L dir       lock directory                          (default /var/lock)
 *   -p path      pppd binary                             (default /usr/sbin/pppd)
 *   -g path      login program for non-PPP calls         (default /bin/login)
 *   -u user      run pppd as user; if it cannot be changed, quit
 *   -w seconds   PPP frame detection window              (default 60)
 *   -P seconds   show login prompt after N sec           (default 6)
 *   -c seconds   pause before waiting again              (default 0)
 *   -I minutes   hang up a PPP call after N min with no data from the caller
 *                (default 10; pppd's own output does not count)
 *   -r seconds   modem-reinit/select interval            (default 900)
 *   -o seconds   wait for the modem OK after init        (default 2)
 *   -T string    TERM for login sessions                 (default vt100)
 *   -H           pass the caller id to login as its remote host (login -h;
 *                needs /etc/pam.d/remote, login's PAM service with -h)
 *   -v           trace every step of the call flow to stdout
 *   -F           do not add /etc/ppp/options.<device> to pppd
 *   -h           show this help and exit
 */

#define _GNU_SOURCE
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <grp.h>
#include <limits.h>
#include <paths.h>
#include <poll.h>
#include <pwd.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <utmpx.h>

/* ------------------------------------------------------------------ */
/* configuration constants                                            */
/* ------------------------------------------------------------------ */

/* default timeout values */
#define ANSWER_SECONDS_DEFAULT 30
#define INIT_OK_SECONDS_DEFAULT 2
#define LOGIN_PROMPT_SECONDS_DEFAULT 6
#define MODEM_REINIT_SECONDS_DEFAULT 900
#define PPP_DETECT_SECONDS_DEFAULT 60
#define PPP_IDLE_MINUTES_DEFAULT 10

/* default option values */
#define ANSWER_DEFAULT "ATA"
#define BAUD_DEFAULT 38400
#define DEVICE_DEFAULT "/dev/ttyds1"
#define INIT_DEFAULT "AT&F14+IF5+IM4;S0=0;E0;V1#CID=9"
#define LOCKDIR_DEFAULT "/var/lock"
#define LOGIN_DEFAULT "/bin/login"
#define PPPD_DEFAULT "/usr/sbin/pppd"
#define TERM_DEFAULT "vt100"

/* max extra pppd arguments accepted after "--", not counting "modem" */
#define MAX_PPPD_ARGS 16

#define CHAT_DELAY_MS 500
#define CHILD_TERM_MS 5000
#define CHILD_POLL_MS 100
#define LOGIN_GAP_MS 5000
#define LOGIN_DRAIN_MS 1000
#define CONNECT_CAPTURE_MS 2000
#define POLL_SLICE_MS 1500
/* longest wait for pppd to set up (and flush) its tty before the replay */
#define REPLAY_WAIT_MS 2000
/* longest wait for room to write (e.g. CTS held low by the modem) */
#define WRITE_TIMEOUT_MS 10000
/* bytes written per write() once poll() reports room; a tty with POLLOUT has
 * fewer than 256 bytes queued, so this never blocks */
#define WRITE_CHUNK 256
/* a lock file without a valid pid is stale once it is this old (younger ones
 * may still be being written by another program) */
#define LOCK_EMPTY_STALE_SECONDS 5
#define ISSUE_FILE "/etc/issue.net"
/* PAM service file util-linux login uses with -h (-H) */
#define PAM_REMOTE_FILE "/etc/pam.d/remote"

#define DIGIT_CHARS "0123456789"
/* caller/called numbers may carry *#+/; the CONNECT speed does not */
#define PHONE_CHARS "0123456789+#*/"

/*
 * UUCP lock file(s) to use, chosen at compile time:
 *   LOCK_DEVNUM   - keyed on the device number (alias-safe)
 *   LOCK_BASENAME - keyed on the path basename (UUCP/pppd/mgetty convention)
 *   LOCK_BOTH     - create both
 */
#define LOCK_DEVNUM 1
#define LOCK_BASENAME 2
#define LOCK_BOTH (LOCK_DEVNUM | LOCK_BASENAME)
#ifndef LOCK_KIND /* override at compile time: -DLOCK_KIND=LOCK_DEVNUM|LOCK_BASENAME */
#define LOCK_KIND LOCK_BOTH
#endif

#define MAX_LOCK_FILES 2

/* PPP field constants (RFC 1662 / mgetty.h) */
#define PPP_FLAG 0x7e
#define PPP_ADDRESS 0xff
#define PPP_ESC 0x7d
#define PPP_CTRL 0x03
#define PPP_LCP_PROTO_HI 0xc0
#define PPP_LCP_PROTO_LOW 0x21
#define PPP_UNESCAPE(c) ((unsigned char)((c) ^ 0x20))

/* serial_read_char() results */
#define SERIAL_TIMEOUT 0
#define SERIAL_OK 1
#define SERIAL_ERROR (-1)
#define SERIAL_LOST (-2) /* device gone / line hung up */

/* chat_expect() results */
#define CHAT_MATCH 0
#define CHAT_ERROR (-1)
#define CHAT_ABORT (-2)
#define CHAT_LOST (-3) /* line lost while waiting */

/* wait_ring_or_ok() results (SERIAL_LOST on device loss) */
#define WAIT_OK 0
#define WAIT_RING 1
#define WAIT_ABORT 2
#define WAIT_TIMEOUT (-1)

/* detect_type() results (SERIAL_LOST on device loss) */
#define DETECTED_DROP 0    /* timeout/abort: drop the call        */
#define DETECTED_PPP 1     /* PPP frame seen: run pppd            */
#define DETECTED_LOGIN 2   /* login name entered: run the login   */
#define DETECTED_LOST (-1) /* line lost: reopen                 */

/* chat_capture_connect() results */
#define CAPTURE_OK 0
#define CAPTURE_LOST (-1) /* line lost while scanning             */
#define CAPTURE_FULL (-2) /* PPP replay buffer filled: drop the call */

/* termios flow control */
#define FLOW_NONE 0
#define FLOW_SOFT 1
#define FLOW_HARD 2

/* some termios flags are not present on every platform */
#ifndef CRTSCTS
#define CRTSCTS 0
#endif

/* ------------------------------------------------------------------ */
/* global call data / options                                         */
/* ------------------------------------------------------------------ */

/* set by the signal handler (async-signal-safe), acted on by check_signal() */
static volatile sig_atomic_t g_signal;

static pid_t main_pid; /* the daemon itself, not a forked child */

static const char *opt_device = DEVICE_DEFAULT;   /* -d: serial device */
static const char *opt_lockdir = LOCKDIR_DEFAULT; /* -L: UUCP lock directory */
static const char *opt_pppd = PPPD_DEFAULT;       /* -p: pppd program */
static const char *opt_login = LOGIN_DEFAULT;     /* -g: login program for non-PPP calls */
static const char *opt_runas;                     /* -u user: run pppd as this user, else keep */
static const char *opt_init = INIT_DEFAULT;       /* -i: modem init chat (sent without CR) */
static const char *opt_answer = ANSWER_DEFAULT;   /* -a: answer chat */
static const char *opt_term = TERM_DEFAULT;       /* -T: TERM for login sessions */
static int opt_verbose;                           /* -v: trace each step to stdout */
static int opt_baud = BAUD_DEFAULT;               /* -b: initial line speed */
static int opt_answer_seconds = ANSWER_SECONDS_DEFAULT;         /* -t: answer timeout (sec) */
static int opt_ppp_detect_seconds = PPP_DETECT_SECONDS_DEFAULT; /* -w: PPP detect window (sec) */
static int opt_prompt_seconds = LOGIN_PROMPT_SECONDS_DEFAULT;   /* -P: prompt after (sec) */
static int opt_cooldown_seconds;                                /* -c: pause before waiting (sec) */
static int opt_ppp_idle_minutes = PPP_IDLE_MINUTES_DEFAULT;     /* -I: PPP idle timeout (min) */
static int opt_reinit_seconds = MODEM_REINIT_SECONDS_DEFAULT; /* -r: modem reinit interval (sec) */
static int opt_init_ok_seconds = INIT_OK_SECONDS_DEFAULT;     /* -o: init OK timeout (sec) */
static int opt_no_optsfile;                                   /* -F: no /etc/ppp/options.<dev> */
static int opt_remote_host;                                   /* -H: caller id as login -h host */

/* "modem" plus up to MAX_PPPD_ARGS extra pppd args */
static char *pppd_args[MAX_PPPD_ARGS + 1];
static int pppd_argc;

static char *connect;
static char *caller_id;
static char *called_nr;

static int serial_fd = -1;
static pid_t child_pid = -1;
static const char *child_utmp_line; /* login child's utmp line, ended if we exit */
static char lock_files[MAX_LOCK_FILES][256];
static int lock_count;

/*
 * Bytes read while watching for the PPP frame. These belong to
 * the caller's PPP stream, so they are replayed to pppd over the PTY
 * before the relay takes over (otherwise pppd would lose the first
 * LCP frame).
 */
static unsigned char ppp_buf[8192];
static size_t ppp_len;

/* caller-id line accumulator */
static char cnd_line[512];
static size_t cnd_len;

/*
 * Strings that abort a chat even if the expected answer has not shown up.
 * mgetty calls these "action strings" (ring_chat_actions in mgetty.c); a
 * modem that answers ATA with "NO CARRIER" must not make us wait for
 * CONNECT until the timeout.
 */
static const char *const answer_aborts[] = {"NO CARRIER", "BUSY", "ERROR", NULL};

/* PPP frame detection state machine (see mgetty's logname.c) */
static int ppp_level;
static int ppp_escaped;

/*
 * Numeric baud rate -> termios speed. The high rates are non-POSIX
 * extensions, so they are guarded with #ifdef (as in mgetty's speedtab).
 */
static const struct {
  int rate;
  speed_t speed;
} baudtab[] = {
    {300, B300},       {1200, B1200},   {2400, B2400},   {4800, B4800},
    {9600, B9600},     {19200, B19200}, {38400, B38400},
#ifdef B57600
    {57600, B57600},
#endif
#ifdef B115200
    {115200, B115200},
#endif
};

/* wtmp file the utmp records are appended to (the unit test redirects it) */
static const char *wtmp_path = _PATH_WTMP;

/* "YYYY-MM-DD HH:MM:SS " prefix for log lines */
static void print_timestamp(FILE *fp)
{
  struct tm *tm;
  time_t cur;

  cur = time(NULL);
  tm = localtime(&cur);

  fprintf(fp, "%04d-%02d-%02d %02d:%02d:%02d ", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
          tm->tm_hour, tm->tm_min, tm->tm_sec);
}

/* current monotonic time in milliseconds */
static long long now(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void shutdown_call(void);

/* print "TIMESTAMP m2ppp: <msg>" to stderr and exit (fatal errors); in the
 * daemon also hang up and clean up as on SIGTERM, since this can happen mid-call */
static void die(const char *fmt, ...)
{
  va_list ap;

  print_timestamp(stderr);
  fputs("m2ppp: ", stderr);
  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);
  fputc('\n', stderr);
  if (main_pid && getpid() == main_pid) shutdown_call();
  _exit(1);
}

/* trace one step of the flow to stdout (only with -v) */
static void step(const char *fmt, ...)
{
  va_list ap;

  if (!opt_verbose) return;
  print_timestamp(stdout);
  fputs("m2ppp: ", stdout);
  va_start(ap, fmt);
  vfprintf(stdout, fmt, ap);
  va_end(ap);
  fputc('\n', stdout);
  fflush(stdout);
}

/* set *var to a copy of val when it differs; returns 1 if it changed */
static int cnd_assign(char **var, const char *val)
{
  char *p;
  size_t need;

  if (*var && strcmp(*var, val) == 0) return 0;

  need = strlen(val) + 1;

  if (*var == NULL) { /* first use */
    *var = strdup(val);
    if (!*var) die("out of memory");
    return 1;
  }

  if (need > strlen(*var) + 1) { /* grow only when it does not fit */
    p = realloc(*var, need);
    if (!p) die("out of memory");
    *var = p;
  }
  memcpy(*var, val, need); /* otherwise reuse the existing buffer */
  return 1;
}

static void reset_call_data(void)
{
  cnd_assign(&connect, "");
  cnd_assign(&caller_id, "none");
  cnd_assign(&called_nr, "");
}

/* ------------------------------------------------------------------ */
/* UUCP-style lock files                                              */
/* ------------------------------------------------------------------ */

/*
 * Fill names[] with the UUCP lock file(s) for dev and return the count.
 * The device-number lock (LOCK_DEVNUM) is alias-safe; the basename lock
 * (LOCK_BASENAME) follows the UUCP/pppd/mgetty convention. If neither is
 * available (e.g. LOCK_DEVNUM on a non-character device) the basename is used.
 */
static int lock_names(const char *dev, char names[][256], int max)
{
  const char *b;
  int n = 0;
  struct stat st;

  if ((LOCK_KIND & LOCK_DEVNUM) && n < max && stat(dev, &st) == 0 && S_ISCHR(st.st_mode)) {
    snprintf(names[n], 256, "%s/LCK..%03u%03u", opt_lockdir, major(st.st_rdev), minor(st.st_rdev));
    n++;
  }
  if (((LOCK_KIND & LOCK_BASENAME) || n == 0) && n < max) {
    b = strrchr(dev, '/');
    b = b ? b + 1 : dev;
    snprintf(names[n], 256, "%s/LCK..%s", opt_lockdir, b);
    n++;
  }
  return n;
}

static int lock_read_pid(const char *name)
{
  char buf[32];
  int fd, pid = 0;
  ssize_t n;

  fd = open(name, O_RDONLY);
  if (fd < 0) return 0;
  n = read(fd, buf, sizeof(buf) - 1);
  close(fd);
  if (n <= 0) return 0;
  buf[n] = '\0';
  if (sscanf(buf, "%d", &pid) != 1) return 0;
  return pid;
}

/* return pid of a holder of any lock file, -1 if a lock has no valid pid yet,
 * or 0 if free (stale ones removed) */
static int lock_check(const char *dev)
{
  char names[MAX_LOCK_FILES][256];
  int i, n, pid;
  struct stat st;

  n = lock_names(dev, names, MAX_LOCK_FILES);
  for (i = 0; i < n; i++) {
    if (stat(names[i], &st) < 0) {
      if (errno == ENOENT) continue;
      st.st_mtime = time(NULL); /* age unknown: an empty one counts as busy */
    }

    pid = lock_read_pid(names[i]);
    if (pid <= 0) {
      /* empty or garbage (e.g. a crash between create and write): stale
       * once old enough, else assume its creator is still writing it */
      if (time(NULL) - st.st_mtime < LOCK_EMPTY_STALE_SECONDS) return -1;
      unlink(names[i]);
      continue;
    }
    if (pid == getpid()) continue;
    if (kill(pid, 0) < 0 && errno == ESRCH) {
      unlink(names[i]);
      continue;
    }
    return pid;
  }
  return 0;
}

static void lock_remove(void)
{
  int i;

  if (!lock_count) return;
  for (i = 0; i < lock_count; i++) unlink(lock_files[i]);
  lock_count = 0;
}

/* write our pid to a temporary file and link() it into place, so a lock file
 * is never seen without its pid */
static int lock_make(const char *dev)
{
  char buf[32], tmp[sizeof(lock_files[0]) + 16];
  int fd, i, n, rc;

  n = lock_names(dev, lock_files, MAX_LOCK_FILES);
  snprintf(buf, sizeof(buf), "%10d\n", (int)getpid());

  lock_count = 0;
  for (i = 0; i < n; i++) {
    rc = snprintf(tmp, sizeof(tmp), "%s.%d", lock_files[i], (int)getpid());
    fd = rc > 0 && (size_t)rc < sizeof(tmp)
             ? open(tmp, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644)
             : -1;
    if (fd < 0) {
      lock_remove();
      return -1;
    }
    rc = write(fd, buf, strlen(buf)) == (ssize_t)strlen(buf) ? 0 : -1;
    if (close(fd) < 0) rc = -1;
    if (rc == 0) rc = link(tmp, lock_files[i]);
    unlink(tmp);
    if (rc < 0) {
      lock_remove();
      return -1;
    }
    lock_count++; /* this file exists now and must be cleaned up */
  }
  return 0;
}

/* take the UUCP lock, for the life of the process. 0 on success. */
static int lock_line(void)
{
  step("locking %s", opt_device);
  if (lock_check(opt_device) != 0 || lock_make(opt_device) != 0) {
    step("line is locked, giving up");
    return -1;
  }
  step("lock acquired");
  return 0;
}

/*
 * mgetty-style state machine for handling one call.
 * The line is locked and opened once at startup (main, via lock_line()/
 * open_line()); the state machine then brings the modem up (st_init) and
 * cycles through the call states, with pppd started from st_exec_ppp. When
 * the device disappears, st_open/st_init re-run until it comes back, so the
 * daemon recovers without a process restart.
 */
enum m2ppp_state {
  st_unknown = 0,
  st_open,         /* open + configure the line                 */
  st_init,         /* send the modem init                       */
  st_await_ok,     /* wait for the modem's OK                   */
  st_waiting,      /* wait for RING / periodic re-init          */
  st_answer_phone, /* ATA, wait for CONNECT                     */
  st_detect_ppp,   /* watch for the start of a PPP frame        */
  st_exec_login,   /* run the login program                     */
  st_exec_ppp      /* hand the line to pppd                     */
};

/* ------------------------------------------------------------------ */
/* termios line control (mgetty "tio" style)                          */
/* ------------------------------------------------------------------ */

static speed_t baud_to_speed(int baud)
{
  size_t i;

  if (baud == 0) return B0; /* hang up */
  for (i = 0; i < sizeof(baudtab) / sizeof(baudtab[0]); i++)
    if (baudtab[i].rate == baud) return baudtab[i].speed;

  die("unsupported baud rate %d", baud);
  return B0; /* not reached: die() does not return */
}

/* is baud one of the rates baud_to_speed() actually supports? */
static int baud_supported(int baud)
{
  size_t i;

  for (i = 0; i < sizeof(baudtab) / sizeof(baudtab[0]); i++)
    if (baudtab[i].rate == baud) return 1;
  return 0;
}

static int tio_get(int fd, struct termios *t)
{
  return tcgetattr(fd, t);
}

static int tio_set(int fd, const struct termios *t)
{
  return tcsetattr(fd, TCSANOW, t);
}

static void tio_set_speed(struct termios *t, int baud)
{
  cfsetispeed(t, baud_to_speed(baud));
  cfsetospeed(t, baud_to_speed(baud));
}

/* raw mode: no echo, no editing, no signal generation, no CR/NL munging */
static void tio_mode_raw(struct termios *t)
{
  t->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
  t->c_oflag &= ~OPOST;
  t->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  t->c_cflag &= ~(CSIZE | PARENB | CSTOPB);
  t->c_cflag |= (CS8 | CREAD);
  t->c_cc[VMIN] = 1;
  t->c_cc[VTIME] = 0;
}

/* carrier sensitivity: CLOCAL set means "ignore DCD" (getty default) */
static void tio_carrier(struct termios *t, int sensitive)
{
  if (sensitive) t->c_cflag &= ~CLOCAL;
  else t->c_cflag |= CLOCAL;
}

/*
 * flow control:
 *   FLOW_HARD -> CRTSCTS   (kernel drives RTS/CTS)
 *   FLOW_SOFT -> XON/XOFF
 *   FLOW_NONE -> nothing
 */
static void tio_set_flow_control(struct termios *t, int type)
{
  t->c_iflag &= ~(IXON | IXOFF | IXANY);
  t->c_cflag &= ~CRTSCTS;

  switch (type) {
    case FLOW_HARD:
      t->c_cflag |= CRTSCTS;
      break;
    case FLOW_SOFT:
      t->c_iflag |= (IXON | IXOFF);
      break;
    default:
      break;
  }
}

/* normal getty/login-mode termios: CR/NL mapping, echo, editing, signals */
static void tio_mode_sane_flags(struct termios *t)
{
  t->c_iflag |= ICRNL;
  t->c_iflag &= ~(IGNCR | INLCR | ISTRIP);
  t->c_oflag |= (OPOST | ONLCR);
  t->c_lflag |= (ISIG | ICANON | ECHO | ECHOE | ECHOK);
  t->c_cflag |= (CS8 | CREAD);

  t->c_cc[VINTR] = 0x03;  /* ^C */
  t->c_cc[VQUIT] = 0x1c;  /* ^\ */
  t->c_cc[VERASE] = 0x7f; /* DEL */
  t->c_cc[VKILL] = 0x15;  /* ^U */
  t->c_cc[VEOF] = 0x04;   /* ^D */
  t->c_cc[VSTART] = 0x11; /* ^Q */
  t->c_cc[VSTOP] = 0x13;  /* ^S */
  t->c_cc[VSUSP] = 0x1a;  /* ^Z */
  t->c_cc[VMIN] = 1;
  t->c_cc[VTIME] = 0;
}

/* set the real line to the sane login mode (carrier-sensitive) */
static void tio_mode_sane(int fd)
{
  struct termios t;

  if (tio_get(fd, &t) < 0) return;
  tio_mode_sane_flags(&t);
  tio_carrier(&t, 1);                  /* carrier-sensitive: a DCD drop hangs up the session */
  tio_set_flow_control(&t, FLOW_SOFT); /* software XON/XOFF; drops the stale CRTSCTS */

  tio_set(fd, &t);
}

/* HUPCL drops DTR on close, the termios way to hang the line up */
static void tio_hangup_on_close(struct termios *t, int on)
{
  if (on) t->c_cflag |= HUPCL;
  else t->c_cflag &= ~HUPCL;
}

/*
 * Hang up the line: setting the speed to B0 drops DTR on a real serial
 * port, so no TIOCM* ioctl is needed for this. CLOCAL goes on first: the
 * modem drops DCD in answer, and on the carrier-sensitive call line the
 * driver would hang up our own fd.
 */
static void tio_hangup(int fd)
{
  struct termios t;

  if (tio_get(fd, &t) < 0) return;
  tio_carrier(&t, 0);
  tio_hangup_on_close(&t, 1);
  tio_set_speed(&t, 0); /* B0 == hang up */
  tio_set(fd, &t);
}

/* re-assert DTR by restoring a real line speed, and the modem's RTS/CTS
 * (a login session leaves the line on FLOW_SOFT) */
static void tio_raise(int fd, int baud)
{
  struct termios t;

  if (tio_get(fd, &t) < 0) return;
  tio_mode_raw(&t);
  tio_set_flow_control(&t, FLOW_HARD);
  tio_set_speed(&t, baud);
  tio_carrier(&t, 0);
  tio_hangup_on_close(&t, 1);
  tio_set(fd, &t);
}

static int serial_open(const char *dev)
{
  int fd, fl, saved;

  fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) return -1;

  /* one process per device; flock also catches alternate device paths */
  if (flock(fd, LOCK_EX | LOCK_NB) < 0) {
    saved = errno;
    close(fd);
    errno = saved;
    return -1;
  }

  fl = fcntl(fd, F_GETFL, 0);
  if (fl < 0 || fcntl(fd, F_SETFL, fl & ~O_NONBLOCK) < 0) {
    saved = errno;
    close(fd);
    errno = saved;
    return -1;
  }
  return fd;
}

static int serial_config(int fd, int baud)
{
  struct termios t;

  if (tio_get(fd, &t) < 0) return -1;

  tio_mode_raw(&t);
  tio_set_speed(&t, baud);
  tio_set_flow_control(&t, FLOW_HARD);
  tio_carrier(&t, 0);         /* CLOCAL */
  tio_hangup_on_close(&t, 1); /* HUPCL  */

  if (tio_set(fd, &t) < 0) return -1;
  return 0;
}

/* line mode for a connected call: RTS/CTS, and carrier-sensitive, so a DCD
 * drop makes the driver hang the tty up and the relay ends (read() fails)
 * instead of pppd running on a dead call. hangup_line()/open_line() set
 * CLOCAL again for the idle line. */
static void serial_set_call_mode(int fd)
{
  struct termios t;

  if (tio_get(fd, &t) < 0) return;
  tio_set_flow_control(&t, FLOW_HARD);
  tio_carrier(&t, 1);
  tio_set(fd, &t);
}

static void serial_flush_input(int fd)
{
  tcflush(fd, TCIFLUSH);
}

/* ------------------------------------------------------------------ */
/* signals                                                            */
/* ------------------------------------------------------------------ */

/*
 * Signal handling. The handler only records the signal (async-signal-safe);
 * the real work (log, hang up, unlink the lock, exit) is done by
 * check_signal(), which runs in normal context from the blocking loops.
 */
static void sig_handler(int sig)
{
  g_signal = sig;
}

/* signal the child's process group (it is a session leader after setsid()),
 * so helpers it started in its own group go too; just the child if the
 * group does not exist yet (signalled before its setsid()) */
static void kill_child(pid_t pid, int sig)
{
  if (kill(-pid, sig) < 0) kill(pid, sig);
}

/* terminate and reap the active child before exiting; *status gets its
 * wait status (0 if there was none) */
static void terminate_child(int *status)
{
  long long deadline;
  pid_t r;

  *status = 0;
  if (child_pid <= 0) return;

  kill_child(child_pid, SIGTERM);
  deadline = now() + CHILD_TERM_MS;
  for (;;) {
    r = waitpid(child_pid, status, WNOHANG);
    if (r == child_pid || (r < 0 && errno != EINTR)) break;
    if (now() >= deadline) {
      kill_child(child_pid, SIGKILL);
      while (waitpid(child_pid, status, 0) < 0 && errno == EINTR) {
      }
      break;
    }
    usleep(100000);
  }
  child_pid = -1;
}

static void end_utmp(const char *line, pid_t pid, int status);

/* install a handler that interrupts blocking calls (no SA_RESTART) */
static void install_signal(int sig)
{
  struct sigaction sa;

  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = sig_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(sig, &sa, NULL);
}

/* before exiting on a signal or die(): hang up, end the child and its utmp
 * entry, drop the lock */
static void shutdown_call(void)
{
  pid_t pid = child_pid;
  int status;

  if (serial_fd >= 0) {
    tcflush(serial_fd, TCOFLUSH); /* or the exit's close() waits for CTS */
    tio_hangup(serial_fd);
  }
  terminate_child(&status);
  /* a login session we end must leave who/last too */
  if (child_utmp_line && pid > 0) end_utmp(child_utmp_line, pid, status);
  lock_remove();
}

static void check_signal(void)
{
  const char *name = "signal";
  int sig = g_signal;

  if (!sig) return;
  g_signal = 0;

  if (sig == SIGINT) name = "SIGINT (^C)";
  else if (sig == SIGTERM) name = "SIGTERM";
  else if (sig == SIGHUP) name = "SIGHUP";

  step("caught %s, hanging up", name);
  shutdown_call();
  _exit(1);
}

/*
 * Waits that a signal cannot slip past: one that lands after the last
 * check_signal() but before a blocking sleep()/waitpid() would otherwise
 * wait out the whole call (a long cooldown, or a login session).
 */

/* sleep ms in POLL_SLICE_MS slices, checking for a signal before each */
static void nap(long long ms)
{
  long long end = now() + ms, left;

  for (;;) {
    check_signal();
    left = end - now();
    if (left <= 0) return;
    poll(NULL, 0, left > POLL_SLICE_MS ? POLL_SLICE_MS : (int)left);
  }
}

/* wait up to timeout_ms (< 0: forever) for child pid to exit, checking for
 * a signal every CHILD_POLL_MS. 0 once reaped, -1 on timeout. */
static int wait_child(pid_t pid, int *status, long long timeout_ms)
{
  long long end = now() + timeout_ms;
  pid_t r;

  *status = 0;
  for (;;) {
    check_signal();
    r = waitpid(pid, status, WNOHANG);
    if (r == pid || (r < 0 && errno != EINTR)) return 0;
    if (timeout_ms >= 0 && now() >= end) return -1;
    poll(NULL, 0, CHILD_POLL_MS);
  }
}

/* errno values that mean the device/line has gone away */
static int serial_gone(int err)
{
  return err == EIO || err == ENXIO || err == ENODEV || err == ECONNRESET || err == EBADF;
}

/*
 * read one byte with timeout.
 *   returns  SERIAL_OK      on success (byte in *c)
 *            SERIAL_TIMEOUT on timeout
 *            SERIAL_ERROR   on a recoverable error
 *            SERIAL_LOST    on EOF / hung-up device
 */
static int serial_read_char(int fd, int timeout_ms, unsigned char *c)
{
  struct pollfd pfd = {.fd = fd, .events = POLLIN};
  long long end = now() + timeout_ms, left;
  int r, wait;

  for (;;) {
    check_signal();
    left = end - now();
    wait = left > POLL_SLICE_MS ? POLL_SLICE_MS : (left > 0 ? (int)left : 0);

    r = poll(&pfd, 1, wait);
    if (r < 0) {
      if (errno == EINTR) {
        check_signal();
        continue;
      }
      return SERIAL_ERROR;
    }
    if (r == 0) return SERIAL_TIMEOUT;
    if (pfd.revents & POLLIN) {
      /* the fd is readable: drain one byte */
      r = (int)read(fd, c, 1);
      if (r == 1) return SERIAL_OK;
      if (r == 0) return SERIAL_LOST; /* EOF */
      if (errno == EINTR) {
        check_signal();
        continue;
      }
      if (serial_gone(errno)) return SERIAL_LOST;
      return SERIAL_ERROR;
    }
    if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) return SERIAL_LOST;
    /* spurious wakeup: recompute the remaining time and re-poll */
  }
}

/* ------------------------------------------------------------------ */
/* caller-id / CONNECT parsing (subset of mgetty cnd.c)               */
/* ------------------------------------------------------------------ */

static void log_number(const char *label, const char *val)
{
  print_timestamp(stdout);
  fprintf(stdout, "m2ppp: %s: %s\n", label, val);
  fflush(stdout);
}

static void cnd_set(char **var, const char *val)
{
  if (!cnd_assign(var, val)) return;

  if (var == &caller_id) log_number("caller id", *var);
  else if (var == &called_nr) log_number("called number", *var);
}

/* store the run of "accept" characters starting at p into *var */
static void cnd_set_field(char **var, const char *p, const char *accept)
{
  char num[128];
  size_t n = strspn(p, accept);

  if (!n) return;
  if (n >= sizeof(num)) n = sizeof(num) - 1;
  memcpy(num, p, n);
  num[n] = '\0';
  cnd_set(var, num);
}

/*
 * Parse the "CID: <number>" (caller) and "DAD: <number>" (dialed
 * number) fields that some modems append to RING and CONNECT result
 * lines, e.g. "RING CID: 2041 DAD: 5551234".
 */
static void parse_cid_dad(const char *line)
{
  static const struct {
    const char *tag;
    char **var;
  } fields[] = {
      {"CID:", &caller_id},
      {"DAD:", &called_nr},
  };
  const char *p;
  size_t f;

  for (f = 0; f < sizeof(fields) / sizeof(fields[0]); f++) {
    p = strcasestr(line, fields[f].tag);

    if (!p) continue;
    p += strlen(fields[f].tag);
    while (*p && isspace((unsigned char)*p)) p++;
    cnd_set_field(fields[f].var, p, PHONE_CHARS);
  }
}

static void cnd_process(const char *line)
{
  char buf[512], *semi, str[512];
  const char *p;
  size_t i;

  while (*line && isspace((unsigned char)*line)) line++;
  snprintf(str, sizeof(str), "%s", line);
  i = strlen(str);
  while (i > 0 && isspace((unsigned char)str[i - 1])) str[--i] = '\0';
  if (!str[0]) return;

  /* CID:/DAD: fields may appear in RING and CONNECT result lines */
  parse_cid_dad(str);

  /* ELINK: bare phone number */
  if (isdigit((unsigned char)str[0])) {
    cnd_set_field(&caller_id, str, PHONE_CHARS);
    return;
  }

  /* RING;<caller>[;<called>] */
  if (strncmp(str, "RING;", 5) == 0) {
    snprintf(buf, sizeof(buf), "%s", str + 5);
    if ((semi = strchr(buf, ';')) != NULL) {
      *semi++ = '\0';
      while (*semi && !strchr(PHONE_CHARS, *semi)) semi++;
      cnd_set_field(&called_nr, semi, PHONE_CHARS);
    }
    cnd_set_field(&caller_id, buf, PHONE_CHARS);
    return;
  }

  if (strncmp(str, "CONNECT", 7) == 0) {
    p = str + 7;
    while (*p == ' ' || *p == '\t' || *p == ':') p++;
    cnd_set_field(&connect, p, DIGIT_CHARS);
    return;
  }

  if (strncmp(str, "CALLER NUMBER: ", 15) == 0) {
    cnd_set_field(&caller_id, str + 15, PHONE_CHARS);
    return;
  }
}

static void cnd_feed(unsigned char c)
{
  if (c == '\r' || c == '\n') {
    if (cnd_len) {
      cnd_line[cnd_len] = '\0';
      cnd_process(cnd_line);
      cnd_len = 0;
    }
    return;
  }
  if (cnd_len < sizeof(cnd_line) - 1) cnd_line[cnd_len++] = (char)c;
}

/* ------------------------------------------------------------------ */
/* chat: send / expect                                                */
/* ------------------------------------------------------------------ */

/*
 * Write all of buf, but never block for good: with RTS/CTS and CTS held low
 * the tty stops sending and a plain write() would hang once its buffer fills.
 * Wait for room with poll() (at most WRITE_TIMEOUT_MS without progress) and
 * write small chunks that fit; a timeout fails like a write error. The fds
 * stay blocking, since login shares the line's open file.
 */
static int write_full(int fd, const void *buf, size_t len)
{
  const char *p = buf;
  struct pollfd pfd;
  long long deadline = now() + WRITE_TIMEOUT_MS, left;
  ssize_t n;
  int r;

  while (len) {
    left = deadline - now();
    if (left <= 0) {
      step("write timed out after %dms (flow control stuck?)", WRITE_TIMEOUT_MS);
      errno = ETIMEDOUT;
      return -1;
    }
    pfd.fd = fd;
    pfd.events = POLLOUT;
    r = poll(&pfd, 1, left > POLL_SLICE_MS ? POLL_SLICE_MS : (int)left);
    if (r < 0 && errno == EINTR) check_signal();
    if (r <= 0) continue;
    if (pfd.revents & (POLLERR | POLLNVAL)) return -1;
    n = write(fd, p, len > WRITE_CHUNK ? WRITE_CHUNK : len);
    if (n < 0) {
      if (errno == EINTR) {
        check_signal();
        continue;
      }
      return -1;
    }
    if (n == 0) return -1; /* no progress: do not spin forever */
    p += n;
    len -= (size_t)n;
    deadline = now() + WRITE_TIMEOUT_MS; /* progress: restart the timeout */
  }
  return 0;
}

static void chat_send(int fd, const char *p)
{
  char buf[128];
  int no_cr = 0;
  size_t n = 0;

  while (*p) {
    if (*p == '\\') {
      p++;
      if (*p == '\0') break;
      if (*p == 'd') {
        if (n && write_full(fd, buf, n) < 0) return;
        n = 0;
        nap(CHAT_DELAY_MS);
        p++;
        continue;
      }
      if (*p == 'c') {
        no_cr = 1;
        p++;
        continue;
      }
    }

    if (n == sizeof(buf)) {
      if (write_full(fd, buf, n) < 0) return;
      n = 0;
    }
    buf[n++] = *p++;
  }

  if (!no_cr) {
    if (n == sizeof(buf)) {
      if (write_full(fd, buf, n) < 0) return;
      n = 0;
    }
    buf[n++] = '\r';
  }
  if (n) write_full(fd, buf, n);
}

/* does buf[0..len) end with a modem "action" string (ERROR, NO CARRIER, ...)? */
static int is_abort_line(const char *buf, size_t len)
{
  int i;
  size_t n;

  for (i = 0; answer_aborts[i] != NULL; i++) {
    n = strlen(answer_aborts[i]);
    if (len >= n && memcmp(buf + len - n, answer_aborts[i], n) == 0) return 1;
  }
  return 0;
}

/* returns CHAT_MATCH, CHAT_ERROR, CHAT_ABORT or CHAT_LOST */
static int chat_expect(int fd, const char *expect, int timeout_sec)
{
  char match[512];
  int r;
  long long deadline = now() + (long long)timeout_sec * 1000;
  size_t expect_len = strlen(expect), match_len = 0;
  unsigned char c;

  for (;;) {
    r = serial_read_char(fd, 1000, &c);

    if (r == SERIAL_LOST) return CHAT_LOST;
    if (r == SERIAL_ERROR) return CHAT_ERROR;
    if (r == SERIAL_TIMEOUT) {
      if (now() >= deadline) return CHAT_ERROR;
      continue;
    }

    cnd_feed(c);

    if (match_len < sizeof(match) - 1) match[match_len++] = (char)c;
    else {
      memmove(match, match + 1, match_len - 1);
      match[match_len - 1] = (char)c;
    }

    if (match_len >= expect_len && memcmp(match + match_len - expect_len, expect, expect_len) == 0)
      return CHAT_MATCH;

    /* a modem action string means the command failed */
    if (is_abort_line(match, match_len)) {
      step("modem abort string");
      return CHAT_ABORT;
    }
  }
}

/*
 * Parse one CONNECT result line. Accepts both
 *   "CONNECT 64000"                 (speed on the same line)
 *   "64000"                         (speed on its own, split by the modem)
 * Other lines are ignored.
 */
static void parse_connect_line(const char *line)
{
  const char *p;

  while (*line && isspace((unsigned char)*line)) line++;
  if (!*line) return;

  parse_cid_dad(line);

  /*
   * "CONNECT <speed>" or, if the modem split the result, just "<speed>"
   * (possibly followed by CID:/DAD: fields, e.g. " 64000 CID: 202 ...").
   */
  p = line;
  if (strncmp(p, "CONNECT", 7) == 0) p += 7;
  while (*p == ' ' || *p == '\t' || *p == ':') p++;
  cnd_set_field(&connect, p, DIGIT_CHARS);
}

/*
 * Read the remainder of the CONNECT result. Some modems deliver the
 * speed on the same line, others on a following line, so keep scanning
 * (briefly) until a numeric speed shows up. Any byte consumed here is
 * buffered in ppp_buf so it is not lost when the stream turns into PPP.
 *
 * returns CAPTURE_OK normally, CAPTURE_LOST if the line was lost while
 * scanning, or CAPTURE_FULL if the replay buffer filled.
 */
static int chat_capture_connect(int fd)
{
  char line[128];
  int binary = 0, r;
  long long deadline = now() + CONNECT_CAPTURE_MS;
  size_t line_len = 0;
  unsigned char c;

  ppp_len = 0;
  cnd_len = 0; /* drop any partial line left over from chat_expect() */

  for (;;) {
    r = serial_read_char(fd, 300, &c);

    if (r == SERIAL_LOST) return CAPTURE_LOST;
    if (r == SERIAL_ERROR) break;
    if (r == SERIAL_TIMEOUT) {
      if (connect[0] && strpbrk(connect, DIGIT_CHARS)) break;
      if (now() >= deadline) break;
      continue;
    }

    /* keep every byte for pppd; drop the call if the buffer cannot hold it */
    if (ppp_len >= sizeof(ppp_buf)) return CAPTURE_FULL;
    ppp_buf[ppp_len++] = c;

    /* start of the PPP stream - stop scanning text (byte is kept) */
    if (c == PPP_FLAG) break;

    if (c == '\r' || c == '\n') {
      line[line_len] = '\0';
      if (!binary) parse_connect_line(line); /* binary lines are not results */
      line_len = 0;
      binary = 0;
      if (connect[0] && strpbrk(connect, DIGIT_CHARS)) break;
      continue;
    }

    /* a non-text byte marks the line unparseable but does not stop the scan,
     * so flow-control noise (XON/XOFF/NUL) is ignored rather than treated as
     * the start of binary data */
    if (c < 0x20 && c != '\t') {
      binary = 1;
      continue;
    }
    if (line_len < sizeof(line) - 1) line[line_len++] = (char)c;
  }
  return CAPTURE_OK;
}

/* ------------------------------------------------------------------ */
/* RING handling                                                      */
/* ------------------------------------------------------------------ */

/* a result line that really is a RING (not RINGER, RINGBACK, ...) */
static int is_ring_line(const char *line)
{
  return strcmp(line, "RING") == 0 || strncmp(line, "RING;", 5) == 0 ||
         strncmp(line, "RING ", 5) == 0;
}

/*
 * Read result lines until a RING, OK or abort (ERROR, ...) shows up, or the
 * timeout expires.
 *   returns WAIT_RING    on a RING line
 *           WAIT_OK      on an "OK" line
 *           WAIT_ABORT   on a modem action line (ERROR, NO CARRIER, ...)
 *           WAIT_TIMEOUT on timeout / error
 *           SERIAL_LOST  if the line went away
 * timeout_ms < 0 waits forever.
 */
static int wait_ring_or_ok(int fd, int timeout_ms)
{
  char line[512];
  int r;
  long long deadline = timeout_ms >= 0 ? now() + timeout_ms : 0;
  size_t line_len = 0;
  unsigned char c;

  reset_call_data(); /* clean slate before any RING is parsed */

  for (;;) {
    r = serial_read_char(fd, 1000, &c);

    if (r == SERIAL_LOST) return SERIAL_LOST;
    if (r == SERIAL_ERROR) return WAIT_TIMEOUT;
    if (r == SERIAL_TIMEOUT) {
      if (timeout_ms >= 0 && now() >= deadline) return WAIT_TIMEOUT;
      continue;
    }

    if (c != '\r' && c != '\n') {
      if (line_len == 0 && isspace(c)) continue;
      if (line_len < sizeof(line) - 1) line[line_len++] = (char)c;
      continue;
    }

    /* end of line */
    if (line_len == 0) continue;
    line[line_len] = '\0';
    cnd_process(line);

    /* cnd_process() already pulled out any caller id from "RING;<...>" */
    if (is_ring_line(line)) return WAIT_RING;
    if (strcmp(line, "OK") == 0) return WAIT_OK;
    if (is_abort_line(line, line_len)) return WAIT_ABORT;

    /* not interesting - keep listening */
    line_len = 0;
  }
}

/* ------------------------------------------------------------------ */
/* PPP frame detection (state machine over the PPP frame header)       */
/* ------------------------------------------------------------------ */

/*
 * Feed one byte to the PPP frame detector.
 *   1  = LCP header recognised
 *   0  = still plausible PPP / harmless leading noise
 *  -1  = first meaningful byte is not a PPP frame start (not PPP)
 */
static int ppp_feed(unsigned char c)
{
  unsigned char pch;

  if (ppp_escaped) {
    /* an escaped byte is data: never a delimiter or a new escape */
    ppp_escaped = 0;
    pch = PPP_UNESCAPE(c);
  }
  else {
    /*
     * 0x7e is the frame delimiter: it starts a frame when idle and also
     * resynchronizes to a new frame after a partial/garbled one, so it is
     * handled before the idle "not PPP" test below.
     */
    if (c == PPP_FLAG) {
      ppp_level = 1;
      return 0;
    }
    if (ppp_level != 0 && c == PPP_ESC) {
      ppp_escaped = 1;
      return 0;
    }
    pch = c;
  }

  if (ppp_level == 0) {
    if (pch == '\r' || pch == '\n' || pch == '\0') return 0; /* modem noise */
    return -1;                                               /* not PPP */
  }

  if (pch == PPP_ADDRESS && ppp_level == 1) ppp_level = 2;
  else if (pch == PPP_CTRL && ppp_level == 2) ppp_level = 3;
  else if (pch == PPP_LCP_PROTO_HI && ppp_level == 3) ppp_level = 4;
  else if (pch == PPP_LCP_PROTO_LOW && ppp_level == 4) return 1;
  else {
    ppp_level = 0;
    ppp_escaped = 0;
  }

  return 0;
}

/* ------------------------------------------------------------------ */
/* environment, privileges and relay                                  */
/* ------------------------------------------------------------------ */

/* setenv() that reports failure; DEVICE is required so its failure is fatal */
static void set_env(const char *name, const char *val, int fatal)
{
  if (setenv(name, val, 1) == 0) return;
  if (fatal) die("cannot set %s: %s", name, strerror(errno));
  fprintf(stderr, "m2ppp: cannot set %s: %s\n", name, strerror(errno));
}

static void setup_environment(void)
{
  if (caller_id && *caller_id) set_env("CALLER_ID", caller_id, 0);
  else unsetenv("CALLER_ID");
  if (called_nr && *called_nr) set_env("CALLED_ID", called_nr, 0);
  else unsetenv("CALLED_ID");
  if (opt_term && *opt_term) set_env("TERM", opt_term, 0);
  set_env("CONNECT", connect ? connect : "", 0);
  set_env("DEVICE", opt_device, 1);
}

/*
 * Drop privileges to "user" (the -u option). If the user cannot be
 * resolved or the uid/gid cannot be set, we quit instead of running
 * pppd with the wrong privileges.
 */
static int switch_user(const char *user)
{
  struct passwd *pw = getpwnam(user);

  if (pw == NULL) return -1;

  if (setgid(pw->pw_gid) < 0 || initgroups(pw->pw_name, pw->pw_gid) < 0 || setuid(pw->pw_uid) < 0)
    return -1;
  return 0;
}

enum relay_result {
  relay_closed = 0, /* pppd side closed, or a generic end */
  relay_idle,       /* PPP session idle too long */
  relay_lost,       /* serial device went away */
  relay_exited      /* the watched child exited and was reaped */
};

/* pass the PTY's last output (e.g. "logout") on to the modem: until it has
 * been quiet for CHILD_POLL_MS, at most LOGIN_DRAIN_MS */
static void relay_drain(int master, int modem)
{
  struct pollfd pfd = {.fd = master, .events = POLLIN};
  long long end = now() + LOGIN_DRAIN_MS;
  unsigned char buf[4096];
  ssize_t n;

  while (now() < end && poll(&pfd, 1, CHILD_POLL_MS) > 0) {
    n = read(master, buf, sizeof(buf));
    if (n <= 0 || write_full(modem, buf, (size_t)n) < 0) return;
  }
}

/*
 * shuttle data both ways between the real modem line and the child's PTY.
 * With watch (the PTY login), the relay ends when that child exits (reaped
 * into *status), checked every CHILD_POLL_MS: a process the caller left
 * running keeps the PTY open, so the master never hangs up by itself. A
 * hangup on the master while the child runs is login's vhangup(), which
 * closes every slave fd until login reopens its tty: the modem is not read
 * during that gap, so the caller's input waits in the line; a gap longer than
 * LOGIN_GAP_MS ends the relay. The idle timeout (idle_sec) counts only data
 * from the modem: pppd's own keepalives (LCP echo) must not keep a call up
 * the caller has gone quiet on.
 */
static int relay_loop(int modem, int master, int idle_sec, pid_t watch, int *status)
{
  struct pollfd pfd[2];
  int r, timeout;
  long long last_rx = now(), left, gap_start = 0;
  ssize_t n;
  unsigned char buf[4096];

  pfd[0].fd = modem;
  pfd[0].events = POLLIN;
  pfd[1].fd = master;
  pfd[1].events = POLLIN;

  for (;;) {
    check_signal();
    timeout = POLL_SLICE_MS;
    if (watch > 0) {
      if (wait_child(watch, status, 0) == 0) {
        relay_drain(master, modem);
        return relay_exited;
      }
      timeout = CHILD_POLL_MS;
    }
    if (idle_sec > 0) {
      left = (long long)idle_sec * 1000 - (now() - last_rx);
      if (left <= 0) {
        step("no data from the caller for %d minute(s), hanging up", opt_ppp_idle_minutes);
        return relay_idle;
      }
      if (left < timeout) timeout = (int)left;
    }

    r = poll(pfd, 2, timeout);
    if (r < 0) {
      if (errno == EINTR) {
        check_signal();
        continue;
      }
      return relay_closed;
    }
    if (r == 0) continue; /* idle deadline re-checked at the top */

    if (pfd[1].revents & (POLLIN | POLLHUP | POLLERR | POLLNVAL)) {
      n = read(master, buf, sizeof(buf));
      if (n < 0 && errno == EINTR) {
        check_signal();
        continue;
      }
      if (n <= 0) {
        if (watch <= 0) return relay_closed;
        if (!gap_start) gap_start = now();
        else if (now() - gap_start > LOGIN_GAP_MS) {
          step("pty hung up for %dms with login still running, ending the relay", LOGIN_GAP_MS);
          return relay_closed;
        }
        nap(CHILD_POLL_MS); /* the master stays hung up until the slave is reopened */
        continue;
      }
      if (write_full(modem, buf, (size_t)n) < 0) return relay_lost;
    }
    gap_start = 0;
    if (pfd[0].revents & (POLLIN | POLLHUP | POLLERR | POLLNVAL)) {
      n = read(modem, buf, sizeof(buf));
      if (n < 0 && errno == EINTR) {
        check_signal();
        continue;
      }
      if (n <= 0) return relay_lost;
      last_rx = now();
      if (write_full(master, buf, (size_t)n) < 0) return relay_closed;
    }
  }
}

/* reap a child, terminating it first and escalating to SIGKILL if needed */
static void reap_child(pid_t pid, int *status, int terminate)
{
  if (terminate) {
    kill_child(pid, SIGTERM);
    if (wait_child(pid, status, CHILD_TERM_MS) == 0) return;
    step("child did not exit, sending SIGKILL");
    kill_child(pid, SIGKILL);
  }
  wait_child(pid, status, -1);
}

/* ------------------------------------------------------------------ */
/* login fallback                                                     */
/* ------------------------------------------------------------------ */

/*
 * Send /etc/issue.net before the login prompt, keeping only plain ASCII
 * (printable, CR, LF, TAB) so escape/binary sequences are dropped.
 */
static void show_issue(int fd)
{
  char buf[256];
  FILE *f = fopen(ISSUE_FILE, "r");
  int ch;
  size_t n = 0;

  if (!f) return;

  while ((ch = fgetc(f)) != EOF) {
    if (ch != '\n' && ch != '\r' && ch != '\t' && (ch < 0x20 || ch > 0x7e)) continue;
    buf[n++] = (char)ch;
    if (n == sizeof(buf)) {
      write_full(fd, buf, n);
      n = 0;
    }
  }
  if (n) write_full(fd, buf, n);
  fclose(f);
}

/*
 * Apply one typed character to the login-name buffer: append a printable
 * char (bounded by max), or erase the last char on DEL/BS. Returns 1 if the
 * buffer changed, so an echoing caller knows whether to redraw.
 */
static int name_edit(unsigned char c, char *name, size_t *len, size_t max)
{
  if (c == 0x7f || c == 0x08) {
    if (*len == 0) return 0;
    (*len)--;
    return 1;
  }
  if (isprint(c) && *len < max) {
    name[(*len)++] = (char)c;
    return 1;
  }
  return 0;
}

/*
 * Append one typed character to the login name and echo it back
 * (printable chars as-is, backspace as "\b \b").
 */
static void name_putc(int fd, char *name, size_t *len, size_t max, unsigned char c)
{
  if (!name_edit(c, name, len, max)) return;
  if (c == 0x7f || c == 0x08) write_full(fd, "\b \b", 3);
  else write_full(fd, &c, 1);
}

/*
 * Handle one non-newline byte at the login prompt (mgetty style):
 *   ^C / ^U   discard the name and show the prompt again
 *   ^D        abort if the name is empty
 *   ^\        abort
 *   ^Q/^S/ESC ignored
 * Anything else goes through name_putc() (printable / backspace).
 * Returns 1 when the caller should abort the login.
 */
static int login_special(int fd, char *name, size_t *len, size_t max, const char *prompt,
                         unsigned char c)
{
  switch (c) {
    case 0x03: /* ^C */
    case 0x15: /* ^U */
      *len = 0;
      write_full(fd, prompt, strlen(prompt));
      return 0;
    case 0x04: /* ^D */
      return (*len == 0);
    case 0x1c: /* ^\ */
      return 1;
    case 0x11: /* ^Q */
    case 0x13: /* ^S */
    case 0x1b: /* ESC */
      return 0;
    default:
      name_putc(fd, name, len, max, c);
      return 0;
  }
}

static void create_pty(int *master_out, int *slave_out, char *slave_name, size_t name_len)
{
  const char *name;
  int master, slave;
  struct termios t;

  master = posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (master < 0 || grantpt(master) < 0 || unlockpt(master) < 0)
    die("cannot create pty: %s", strerror(errno));
  name = ptsname(master);
  if (name == NULL) die("ptsname failed: %s", strerror(errno));
  snprintf(slave_name, name_len, "%s", name);

  slave = open(slave_name, O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (slave < 0) die("cannot open %s: %s", slave_name, strerror(errno));
  step("pty slave: %s", slave_name);
  if (tcgetattr(slave, &t) < 0) die("cannot get tty attrs for %s: %s", slave_name, strerror(errno));
  cfmakeraw(&t);
  t.c_cflag |= (CLOCAL | CREAD);
  if (tcsetattr(slave, TCSANOW, &t) < 0)
    die("cannot configure %s: %s", slave_name, strerror(errno));

  *master_out = master;
  *slave_out = slave;
}

/* make fd this process's stdin/stdout/stderr, clearing CLOEXEC on the copies
 * (dup2 is a no-op when oldfd == newfd, so fd 0 needs the explicit clear) */
static void attach_tty(int fd)
{
  dup2(fd, STDIN_FILENO);
  dup2(fd, STDOUT_FILENO);
  dup2(fd, STDERR_FILENO);
  fcntl(STDIN_FILENO, F_SETFD, 0);
  fcntl(STDOUT_FILENO, F_SETFD, 0);
  fcntl(STDERR_FILENO, F_SETFD, 0);
  if (fd > STDERR_FILENO) close(fd);
}

/* m2ppp ignores SIGPIPE and SIGALRM; an ignored signal stays ignored across
 * exec, so give login/pppd (and the shells they start) the defaults back */
static void reset_signals(void)
{
  signal(SIGALRM, SIG_DFL);
  signal(SIGPIPE, SIG_DFL);
}

/* fork a child on the PTY slave (its controlling terminal), optionally
 * dropping privileges, then exec cmd */
static pid_t spawn_child(const char *cmd, char *argv[], int master, int slave, const char *runas)
{
  pid_t pid = fork();

  if (pid < 0) die("fork failed: %s", strerror(errno));
  if (pid == 0) {
    setsid();
    if (ioctl(slave, TIOCSCTTY, 0) < 0) die("cannot set controlling tty: %s", strerror(errno));
    if (runas && switch_user(runas) != 0) _exit(1);
    if (serial_fd >= 0) close(serial_fd);
    if (master >= 0) close(master);
    reset_signals();
    attach_tty(slave);
    execv(cmd, argv);
    _exit(1);
  }
  return pid;
}

/*
 * Start a utmp record of `type` for `line` and `pid`. The name is taken
 * without the leading "/dev/", matching login's ttyname, and the id is its
 * last 4 characters, so the records for one line replace each other.
 */
static void utmp_init(struct utmpx *ut, const char *line, short type, pid_t pid)
{
  const char *tty = line;
  struct timeval tv;
  size_t n;

  if (strncmp(tty, "/dev/", 5) == 0) tty += 5;
  n = strlen(tty);

  memset(ut, 0, sizeof(*ut));
  ut->ut_type = type;
  ut->ut_pid = pid;
  snprintf(ut->ut_line, sizeof(ut->ut_line), "%s", tty);
  snprintf(ut->ut_id, sizeof(ut->ut_id), "%s", tty + (n > 4 ? n - 4 : 0));
  gettimeofday(&tv, NULL);
  ut->ut_tv.tv_sec = (unsigned int)tv.tv_sec;
  ut->ut_tv.tv_usec = (unsigned int)tv.tv_usec;
}

/* write ut to utmp and append it to wtmp. Best effort: errors (e.g.
 * non-root) are ignored. */
static void utmp_write(const struct utmpx *ut)
{
  setutxent();
  pututxline(ut);
  endutxent();
  updwtmpx(wtmp_path, ut);
}

/*
 * Register `line` as a login process in utmp/wtmp (a getty does this; the
 * login program updates the entry to USER_PROCESS on success).
 */
static void put_utmp(const char *line)
{
  struct utmpx ut;

  utmp_init(&ut, line, LOGIN_PROCESS, getpid());
  snprintf(ut.ut_user, sizeof(ut.ut_user), "LOGIN");
  if (caller_id && strcmp(caller_id, "none") != 0)
    snprintf(ut.ut_host, sizeof(ut.ut_host), "%s", caller_id);
  else if (connect) snprintf(ut.ut_host, sizeof(ut.ut_host), "%s", connect);
  utmp_write(&ut);
}

/*
 * Mark the login session on `line` as ended (DEAD_PROCESS, with login's
 * wait status), as init/getty do when login exits; otherwise who/w keep
 * showing the user and last reports "still logged in".
 */
static void end_utmp(const char *line, pid_t pid, int status)
{
  struct utmpx ut;

  utmp_init(&ut, line, DEAD_PROCESS, pid);
  if (WIFEXITED(status)) ut.ut_exit.e_exit = (short)WEXITSTATUS(status);
  else if (WIFSIGNALED(status)) ut.ut_exit.e_termination = (short)WTERMSIG(status);
  utmp_write(&ut);
}

/*
 * Hand the line to the login program and supervise it. We always fork so
 * m2ppp stays the owner of the line: the child becomes a session leader,
 * takes the tty as its controlling terminal, writes the utmp entry (with
 * its own pid) and execs login; the parent waits for the session to end
 * and then returns, so the state machine can hang up and wait for the
 * next ring.
 *
 * TIOCSCTTY is called with arg 0 ("don't steal"): if the tty is already
 * the controlling terminal of another session (a stale login/bash), the
 * ioctl fails with EPERM, the child reports that over a pipe, and we
 * return -1 so the caller backs off rather than stealing the line.
 *
 * The pipe carries one int: LOGIN_TTY_BUSY for the TIOCSCTTY failure, or
 * the errno of a failed execv() so it can be logged here; nothing (EOF)
 * once login has been exec'd.
 */
#define LOGIN_TTY_BUSY (-1)

static int exec_login(int fd, const char *prog, char *const argv[])
{
  int pfd[2], status, err = 0;
  ssize_t n;
  pid_t pid;

  /* a failure to start login is not a busy tty: report it and end the call */
  if (pipe2(pfd, O_CLOEXEC) < 0) {
    fprintf(stderr, "m2ppp: cannot start %s: %s\n", prog, strerror(errno));
    return 0;
  }

  pid = fork();
  if (pid < 0) {
    fprintf(stderr, "m2ppp: cannot start %s: %s\n", prog, strerror(errno));
    close(pfd[0]);
    close(pfd[1]);
    return 0;
  }
  if (pid == 0) {
    close(pfd[0]);
    setsid();
    if (ioctl(fd, TIOCSCTTY, 0) < 0) {
      err = LOGIN_TTY_BUSY;
      if (write(pfd[1], &err, sizeof(err)) < 0) _exit(1);
      _exit(1);
    }
    put_utmp(opt_device); /* the login process owns its pid and the tty now */
    reset_signals();
    attach_tty(fd);
    execv(prog, argv);
    err = errno;
    if (write(pfd[1], &err, sizeof(err)) < 0) _exit(1);
    _exit(1);
  }

  child_pid = pid;
  close(pfd[1]);
  n = read(pfd[0], &err, sizeof(err)); /* EOF once the child execs (CLOEXEC closes pfd[1]) */
  close(pfd[0]);
  /* past TIOCSCTTY the child has written its utmp entry */
  if (!(n == (ssize_t)sizeof(err) && err == LOGIN_TTY_BUSY)) child_utmp_line = opt_device;
  wait_child(pid, &status, -1);
  child_pid = -1;
  child_utmp_line = NULL;

  if (n == (ssize_t)sizeof(err) && err == LOGIN_TTY_BUSY) return -1; /* another session's tty */
  end_utmp(opt_device, pid, status); /* the child wrote the entry before execv */
  if (n == (ssize_t)sizeof(err)) {
    fprintf(stderr, "m2ppp: cannot execute %s: %s\n", prog, strerror(err));
    return 0;
  }
  step("login exited with status %d", WIFEXITED(status) ? WEXITSTATUS(status) : 1);
  return 0;
}

/* close any previous fd, then open and configure the line. 0 on success. */
static int open_line(void)
{
  int saved;

  if (serial_fd >= 0) {
    /* the flock belongs to the open file, which login (and anything it left
     * running) shares through a dup: drop it, or that dup keeps the line
     * locked against our own reopen. The UUCP lock still guards the line. */
    flock(serial_fd, LOCK_UN);
    tcflush(serial_fd, TCOFLUSH); /* unsent output (CTS low) would stall close() */
    close(serial_fd);
    serial_fd = -1;
  }
  step("opening %s", opt_device);
  serial_fd = serial_open(opt_device);
  if (serial_fd < 0) return -1;
  if (serial_config(serial_fd, opt_baud) < 0) {
    saved = errno;
    close(serial_fd);
    serial_fd = -1;
    errno = saved;
    return -1;
  }
  serial_flush_input(serial_fd); /* drop stale bytes from before the open */
  step("line ready (%s, %d baud)", opt_device, opt_baud);
  return 0;
}

/* the PAM service file login -h needs; M2PPP_PAM_REMOTE overrides it (tests) */
static const char *pam_remote_file(void)
{
  const char *env = getenv("M2PPP_PAM_REMOTE");

  return env && *env ? env : PAM_REMOTE_FILE;
}

/*
 * -H: the remote host for login -h, i.e. the caller id's digits (a valid host
 * name for logind, which shows it as FROM in w/who). NULL if there is no
 * caller id, or if the PAM service file is missing: util-linux login uses the
 * PAM service "remote" with -h, and without it every login would fail.
 */
static const char *login_remote_host(char *buf, size_t size)
{
  const char *p;
  size_t n = 0;

  if (!opt_remote_host || caller_id == NULL) return NULL;
  for (p = caller_id; *p && n + 1 < size; p++)
    if (isdigit((unsigned char)*p)) buf[n++] = *p;
  buf[n] = '\0';
  if (n == 0) return NULL;
  if (access(pam_remote_file(), F_OK) < 0) {
    step("%s missing, not passing the caller id to login (-H)", pam_remote_file());
    return NULL;
  }
  return buf;
}

/*
 * Run the login program for `name` (the st_exec_login handoff).
 * Returns 0 when the session ended (or the tty was busy), -1 if the line was
 * lost and the caller should reopen it.
 */
static int run_login(const char *name)
{
  char *argv[5], hostbuf[64];
  const char *prog = opt_login, *slash, *host;
  int argc = 0;
  struct termios t;

  slash = strrchr(prog, '/');
  argv[argc++] = (char *)(slash ? slash + 1 : prog);
  host = login_remote_host(hostbuf, sizeof(hostbuf));
  if (host) {
    step("login remote host: %s", host);
    argv[argc++] = (char *)"-h";
    argv[argc++] = (char *)host;
  }
  argv[argc++] = (char *)name;
  argv[argc] = NULL;

  step("login as '%s'", name);

  /* Hand the line to login and stay its owner: the UUCP lock and the open fd
   * are kept for the whole process, and exec_login() returns once the login
   * session ends so the state machine hangs up and waits for the next ring.
   * A busy tty (exec_login -1) is a back-off, not a lost line. */
  tio_mode_sane(serial_fd);
  setup_environment();
  if (exec_login(serial_fd, prog, argv) < 0) {
    step("cannot take the tty for login (already in use)");
    return 0;
  }

  /* without the vhangup shim, login's vhangup() revoked our fd too (every
   * ioctl now fails with EIO): reopen it so hangup_line() and the cooldown
   * act on the real line, and only report a lost line if that fails */
  if (tio_get(serial_fd, &t) < 0) {
    step("line was hung up by login, reopening %s", opt_device);
    if (open_line() < 0) return -1;
  }
  return 0;
}

/* show issue/banner + hostname login prompt (+ pre-prompt typing) */
static void show_login_prompt(int fd, const char *prompt, const char *name, size_t len)
{
  show_issue(fd);
  write_full(fd, prompt, strlen(prompt));
  if (len) write_full(fd, name, len);
}

/* buffer one keystroke received before the prompt is shown */
static void name_input(unsigned char c, char *name, size_t *len, size_t max)
{
  name_edit(c, name, len, max);
}

/*
 * After CONNECT: watch for a PPP frame for up to detect_sec. If no PPP frame
 * has been seen after prompt_sec, show the login prompt; detection keeps
 * running while the user types so a slow PPP client can still take over.
 * On Enter the collected login name is returned to the caller.
 *
 * Returns 1 if PPP was detected (hand over to pppd), 2 if a login was
 * requested (name filled in), 0 if the call should be dropped, -1 if the
 * line was lost.
 */
static int detect_type(int fd, int detect_sec, int prompt_sec, char *name, size_t name_max)
{
  char host[64], prompt[128];
  int fr, prompted = 0, r;
  long long cur, deadline, prompt_at, wait_ms;
  size_t i, len = 0;
  unsigned char c;

  if (name_max == 0) return DETECTED_DROP;
  name_max--; /* reserve space for the terminating NUL */

  step("waiting for PPP frame (%ds; login prompt after %ds)", detect_sec, prompt_sec);

  ppp_level = 0;
  ppp_escaped = 0;

  /* bytes buffered with the CONNECT result may already start the frame */
  for (i = 0; i < ppp_len; i++)
    if (ppp_feed(ppp_buf[i]) == 1) return DETECTED_PPP;

  if (gethostname(host, sizeof(host)) < 0) snprintf(host, sizeof(host), "localhost");
  host[sizeof(host) - 1] = '\0';
  snprintf(prompt, sizeof(prompt), "\r\n%s login: ", host);

  /* monotonic milliseconds so the prompt/deadline are hit precisely */
  cur = now();
  prompt_at = cur + (long long)prompt_sec * 1000;
  deadline = cur + (long long)detect_sec * 1000;

  for (;;) {
    cur = now();

    if (!prompted && cur >= prompt_at) {
      show_login_prompt(fd, prompt, name, len);
      step("no PPP after %ds, showing login prompt", prompt_sec);
      prompted = 1;
    }
    if (cur >= deadline) return DETECTED_DROP;

    /* next event is the prompt (if still pending) or the deadline; both follow cur */
    wait_ms = (prompted ? deadline : prompt_at) - cur;
    if (wait_ms > INT_MAX) wait_ms = INT_MAX;

    r = serial_read_char(fd, (int)wait_ms, &c);
    if (r == SERIAL_LOST) return DETECTED_LOST;
    if (r == SERIAL_ERROR) return DETECTED_DROP;
    if (r == SERIAL_TIMEOUT) continue;

    /* keep the byte for pppd and keep detecting PPP */
    if (ppp_len >= sizeof(ppp_buf)) {
      step("PPP replay buffer full, dropping the call");
      return DETECTED_DROP;
    }
    ppp_buf[ppp_len++] = c;
    fr = ppp_feed(c);
    if (fr == 1) return DETECTED_PPP;

    if (!prompted) {
      /* remember what was typed before the prompt, show it afterwards */
      name_input(c, name, &len, name_max);

      if (fr < 0) {
        show_login_prompt(fd, prompt, name, len);
        step("input is not PPP, showing login prompt");
        prompted = 1;
      }
      continue; /* still gathering before the prompt */
    }

    if (c == '\r' || c == '\n') {
      /* the name becomes login's argument: a leading '-' would be an option
       * (-f skips authentication, -h fakes the remote host) */
      if (len > 0 && name[0] == '-') {
        step("login name starting with '-' rejected");
        len = 0;
      }
      if (len == 0) {
        write_full(fd, prompt, strlen(prompt));
        continue;
      }
      name[len] = '\0';
      return DETECTED_LOGIN;
    }
    if (login_special(fd, name, &len, name_max, prompt, c)) return DETECTED_DROP;
  }
}

/* ------------------------------------------------------------------ */
/* hand off to pppd                                                   */
/* ------------------------------------------------------------------ */

static void build_pppd_argv(char *argv[], char *opts_file, size_t opts_size)
{
  const char *cmd = opt_pppd, *devname, *slash;
  int argc = 0, i;

  slash = strrchr(cmd, '/');
  argv[argc++] = (char *)(slash ? slash + 1 : cmd);
  for (i = 0; i < pppd_argc; i++) argv[argc++] = pppd_args[i];

  /*
   * pppd runs on a PTY, so it won't auto-read /etc/ppp/options.<device>
   * for the real line. Add it explicitly (as "file <name>"), derived
   * from DEVICE.
   */
  devname = strrchr(opt_device, '/');
  devname = devname ? devname + 1 : opt_device;
  if (opt_no_optsfile) {
    step("pppd options file disabled");
  }
  else if (snprintf(opts_file, opts_size, "/etc/ppp/options.%s", devname) < (int)opts_size &&
           access(opts_file, R_OK) == 0) {
    step("pppd options file: %s", opts_file);
    argv[argc++] = (char *)"file";
    argv[argc++] = opts_file;
  }

  argv[argc] = NULL;
}

/*
 * pppd flushes its tty's pending input when it sets the tty up (TCSAFLUSH,
 * TCSETSF2 in pppd 2.5), so bytes written before that are lost and the peer
 * has to retransmit its first LCP request (about 2s). The master is in packet
 * mode (TIOCPKT) until here: the kernel reports that flush as
 * TIOCPKT_FLUSHREAD, and the replay goes after it. Output pppd sends meanwhile
 * (its own LCP request) is passed on to the modem. If no flush shows up within
 * REPLAY_WAIT_MS, replay anyway. Returns -1 if writing to the modem failed
 * (line lost or stuck), else 0.
 */
static int replay_after_flush(int master, int modem)
{
  unsigned char buf[4096];
  struct pollfd pfd;
  long long deadline = now() + REPLAY_WAIT_MS, left;
  ssize_t n;
  int off = 0;

  for (;;) {
    check_signal();
    left = deadline - now();
    if (left <= 0) {
      step("pppd did not flush its tty within %dms, replaying anyway", REPLAY_WAIT_MS);
      break;
    }
    pfd.fd = master;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, (int)left) <= 0) continue; /* timeout or EINTR: re-check */
    n = read(master, buf, sizeof(buf));
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) break; /* pppd is gone: the relay sees it too */
    if (buf[0] == TIOCPKT_DATA) {
      if (n > 1 && write_full(modem, buf + 1, (size_t)n - 1) < 0) return -1;
    }
    else if (buf[0] & TIOCPKT_FLUSHREAD) {
      step("pppd flushed its tty");
      break;
    }
  }
  ioctl(master, TIOCPKT, &off); /* plain reads from here on (the relay) */
  step("replaying %zu PPP byte(s) to pppd", ppp_len);
  write_full(master, ppp_buf, ppp_len);
  return 0;
}

static int run_pppd(void)
{
  /* prog name + "modem" + MAX_PPPD_ARGS extras + "file" + path + NULL */
  char *argv[MAX_PPPD_ARGS + 5], opts_file[256], slave_name[128];
  int master, slave, rc, status, on = 1;
  pid_t pid;

  setup_environment();
  build_pppd_argv(argv, opts_file, sizeof(opts_file));

  /* pppd runs on a PTY, so keep the lock on the real line while we relay */
  create_pty(&master, &slave, slave_name, sizeof(slave_name));
  /* packet mode before pppd starts, so its tty flush is reported */
  if (ppp_len && ioctl(master, TIOCPKT, &on) < 0) {
    step("TIOCPKT: %s", strerror(errno));
    on = 0;
  }
  pid = spawn_child(opt_pppd, argv, master, slave, opt_runas);
  child_pid = pid;

  /* parent: relay */
  close(slave);
  rc = relay_closed;
  if (ppp_len && on && replay_after_flush(master, serial_fd) < 0) rc = relay_lost;
  else if (ppp_len && !on) {
    step("replaying %zu PPP byte(s) to pppd", ppp_len);
    write_full(master, ppp_buf, ppp_len);
  }

  if (rc != relay_lost) {
    step("pppd started, relaying");
    rc = relay_loop(serial_fd, master, opt_ppp_idle_minutes * 60, 0, &status);
  }
  if (rc == relay_idle || rc == relay_lost) {
    step(rc == relay_idle ? "idle timeout, terminating pppd"
                          : "serial line lost, terminating pppd");
  }
  reap_child(pid, &status, rc == relay_idle || rc == relay_lost);
  child_pid = -1;

  step("pppd exited with status %d", WIFEXITED(status) ? WEXITSTATUS(status) : 1);
  close(master);

  return rc == relay_lost ? -1 : 0;
}

/* ------------------------------------------------------------------ */
/* call teardown                                                      */
/* ------------------------------------------------------------------ */

/*
 * Cut off every process that still has the line open, e.g. a background job
 * the last caller left running, which could otherwise read the next call
 * (RING, typed passwords). A child takes the tty as its controlling terminal
 * and vhangup()s it: that revokes all open fds on it, ours included, so the
 * line is reopened. Needs root (CAP_SYS_TTY_CONFIG). Returns 0 if revoked and
 * reopened, 1 if not revoked (line unchanged), -1 if the reopen failed.
 */
static int revoke_line(void)
{
  int status;
  pid_t pid;

  /* without root vhangup() fails, and a child that took the tty and exits
   * would still hang up a real serial line behind our back: skip it */
  if (geteuid() != 0) {
    step("cannot revoke %s: not running as root", opt_device);
    return 1;
  }
  pid = fork();
  if (pid < 0) {
    step("cannot revoke %s: fork: %s", opt_device, strerror(errno));
    return 1;
  }
  if (pid == 0) {
    setsid();
    if (ioctl(serial_fd, TIOCSCTTY, 1) < 0 || vhangup() < 0) _exit(errno ? errno : 1);
    _exit(0);
  }
  wait_child(pid, &status, -1);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    step("cannot revoke %s: %s", opt_device,
         WIFEXITED(status) ? strerror(WEXITSTATUS(status)) : "child killed");
    return 1;
  }
  step("revoked %s (vhangup), reopening", opt_device);
  return open_line() < 0 ? -1 : 0;
}

/* hang the call up (B0), revoke leftover users of the line, then reset it for
 * the next call. The callers re-send the init afterwards: a modem set to reset
 * on DTR (&D3) is back at its stored profile and has lost E0/V1/#CID.
 * Returns -1 if the line was lost (reopen after the revoke failed). */
static int hangup_line(void)
{
  struct termios t;
  int rc;

  step("hanging up");
  if (serial_fd >= 0 && tio_get(serial_fd, &t) < 0) {
    /* the tty was hung up already (DCD drop on the carrier-sensitive line):
     * DTR is down and every fd on it, ours included, is revoked, so there is
     * nothing left to hang up or revoke; just reopen */
    step("line was hung up (carrier lost), reopening %s", opt_device);
    if (open_line() < 0) return -1;
  }
  else if (serial_fd >= 0) {
    tio_hangup(serial_fd);
    nap(500);
    rc = revoke_line();
    if (rc < 0) return -1;
    if (rc > 0) tio_raise(serial_fd, opt_baud); /* the reopen already set the line up */
    serial_flush_input(serial_fd);
  }

  /* optional cooldown before going back to "waiting for call" */
  if (opt_cooldown_seconds > 0) {
    step("cooling down for %d second(s) before next call", opt_cooldown_seconds);
    nap(opt_cooldown_seconds * 1000LL);
    if (serial_fd >= 0) serial_flush_input(serial_fd); /* drop RINGs meanwhile */
  }
  return 0;
}

/* ------------------------------------------------------------------ */
/* main                                                               */
/* ------------------------------------------------------------------ */

static void usage(FILE *fp, const char *argv0)
{
  fprintf(fp,
          "usage: %s [-d dev] [-b baud] [-i init] [-a answer]\n"
          "          [-t sec] [-L lockdir] [-p pppd] [-g login] [-u user]\n"
          "          [-w sec] [-P sec] [-c sec] [-I min] [-r sec] [-o sec]\n"
          "          [-T term] [-v] [-F] [-H] [-h] [-- pppd-args]\n",
          argv0);
}

/*
 * How long to wait between attempts to (re)open the line. The device may be
 * gone (unplugged / hung up); we keep the UUCP lock so the line stays reserved
 * while we are down. The retry interval is the modem-reinit interval, clamped to
 * at most 60s so a replug is noticed reasonably quickly.
 */
static int reopen_retry_seconds(void)
{
  int retry = opt_reinit_seconds > 0 ? opt_reinit_seconds : 1;

  if (retry > 60) retry = 60;
  return retry;
}

/* parse a required integer option; 0 on success, -1 on a malformed value */
static int parse_int(const char *arg, int *out)
{
  char *end;
  long val;

  errno = 0;
  val = strtol(arg, &end, 10);
  if (errno != 0 || end == arg || *end != '\0' || val < INT_MIN || val > INT_MAX) return -1;
  *out = (int)val;
  return 0;
}

/* parse CLI options and remaining pppd arguments */
static int parse_options(int argc, char **argv)
{
  int c;
  int bad;

  pppd_args[pppd_argc++] = (char *)"modem";

  while ((c = getopt(argc, argv, "d:b:i:a:t:L:p:g:u:vFHw:P:c:I:r:o:T:h")) != -1) {
    bad = 0;
    switch (c) {
      case 'd':
        opt_device = optarg;
        break;
      case 'b':
        bad = parse_int(optarg, &opt_baud);
        break;
      case 'i':
        opt_init = optarg;
        break;
      case 'a':
        opt_answer = optarg;
        break;
      case 't':
        bad = parse_int(optarg, &opt_answer_seconds);
        break;
      case 'L':
        opt_lockdir = optarg;
        break;
      case 'p':
        opt_pppd = optarg;
        break;
      case 'g':
        opt_login = optarg;
        break;
      case 'u':
        opt_runas = optarg;
        break;
      case 'v':
        opt_verbose = 1;
        break;
      case 'H':
        opt_remote_host = 1;
        break;
      case 'F':
        opt_no_optsfile = 1;
        break;
      case 'w':
        bad = parse_int(optarg, &opt_ppp_detect_seconds);
        break;
      case 'P':
        bad = parse_int(optarg, &opt_prompt_seconds);
        break;
      case 'c':
        bad = parse_int(optarg, &opt_cooldown_seconds);
        break;
      case 'I':
        bad = parse_int(optarg, &opt_ppp_idle_minutes);
        break;
      case 'r':
        bad = parse_int(optarg, &opt_reinit_seconds);
        break;
      case 'o':
        bad = parse_int(optarg, &opt_init_ok_seconds);
        break;
      case 'T':
        opt_term = optarg;
        break;
      case 'h':
        usage(stdout, argv[0]);
        return 1; /* help requested */
      default:
        usage(stderr, argv[0]);
        return -1;
    }
    if (bad) {
      usage(stderr, argv[0]);
      return -1;
    }
  }

  /* remaining arguments become pppd args (pppd_argc already counts "modem") */
  while (optind < argc) {
    if (pppd_argc < MAX_PPPD_ARGS + 1) pppd_args[pppd_argc++] = argv[optind];
    else
      fprintf(stderr, "m2ppp: too many pppd arguments (max %d), '%s' ignored\n", MAX_PPPD_ARGS,
              argv[optind]);
    optind++;
  }

  if (opt_ppp_detect_seconds < 1 || opt_prompt_seconds < 0 ||
      opt_prompt_seconds > opt_ppp_detect_seconds || opt_answer_seconds < 1 ||
      opt_answer_seconds > 3600 || !baud_supported(opt_baud) || opt_cooldown_seconds < 0 ||
      opt_cooldown_seconds > 86400 || opt_ppp_idle_minutes < 0 || opt_ppp_idle_minutes > 10080 ||
      opt_reinit_seconds < 0 || opt_reinit_seconds > 86400 || opt_init_ok_seconds < 1 ||
      opt_init_ok_seconds > 3600) {
    usage(stderr, argv[0]);
    return -1;
  }
  /* -u is used per call, after fork: catch a bad user name now */
  if (opt_runas && getpwnam(opt_runas) == NULL) {
    fprintf(stderr, "m2ppp: unknown user '%s' (-u)\n", opt_runas);
    return -1;
  }
  return 0;
}

/* cycle through the call states until a fatal error or a signal */
static int run_state_machine(void)
{
  enum m2ppp_state state = st_init;
  char login_name[128];
  int rc, retry;

  while (state != st_unknown) {
    check_signal();
    switch (state) {
      case st_open:
        if (open_line() < 0) {
          retry = reopen_retry_seconds();
          step("line not available, retrying in %ds", retry);
          nap(retry * 1000LL);
          state = st_open;
        }
        else {
          state = st_init;
        }
        break;

      case st_init:
        step("sending init: %s", opt_init);
        chat_send(serial_fd, opt_init);
        state = st_await_ok;
        break;

      case st_await_ok:
        step("waiting for OK after init");
        rc = wait_ring_or_ok(serial_fd, opt_init_ok_seconds * 1000);
        if (rc == WAIT_OK) {
          state = st_waiting;
        }
        else if (rc == WAIT_RING) {
          state = st_answer_phone;
        }
        else if (rc == SERIAL_LOST) {
          state = st_open;
        }
        else {
          /* an ERROR comes back at once: wait as long as a missing OK would,
           * or a rejected init string is re-sent in a tight loop */
          if (rc == WAIT_ABORT) {
            step("modem rejected init, retrying in %ds", opt_init_ok_seconds);
            nap(opt_init_ok_seconds * 1000LL);
          }
          step("no OK after init, reopening line");
          state = st_open;
        }
        break;

      case st_waiting:
        step("waiting for RING");
        rc = wait_ring_or_ok(serial_fd, opt_reinit_seconds > 0 ? opt_reinit_seconds * 1000 : -1);
        if (rc == WAIT_RING) {
          state = st_answer_phone;
        }
        else if (rc == WAIT_OK) {
          state = st_waiting; /* modem alive: reset the check timer */
        }
        else if (rc == SERIAL_LOST) {
          step("serial line lost");
          state = st_open;
        }
        else {
          state = st_init; /* idle timeout or modem error: re-send the init */
        }
        break;

      case st_answer_phone: {
        step("answering: %s", opt_answer);
        chat_send(serial_fd, opt_answer);
        rc = chat_expect(serial_fd, "CONNECT", opt_answer_seconds);
        if (rc == CHAT_LOST) {
          step("line lost while waiting for CONNECT");
          state = st_open;
          break;
        }
        if (rc != CHAT_MATCH) {
          step("no CONNECT (rc=%d)", rc);
          state = hangup_line() < 0 ? st_open : st_init;
          break;
        }
        rc = chat_capture_connect(serial_fd);
        if (rc == CAPTURE_LOST) {
          state = st_open;
          break;
        }
        if (rc == CAPTURE_FULL) {
          step("PPP replay buffer full during CONNECT, hanging up");
          state = hangup_line() < 0 ? st_open : st_init;
          break;
        }
        step("CONNECT: %s", connect[0] ? connect : "(no speed)");
        serial_set_call_mode(serial_fd);
        state = st_detect_ppp;
        break;
      }

      case st_detect_ppp:
        rc = detect_type(serial_fd, opt_ppp_detect_seconds, opt_prompt_seconds, login_name,
                         sizeof(login_name));
        if (rc == DETECTED_PPP) {
          step("PPP frame detected");
          state = st_exec_ppp;
        }
        else if (rc == DETECTED_LOGIN) {
          state = st_exec_login;
        }
        else if (rc == DETECTED_LOST) {
          state = st_open;
        }
        else {
          step("login/PPP timed out, hanging up");
          state = hangup_line() < 0 ? st_open : st_init;
        }
        break;

      case st_exec_login:
        write_full(serial_fd, "\r\n", 2);
        if (run_login(login_name) < 0) state = st_open; /* line lost */
        else {
          state = hangup_line() < 0 ? st_open : st_init;
        }
        break;

      case st_exec_ppp:
        step("handing over to pppd");
        if (run_pppd() < 0) state = st_open; /* line lost during the session */
        else {
          state = hangup_line() < 0 ? st_open : st_init;
        }
        break;

      case st_unknown:
      default:
        state = st_unknown;
        break;
    }
  }
  return 0;
}

int main(int argc, char **argv)
{
  int rc, saved;

  main_pid = getpid();
  rc = parse_options(argc, argv);
  if (rc > 0) return 0; /* -h: usage already printed */
  if (rc < 0) return 1;

  step("m2ppp built %s %s", __DATE__, __TIME__);
  if (opt_remote_host && access(pam_remote_file(), F_OK) < 0)
    fprintf(stderr,
            "m2ppp: warning: -H needs %s (login -h uses the PAM service \"remote\"); "
            "the caller id is not passed to login until it exists\n",
            pam_remote_file());

  install_signal(SIGINT);
  install_signal(SIGTERM);
  install_signal(SIGHUP);
  signal(SIGALRM, SIG_IGN);
  signal(SIGPIPE, SIG_IGN);

  reset_call_data(); /* allocate the default caller data */

  /*
   * Take the lock and check the device up front: if the line cannot be
   * opened or configured at startup, fail fast so a wrong -d path (or a
   * missing device) is visible instead of spinning here forever. Once
   * running, a device that disappears is re-opened by st_open/st_init.
   */
  if (lock_line() < 0) return 1;
  if (open_line() < 0) {
    saved = errno;
    die("cannot open %s: %s", opt_device, strerror(saved));
  }

  rc = run_state_machine();

  shutdown_call();
  return rc;
}
