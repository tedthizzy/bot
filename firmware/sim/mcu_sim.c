/* mcu-sim: firmware/core linked against a first-order wheel plant, speaking
 * the real line protocol on stdin and stdout.
 *
 * This is the C half of ARCHITECTURE 10's `mcu-sim`.  It is the *same* core
 * object code the ESP32-S3 runs (A3), so a gate that passes here is not
 * testing a second implementation of the safety logic -- which is the whole
 * reason firmware/core has no IDF headers in it.  `rover_devtools.mcu_sim`
 * owns the pty, the symlink and the Python-side flags; this binary owns the
 * plant and the injections named below.
 *
 * The fault flags are ARCHITECTURE 10's, in its own spelling, so the token the
 * document writes is the token the binary reads and `rover_devtools.mcu_sim`
 * passes straight through:
 *
 *   mcu-sim [ttl_drop] [garbage] [crc_flip] [reset_mid_drive] [hang=<ms>]
 *           [obstacle=<mm>] [no_target] [tof_error=fl|fr|both] [cliff] [bumper]
 *           [estop] [seq_replay] [seq_desync] [no_t_before_h] [lag=<ms>]
 *           [vbat=<volts>] [stall] [stall_one_channel] [slip_one_channel]
 *           [--session N] [--tick-us N] [--drop-frames-pct N]
 */
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "rover_core.h"

#define PLANT_FULL_DUTY_MM_S 450
#define PLANT_TAU_US 150000
#define PLANT_BRAKE_TAU_US 30000
#define WHEEL_CIRCUMFERENCE_UM 282743
#define CLIFF_APPEARS_US 3000000 /* after the 50-sample baseline is stored */
#define DELAY_QUEUE 256
/* esp_reset_reason_t's ESP_RST_TASK_WDT, which is what firmware/main copies
 * into cfg.reset_reason and what the B banner then carries (I-20). */
#define SIM_RESET_REASON_TASK_WDT 6

typedef struct {
    uint64_t due_us;
    uint16_t len;
    uint8_t bytes[ROVER_MAX_LINE_BYTES + 2];
} delayed_line_t;

typedef struct {
    int obstacle_mm;      /* obstacle=<mm>; no_target sets the 65534 sentinel */
    int tof_error;        /* tof_error=fl|fr|both, as bit0 front-L, bit1 front-R */
    int battery_mv;       /* vbat=<volts>, carried in mV */
    int drop_frames_pct;  /* --drop-frames-pct N: whole lines lost, as a UART loses them */
    int latency_ms;       /* lag=<ms> */
    int hang_ms;          /* hang=<ms>: the control step stops, then the WDT reboots */
    int session;
    int tick_us;
    bool stall;            /* both wheels held */
    bool stall_one;        /* stall_one_channel: the left wheel only */
    bool slip_one;         /* slip_one_channel: the left wheel at 40% of commanded */
    bool cliff;
    bool bumper;           /* the series-NC line is pulled low */
    bool estop;            /* the mushroom is pressed: coil node A reads low */
    bool ttl_drop;         /* every inbound line is lost once the wheels turn */
    bool garbage;          /* non-frame bytes into the decoder, 5 times a second */
    bool crc_flip;         /* one body byte of every 4th inbound line is corrupted */
    bool reset_mid_drive;  /* a new session under a still-open fd, once moving */
    bool seq_replay;       /* every 4th line is fed a second time */
    bool seq_desync;       /* the first V is re-fed later, far behind `last` */
    bool no_t_before_h;    /* T is withheld until an H has been accepted */
} options_t;

static void usage(FILE *out)
{
    fprintf(out,
            "mcu-sim -- the rover motion controller's core against a wheel plant\n"
            "\n"
            "Fault flags (ARCHITECTURE 10, in its own spelling):\n"
            "  obstacle=<mm>        forward ToF distance (default 2000)\n"
            "  no_target            both forward sensors return 65534: a clear path\n"
            "  tof_error=fl|fr|both that sensor returns 65535: TOF_STALE\n"
            "  cliff                the downward sensor sees the void after 3 s\n"
            "  bumper               the series-NC bumper line is pulled low\n"
            "  estop                the mushroom is pressed\n"
            "  stall                both wheels are held\n"
            "  stall_one_channel    the left wheel is held; the right runs free\n"
            "  slip_one_channel     the left wheel turns at 40%% of commanded\n"
            "  vbat=<volts>         pack voltage, e.g. vbat=9.5 (default 11.62)\n"
            "  lag=<ms>             delay every inbound line, e.g. lag=200ms\n"
            "  ttl_drop             lose every inbound line once the wheels turn\n"
            "  garbage              non-frame bytes into the decoder at 5 Hz\n"
            "  crc_flip             corrupt one body byte of every 4th line\n"
            "  seq_replay           feed every 4th line twice\n"
            "  seq_desync           re-feed an old line, far behind `last`\n"
            "  reset_mid_drive      mint a new session mid-drive, fd still open\n"
            "  hang=<ms>            stop the control step, then reboot WDT_REBOOT\n"
            "  no_t_before_h        withhold T until an H is accepted\n"
            "\n"
            "Simulator knobs (not fault injections):\n"
            "  --drop-frames-pct N  lose this percentage of inbound lines\n"
            "  --session N          MCU session id (default 40010)\n"
            "  --tick-us N          control period in us (default 10000)\n"
            "  --help               this text\n");
}

static bool parse_int(const char *text, int *out)
{
    char *end = NULL;
    long value = strtol(text, &end, 10);
    if (end == text || *end != '\0' || value < 0 || value > 100000) {
        return false;
    }
    *out = (int)value;
    return true;
}

/* `lag=200ms` carries its unit and `vbat=9.5` its decimal point, so the two
 * value forms ARCHITECTURE 10 writes are parsed where they are read. */
static bool parse_ms(const char *text, int *out)
{
    size_t n = strlen(text);
    char digits[16];
    if (n >= 2 && text[n - 2] == 'm' && text[n - 1] == 's') {
        n -= 2;
    }
    if (n == 0 || n >= sizeof digits) {
        return false;
    }
    memcpy(digits, text, n);
    digits[n] = '\0';
    return parse_int(digits, out);
}

static bool parse_millivolts(const char *text, int *out)
{
    char *end = NULL;
    double volts = strtod(text, &end);
    if (end == text || *end != '\0' || volts < 0.0 || volts > 100.0) {
        return false;
    }
    *out = (int)(volts * 1000.0 + 0.5);
    return true;
}

static bool parse_args(int argc, char **argv, options_t *opt)
{
    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        const char *value = NULL;
        char name[32];
        const char *equals = strchr(arg, '=');
        if (equals != NULL) {
            size_t n = (size_t)(equals - arg);
            if (n >= sizeof name) {
                fprintf(stderr, "mcu-sim: flag name too long: %s\n", arg);
                return false;
            }
            memcpy(name, arg, n);
            name[n] = '\0';
            value = equals + 1;
            arg = name;
        }

        /* --- ARCHITECTURE 10's fault flags, valueless ------------------- */
        if (strcmp(arg, "--help") == 0) {
            usage(stdout);
            exit(0);
        } else if (strcmp(arg, "ttl_drop") == 0) {
            opt->ttl_drop = true;
        } else if (strcmp(arg, "garbage") == 0) {
            opt->garbage = true;
        } else if (strcmp(arg, "crc_flip") == 0) {
            opt->crc_flip = true;
        } else if (strcmp(arg, "reset_mid_drive") == 0) {
            opt->reset_mid_drive = true;
        } else if (strcmp(arg, "no_target") == 0) {
            opt->obstacle_mm = ROVER_TOF_NO_TARGET_MM;
        } else if (strcmp(arg, "cliff") == 0) {
            opt->cliff = true;
        } else if (strcmp(arg, "bumper") == 0) {
            opt->bumper = true;
        } else if (strcmp(arg, "estop") == 0) {
            opt->estop = true;
        } else if (strcmp(arg, "seq_replay") == 0) {
            opt->seq_replay = true;
        } else if (strcmp(arg, "seq_desync") == 0) {
            opt->seq_desync = true;
        } else if (strcmp(arg, "no_t_before_h") == 0) {
            opt->no_t_before_h = true;
        } else if (strcmp(arg, "stall") == 0) {
            opt->stall = true;
        } else if (strcmp(arg, "stall_one_channel") == 0) {
            opt->stall_one = true;
        } else if (strcmp(arg, "slip_one_channel") == 0) {
            opt->slip_one = true;
        } else if (strcmp(arg, "tof_error") == 0) {
            if (value == NULL) {
                fprintf(stderr, "mcu-sim: tof_error needs fl, fr or both\n");
                return false;
            }
            if (strcmp(value, "fl") == 0) {
                opt->tof_error = 1;
            } else if (strcmp(value, "fr") == 0) {
                opt->tof_error = 2;
            } else if (strcmp(value, "both") == 0) {
                opt->tof_error = 3;
            } else {
                fprintf(stderr, "mcu-sim: tof_error must be fl, fr or both, got %s\n",
                        value);
                return false;
            }
        } else {
            /* --- the value-taking flags -------------------------------- */
            int *target = NULL;
            bool (*parse)(const char *, int *) = parse_int;
            if (strcmp(arg, "obstacle") == 0) {
                target = &opt->obstacle_mm;
            } else if (strcmp(arg, "hang") == 0) {
                target = &opt->hang_ms;
                parse = parse_ms;
            } else if (strcmp(arg, "lag") == 0) {
                target = &opt->latency_ms;
                parse = parse_ms;
            } else if (strcmp(arg, "vbat") == 0) {
                target = &opt->battery_mv;
                parse = parse_millivolts;
            } else if (strcmp(arg, "--drop-frames-pct") == 0) {
                target = &opt->drop_frames_pct;
            } else if (strcmp(arg, "--session") == 0) {
                target = &opt->session;
            } else if (strcmp(arg, "--tick-us") == 0) {
                target = &opt->tick_us;
            } else {
                fprintf(stderr, "mcu-sim: unknown option %s\n", arg);
                return false;
            }
            if (value == NULL) {
                if (++i >= argc) {
                    fprintf(stderr, "mcu-sim: %s needs a value\n", arg);
                    return false;
                }
                value = argv[i];
            }
            if (!parse(value, target)) {
                fprintf(stderr, "mcu-sim: bad value for %s: %s\n", arg, value);
                return false;
            }
        }
    }
    return true;
}

/* The simulator's stand-in for esp_timer_get_time(): in mcu-sim there is only
 * one clock and this is it, which is why the name says MCU and not host.  A
 * name from the host's vocabulary here would read to I-17's checker -- and to
 * a reviewer -- as the controller differencing a Pi timestamp. */
static uint64_t sim_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)(ts.tv_nsec / 1000);
}

/* ---- the plant --------------------------------------------------------- */

typedef struct {
    int64_t wheel_um_s[2];
    int64_t wheel_pos_um[2];
} plant_t;

static void plant_step(plant_t *plant, const rover_out_t *out, const options_t *opt,
                       uint32_t dt_us, rover_in_t *in)
{
    const uint32_t tau = out->brake ? PLANT_BRAKE_TAU_US : PLANT_TAU_US;
    const int16_t duty[2] = {out->duty_l_q15, out->duty_r_q15};
    int32_t current_ma = 0;
    for (int i = 0; i < 2; i++) {
        int64_t open_loop_um_s =
            out->brake ? 0
                       : ((int64_t)duty[i] * PLANT_FULL_DUTY_MM_S * 1000) /
                             ROVER_DUTY_MAX_Q15;
        if (opt->stall || (i == 0 && opt->stall_one)) {
            plant->wheel_um_s[i] = 0; /* the wheel is held; the motor is not */
        } else if (i == 0 && opt->slip_one) {
            /* A24's other branch: turning, but well under command, so the 5%
             * stall test never fires and only the 500 ms slip test can. */
            open_loop_um_s = (open_loop_um_s * 40) / 100;
            plant->wheel_um_s[i] +=
                ((open_loop_um_s - plant->wheel_um_s[i]) * dt_us) / tau;
        } else {
            plant->wheel_um_s[i] +=
                ((open_loop_um_s - plant->wheel_um_s[i]) * dt_us) / tau;
        }
        plant->wheel_pos_um[i] += (plant->wheel_um_s[i] * dt_us) / 1000000;

        /* The same motor model the core's I2t layer inverts, so the reported
         * pack current is consistent with the duty and the wheel speed. */
        int32_t applied_mv =
            (int32_t)(((int64_t)duty[i] * opt->battery_mv) / 32768);
        int32_t bemf_mv = (int32_t)((13270LL * (plant->wheel_um_s[i] / 1000)) / 1000);
        int32_t across = applied_mv - bemf_mv;
        int32_t channel_ma = (int32_t)(((int64_t)(across < 0 ? -across : across) *
                                        1000) /
                                       3430);
        current_ma += channel_ma;
    }
    in->left_ticks =
        (int32_t)((plant->wheel_pos_um[0] * ROVER_TICKS_PER_REV) / WHEEL_CIRCUMFERENCE_UM);
    in->right_ticks =
        (int32_t)((plant->wheel_pos_um[1] * ROVER_TICKS_PER_REV) / WHEEL_CIRCUMFERENCE_UM);
    in->imotor_ma = (int16_t)(current_ma > 32767 ? 32767 : current_ma);
}

/* ---- inbound: whole-line delay and drop -------------------------------- */

typedef struct {
    delayed_line_t queue[DELAY_QUEUE];
    int head;
    int count;
    uint8_t partial[ROVER_MAX_LINE_BYTES + 2];
    uint16_t partial_len;
    uint32_t rng;
    uint32_t lines;                              /* complete lines seen */
    bool drop_everything;                        /* ttl_drop, once the wheels turn */
    uint8_t stale[ROVER_MAX_LINE_BYTES + 2];     /* seq_desync's held-back line */
    uint16_t stale_len;
} inbound_t;

static uint32_t next_random(inbound_t *in)
{
    in->rng = in->rng * 1664525u + 1013904223u;
    return in->rng >> 16;
}

static void inbound_push(inbound_t *inbound, rover_core_t *core,
                         const options_t *opt, uint64_t now_us, const uint8_t *data,
                         size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (inbound->partial_len < sizeof inbound->partial) {
            inbound->partial[inbound->partial_len++] = data[i];
        }
        if (data[i] != '\n') {
            continue;
        }
        uint16_t len = inbound->partial_len;
        inbound->partial_len = 0;
        inbound->lines++;
        if (inbound->drop_everything) {
            continue; /* ttl_drop: the cable is out, and T1 is what stops the wheels */
        }
        if (opt->drop_frames_pct > 0 &&
            (int)(next_random(inbound) % 100u) < opt->drop_frames_pct) {
            continue; /* the whole line is lost, exactly as a UART glitch loses it */
        }
        /* seq_desync keeps the first V and re-feeds it much later, so `last`
         * has moved far past it -- A8's replay, not a one-frame duplicate. */
        if (opt->seq_desync && inbound->stale_len == 0 && len > 3 &&
            inbound->partial[1] == 'V') {
            memcpy(inbound->stale, inbound->partial, len);
            inbound->stale_len = len;
        }
        if (opt->crc_flip && (inbound->lines % 4u) == 0u && len > 6) {
            /* One body byte, so the line still frames and only the CRC fails. */
            inbound->partial[2] ^= 0x20u;
        }
        if (opt->latency_ms == 0) {
            rover_core_feed(core, inbound->partial, len);
            if (opt->seq_replay && (inbound->lines % 4u) == 0u) {
                rover_core_feed(core, inbound->partial, len);
            }
            continue;
        }
        if (inbound->count >= DELAY_QUEUE) {
            continue;
        }
        int slot = (inbound->head + inbound->count) % DELAY_QUEUE;
        inbound->queue[slot].due_us = now_us + (uint64_t)opt->latency_ms * 1000u;
        inbound->queue[slot].len = len;
        memcpy(inbound->queue[slot].bytes, inbound->partial, len);
        inbound->count++;
    }
}

static void inbound_release(inbound_t *inbound, rover_core_t *core, uint64_t now_us)
{
    while (inbound->count > 0 && inbound->queue[inbound->head].due_us <= now_us) {
        rover_core_feed(core, inbound->queue[inbound->head].bytes,
                        inbound->queue[inbound->head].len);
        inbound->head = (inbound->head + 1) % DELAY_QUEUE;
        inbound->count--;
    }
}

/* Drop `$T,...` lines from an outbound buffer, keeping every other frame.
 * The stream is newline-framed ASCII, so this is a line filter and nothing
 * more -- the core is untouched. */
static size_t strip_telemetry(uint8_t *buf, size_t n)
{
    size_t out = 0;
    size_t start = 0;
    while (start < n) {
        size_t end = start;
        while (end < n && buf[end] != '\n') {
            end++;
        }
        if (end < n) {
            end++; /* include the newline */
        }
        const size_t len = end - start;
        if (!(len >= 2 && buf[start] == '$' && buf[start + 1] == 'T')) {
            memmove(buf + out, buf + start, len);
            out += len;
        }
        start = end;
    }
    return out;
}

/* ---- main -------------------------------------------------------------- */

int main(int argc, char **argv)
{
    options_t opt = {
        .obstacle_mm = 2000,
        .tof_error = 0,
        .battery_mv = 11620,
        .drop_frames_pct = 0,
        .latency_ms = 0,
        .hang_ms = 0,
        .session = 40010,
        .tick_us = ROVER_CTRL_PERIOD_US,
    };
    if (!parse_args(argc, argv, &opt)) {
        usage(stderr);
        return 2;
    }
    if (opt.tick_us < 1000) {
        opt.tick_us = 1000;
    }

    rover_cfg_t cfg;
    rover_cfg_default(&cfg);
    rover_core_t *core = malloc(rover_core_size());
    if (core == NULL) {
        return 1;
    }
    plant_t plant;
    inbound_t inbound;
    rover_in_t in;
    rover_out_t out;
    memset(&plant, 0, sizeof plant);
    memset(&inbound, 0, sizeof inbound);
    memset(&in, 0, sizeof in);
    memset(&out, 0, sizeof out);
    inbound.rng = 0x2545F491u;
    in.tof_cliff_mm = 98;
    in.bumper_clear = 1;
    in.estop_released = 1;
    in.ntc_c = 30;
    in.gyro_z_mrad_s = 32767;
    out.brake = 1;
    out.pi_rail_en = 1;

    const uint64_t start_us = sim_now_us();
    rover_core_init(core, &cfg, (uint32_t)opt.session, 0);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    (void)fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    uint64_t next_tick_us = (uint64_t)opt.tick_us;
    uint64_t garbage_due_us = 200000;
    uint64_t hang_until_us = 0;
    uint32_t session = (uint32_t)opt.session;
    bool stdin_open = true;
    bool moved = false;      /* the wheels have turned at least once */
    bool restarted = false;  /* reset_mid_drive / hang has already fired */
    bool hello_seen = false; /* no_t_before_h: an H has been accepted */
    for (;;) {
        uint64_t now_us = sim_now_us() - start_us;
        if (stdin_open) {
            struct pollfd pfd = {STDIN_FILENO, POLLIN, 0};
            int wait_ms = (int)((next_tick_us > now_us ? next_tick_us - now_us : 0) /
                                1000u);
            int ready = poll(&pfd, 1, wait_ms > 0 ? wait_ms : 0);
            if (ready > 0) {
                uint8_t buffer[1024];
                ssize_t got = read(STDIN_FILENO, buffer, sizeof buffer);
                if (got > 0) {
                    inbound_push(&inbound, core, &opt, now_us, buffer, (size_t)got);
                } else if (got == 0) {
                    stdin_open = false; /* the host closed the link; keep running */
                }
            }
        } else {
            struct timespec nap = {0, 1000000};
            nanosleep(&nap, NULL);
        }

        now_us = sim_now_us() - start_us;
        if (now_us < next_tick_us) {
            continue;
        }
        next_tick_us += (uint64_t)opt.tick_us;
        if (next_tick_us < now_us) {
            next_tick_us = now_us + (uint64_t)opt.tick_us; /* never spiral */
        }

        inbound_release(&inbound, core, now_us);

        if (opt.garbage && now_us >= garbage_due_us) {
            /* Bytes that cannot frame: I-2 says they are dropped and counted,
             * and A5's premise is that the decoder meets them routinely. */
            static const uint8_t noise[] = "\x01\x02$not,a,frame*ZZZZ\n\xff\xfe";
            rover_core_feed(core, noise, sizeof noise - 1);
            garbage_due_us = now_us + 200000;
        }
        if (opt.seq_desync && moved && inbound.stale_len > 0 && !restarted) {
            rover_core_feed(core, inbound.stale, inbound.stale_len);
        }

        in.tof_fl_mm = (opt.tof_error & 1) ? (uint16_t)ROVER_TOF_ERROR_MM
                                           : (uint16_t)opt.obstacle_mm;
        in.tof_fr_mm = (opt.tof_error & 2) ? (uint16_t)ROVER_TOF_ERROR_MM
                                           : (uint16_t)opt.obstacle_mm;
        in.tof_status = (uint8_t)opt.tof_error;
        in.bumper_clear = opt.bumper ? 0 : 1;
        in.estop_released = opt.estop ? 0 : 1;
        in.vbat_mv = (uint16_t)opt.battery_mv;
        in.tof_cliff_mm = (opt.cliff && now_us >= CLIFF_APPEARS_US)
                              ? (uint16_t)(98 + ROVER_CLIFF_DELTA_MM + 50)
                              : 98;
        plant_step(&plant, &out, &opt, (uint32_t)opt.tick_us, &in);

        /* "Mid-drive" means the wheels are actually turning, not merely that a
         * setpoint was accepted: I-20 measures travel *during* the hang and
         * ttl_drop is a cable pull, so both have to fire on a moving plant. */
        if (!moved && rover_core_v_meas_mm_s(core) > 150) {
            moved = true;
            if (opt.ttl_drop) {
                inbound.drop_everything = true;
            }
            if (opt.hang_ms > 0) {
                hang_until_us = now_us + (uint64_t)opt.hang_ms * 1000u;
            }
        }

        /* I-20: the control task stops being serviced.  MCPWM holds the last
         * duty, so the plant keeps rolling; the Task-WDT then panic-resets into
         * DISARMED with WDT_REBOOT latched from the RTC reset reason. */
        if (hang_until_us != 0 && now_us < hang_until_us) {
            continue;
        }
        if (hang_until_us != 0) {
            hang_until_us = 0;
            restarted = true;
            cfg.wdt_reboot = true;
            cfg.reset_reason = SIM_RESET_REASON_TASK_WDT;
            session = session == 0xFFFFu ? 1u : session + 1u;
            rover_core_init(core, &cfg, session, now_us);
            hello_seen = false;
        }
        /* I-3's other half: a new session under a still-open fd, which is what
         * every real MCU restart looks like to a Pi whose tty never closed. */
        if (opt.reset_mid_drive && moved && !restarted) {
            restarted = true;
            session = session == 0xFFFFu ? 1u : session + 1u;
            rover_core_init(core, &cfg, session, now_us);
            hello_seen = false;
        }

        rover_core_step(core, now_us, &in, &out);

        uint8_t tx[4096];
        size_t n = rover_core_drain_tx(core, tx, sizeof tx);
        if (opt.no_t_before_h) {
            /* 5.1: T streams from boot regardless of H.  This flag withholds it
             * so G2 covers robotd's bounded re-seed fallback. */
            if (!hello_seen && rover_core_counters(core)->ok > 0) {
                hello_seen = true;
            }
            if (!hello_seen) {
                n = strip_telemetry(tx, n);
            }
        }
        size_t written = 0;
        while (written < n) {
            ssize_t wrote = write(STDOUT_FILENO, tx + written, n - written);
            if (wrote > 0) {
                written += (size_t)wrote;
            } else if (wrote < 0 && errno == EINTR) {
                continue;
            } else {
                free(core);
                return 0; /* the reader is gone */
            }
        }
    }
}
