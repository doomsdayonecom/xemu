/* retro_control.c — portable emulator control server. See retro_control.h
 * and ../SPEC.md. POSIX (Linux/macOS); on Windows the entry points are no-ops.
 *
 * One accept thread, one request in flight at a time. State reads are marshalled
 * to the emulator thread via retro_control_service() so they observe a
 * consistent machine; /step is driven cooperatively by retro_control_on_frame().
 */
#include "rrdc_core.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32) || defined(__EMSCRIPTEN__)
/* No TCP control server on Windows or in the browser (emscripten) — a localhost
 * socket server has no meaning there. Stub out so those builds still link. */
int  retro_control_start(int port, const retro_control_backend_t *b) { (void)port; (void)b; return -1; }
void retro_control_service(void) {}
int  retro_control_running(void) { return 1; }
void retro_control_on_frame(void) {}
void retro_control_stop(void) {}
#else

#include <pthread.h>
#include <stdatomic.h>
#include <unistd.h>
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define MEM_CAP (1u << 20)   /* /mem hard cap: 1 MiB per request */

static const retro_control_backend_t *g_be = NULL;
static int       g_listen_fd = -1;
static pthread_t g_thread;

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cv   = PTHREAD_COND_INITIALIZER;

/* run control: -1 free-run, 0 halted, >0 run this many frames then halt.
 * Atomic so retro_control_running() (called every instruction) reads it without
 * taking the lock — the mutex is kept only for the /step completion handshake. */
static _Atomic long g_frames_to_run = -1;
static int  g_step_waiting  = 0;

/* single-slot marshalled request. */
typedef enum { REQ_NONE = 0, REQ_MEM, REQ_REGS, REQ_SHOT, REQ_KEY, REQ_RESET,
               REQ_WRITE, REQ_AUDIO, REQ_PTR_SET, REQ_PTR_GET,
               REQ_PAD_SET, REQ_PAD_GET, REQ_JUMP } req_t;
static volatile req_t g_req = REQ_NONE;
static uint32_t g_req_addr, g_req_len;
static int32_t  g_req_bank;
static const uint8_t *g_write_data;   /* POST /mem body (HTTP-thread owned) */
static int      g_key_is_text;   /* 1 => g_key_value is a char code point */
static uint32_t g_key_value;
static int      g_key_action;    /* retro_key_action_t */
static int      g_ptr_absolute;  /* 1 => (x,y) absolute; 0 => relative deltas */
static int32_t  g_ptr_x, g_ptr_y;
static int      g_ptr_buttons;   /* button bitmask, or -1 = leave unchanged */
static int      g_pad_index;     /* which pad (0-based) */
static int      g_pad_buttons;   /* held mask, or -1 = leave unchanged */
static int      g_pad_connected; /* 1 plug, 0 unplug, -1 = unchanged */
static char     g_resp_extra[64];     /* extra response header line, or "" */

static uint8_t    *g_resp_body  = NULL;
static size_t      g_resp_len   = 0;
static int         g_resp_status = 200;
static const char *g_resp_ctype = "application/octet-stream";
static int         g_resp_ready = 0;

/* ---- emulator-thread side ------------------------------------------------ */

static void build_ppm(void)
{
    retro_framebuffer_t fb;
    memset(&fb, 0, sizeof fb);
    g_be->get_framebuffer(&fb);
    int w = fb.width, h = fb.height;
    int bpp = (fb.fmt == RETRO_PIX_RGB888) ? 3 : 4;

    char hdr[64];
    int hl = snprintf(hdr, sizeof hdr, "P6\n%d %d\n255\n", w, h);
    size_t total = (size_t)hl + (size_t)3 * w * h;
    uint8_t *out = (uint8_t *)malloc(total ? total : 1);
    if (!out) { g_resp_body = NULL; g_resp_len = 0; g_resp_status = 500; return; }

    memcpy(out, hdr, hl);
    uint8_t *p = out + hl;
    const uint8_t *s = fb.pixels;
    for (int i = 0; i < w * h; i++) {
        const uint8_t *px = s + (size_t)i * bpp;
        if (fb.fmt == RETRO_PIX_BGRA8888) {
            p[0] = px[2]; p[1] = px[1]; p[2] = px[0];
        } else {                       /* RGB888 / RGBA8888 */
            p[0] = px[0]; p[1] = px[1]; p[2] = px[2];
        }
        p += 3;
    }
    g_resp_body = out;
    g_resp_len = total;
    g_resp_status = 200;
    g_resp_ctype = "image/x-portable-pixmap";
}

#define AUDIO_MAX_SAMPLES (1u << 18)   /* drain cap: 256K int16 (~2.7 s stereo @48.8k) */

static void le16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void le32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

/* Drain the backend's audio ring and wrap it as a canonical PCM WAV. */
static void build_wav(void)
{
    static int16_t samp[AUDIO_MAX_SAMPLES];
    int rate = 48000, channels = 1;
    uint32_t dropped = 0;
    uint32_t n = g_be->capture_audio(samp, AUDIO_MAX_SAMPLES, &rate, &channels, &dropped);
    if (channels < 1) channels = 1;
    if (rate < 1) rate = 1;

    uint32_t data_bytes = n * 2;               /* int16 samples */
    uint32_t total = 44 + data_bytes;
    uint8_t *out = (uint8_t *)malloc(total);
    if (!out) { g_resp_body = NULL; g_resp_len = 0; g_resp_status = 500; return; }

    uint32_t byte_rate = (uint32_t)rate * channels * 2;
    memcpy(out, "RIFF", 4);       le32(out + 4, 36 + data_bytes);
    memcpy(out + 8, "WAVE", 4);
    memcpy(out + 12, "fmt ", 4);  le32(out + 16, 16);
    le16(out + 20, 1);            le16(out + 22, (uint16_t)channels);
    le32(out + 24, (uint32_t)rate);
    le32(out + 28, byte_rate);    le16(out + 32, (uint16_t)(channels * 2));
    le16(out + 34, 16);
    memcpy(out + 36, "data", 4);  le32(out + 40, data_bytes);
    memcpy(out + 44, samp, data_bytes);

    g_resp_body = out; g_resp_len = total; g_resp_status = 200;
    g_resp_ctype = "audio/wav";
    snprintf(g_resp_extra, sizeof g_resp_extra,
             "X-Rrdc-Audio-Dropped: %u\r\n", (unsigned)dropped);
}

void retro_control_service(void)
{
    if (!g_be || g_req == REQ_NONE) return;   /* cheap unlocked fast-path */

    pthread_mutex_lock(&g_lock);
    g_resp_extra[0] = 0;
    switch (g_req) {
    case REQ_MEM: {
        uint32_t len = g_req_len > MEM_CAP ? MEM_CAP : g_req_len;
        uint8_t *buf = (uint8_t *)malloc(len ? len : 1);
        uint32_t got = 0;
        if (buf && g_be->read_mem) got = g_be->read_mem(g_req_addr, g_req_bank, len, buf, len);
        g_resp_body = buf; g_resp_len = got;
        g_resp_status = g_be->read_mem ? 200 : 501;
        g_resp_ctype = "application/octet-stream";
        break;
    }
    case REQ_REGS: {
        char *buf = (char *)malloc(1024);
        if (buf && g_be->get_regs_json) { g_be->get_regs_json(buf, 1024); }
        else if (buf) { buf[0] = 0; }
        g_resp_body = (uint8_t *)buf; g_resp_len = buf ? strlen(buf) : 0;
        g_resp_status = g_be->get_regs_json ? 200 : 501;
        g_resp_ctype = "application/json";
        break;
    }
    case REQ_SHOT:
        if (g_be->get_framebuffer) build_ppm();
        else { g_resp_body = NULL; g_resp_len = 0; g_resp_status = 501; }
        break;
    case REQ_KEY: {
        const char *json;
        if (!g_be->inject_key) {
            json = "{\"error\":\"not implemented\"}"; g_resp_status = 501;
        } else if (g_be->inject_key(g_key_is_text, g_key_value, g_key_action)) {
            json = "{\"injected\":true}"; g_resp_status = 200;
        } else {
            json = "{\"error\":\"unmapped key\"}"; g_resp_status = 400;
        }
        size_t l = strlen(json);
        uint8_t *b = (uint8_t *)malloc(l + 1);
        if (b) memcpy(b, json, l + 1);
        g_resp_body = b; g_resp_len = b ? l : 0;
        g_resp_ctype = "application/json";
        break;
    }
    case REQ_JUMP: {
        const char *json;
        if (g_be->set_pc) { g_be->set_pc(g_req_addr); json = "{\"jumped\":true}"; g_resp_status = 200; }
        else { json = "{\"error\":\"not implemented\"}"; g_resp_status = 501; }
        size_t l = strlen(json);
        uint8_t *b = (uint8_t *)malloc(l + 1);
        if (b) memcpy(b, json, l + 1);
        g_resp_body = b; g_resp_len = b ? l : 0;
        g_resp_ctype = "application/json";
        break;
    }
    case REQ_RESET: {
        const char *json;
        if (g_be->reset) { g_be->reset(); json = "{\"reset\":true}"; g_resp_status = 200; }
        else { json = "{\"error\":\"not implemented\"}"; g_resp_status = 501; }
        size_t l = strlen(json);
        uint8_t *b = (uint8_t *)malloc(l + 1);
        if (b) memcpy(b, json, l + 1);
        g_resp_body = b; g_resp_len = b ? l : 0;
        g_resp_ctype = "application/json";
        break;
    }
    case REQ_WRITE: {
        const char *json; char *b;
        if (!g_be->write_mem) {
            json = "{\"error\":\"not implemented\"}"; g_resp_status = 501;
            size_t l = strlen(json); b = (char *)malloc(l + 1);
            if (b) memcpy(b, json, l + 1);
            g_resp_len = b ? l : 0;
        } else {
            uint32_t wrote = g_be->write_mem(g_req_addr, g_req_bank, g_req_len, g_write_data);
            b = (char *)malloc(48);
            g_resp_len = b ? (size_t)snprintf(b, 48, "{\"written\":%u}", (unsigned)wrote) : 0;
            g_resp_status = 200;
        }
        g_resp_body = (uint8_t *)b;
        g_resp_ctype = "application/json";
        break;
    }
    case REQ_AUDIO:
        if (g_be->capture_audio) build_wav();
        else {
            const char *json = "{\"error\":\"not implemented\"}";
            size_t l = strlen(json); uint8_t *b = (uint8_t *)malloc(l + 1);
            if (b) memcpy(b, json, l + 1);
            g_resp_body = b; g_resp_len = b ? l : 0; g_resp_status = 501;
            g_resp_ctype = "application/json";
        }
        break;
    case REQ_PTR_SET: {
        const char *json;
        if (!g_be->set_pointer) {
            json = "{\"error\":\"not implemented\"}"; g_resp_status = 501;
        } else if (g_be->set_pointer(g_ptr_absolute, g_ptr_x, g_ptr_y, g_ptr_buttons)) {
            json = "{\"injected\":true}"; g_resp_status = 200;
        } else {
            json = "{\"error\":\"no pointer\"}"; g_resp_status = 400;
        }
        size_t l = strlen(json);
        uint8_t *b = (uint8_t *)malloc(l + 1);
        if (b) memcpy(b, json, l + 1);
        g_resp_body = b; g_resp_len = b ? l : 0;
        g_resp_ctype = "application/json";
        break;
    }
    case REQ_PTR_GET: {
        char *b;
        if (!g_be->get_pointer) {
            const char *json = "{\"error\":\"not implemented\"}";
            size_t l = strlen(json); b = (char *)malloc(l + 1);
            if (b) memcpy(b, json, l + 1);
            g_resp_len = b ? l : 0; g_resp_status = 501;
        } else {
            int32_t x = 0, y = 0; int buttons = 0;
            int ok = g_be->get_pointer(&x, &y, &buttons);
            b = (char *)malloc(96);
            if (!b) { g_resp_len = 0; g_resp_status = 500; }
            else if (ok) {
                g_resp_len = (size_t)snprintf(b, 96,
                    "{\"x\":%ld,\"y\":%ld,\"buttons\":%d}",
                    (long)x, (long)y, buttons);
                g_resp_status = 200;
            } else {
                g_resp_len = (size_t)snprintf(b, 96, "{\"error\":\"no pointer\"}");
                g_resp_status = 400;
            }
        }
        g_resp_body = (uint8_t *)b;
        g_resp_ctype = "application/json";
        break;
    }
    case REQ_PAD_SET: {
        const char *json;
        if (!g_be->set_pad) {
            json = "{\"error\":\"not implemented\"}"; g_resp_status = 501;
        } else if (g_be->set_pad(g_pad_index, g_pad_buttons, g_pad_connected)) {
            json = "{\"injected\":true}"; g_resp_status = 200;
        } else {
            json = "{\"error\":\"no controller\"}"; g_resp_status = 400;
        }
        size_t l = strlen(json);
        uint8_t *b = (uint8_t *)malloc(l + 1);
        if (b) memcpy(b, json, l + 1);
        g_resp_body = b; g_resp_len = b ? l : 0;
        g_resp_ctype = "application/json";
        break;
    }
    case REQ_PAD_GET: {
        char *b;
        if (!g_be->get_pad) {
            const char *json = "{\"error\":\"not implemented\"}";
            size_t l = strlen(json); b = (char *)malloc(l + 1);
            if (b) memcpy(b, json, l + 1);
            g_resp_len = b ? l : 0; g_resp_status = 501;
        } else {
            int buttons = 0, connected = 0;
            int ok = g_be->get_pad(g_pad_index, &buttons, &connected);
            b = (char *)malloc(96);
            if (!b) { g_resp_len = 0; g_resp_status = 500; }
            else if (ok) {
                g_resp_len = (size_t)snprintf(b, 96,
                    "{\"index\":%d,\"connected\":%s,\"buttons\":%d}",
                    g_pad_index, connected ? "true" : "false", buttons);
                g_resp_status = 200;
            } else {
                g_resp_len = (size_t)snprintf(b, 96,
                    "{\"error\":\"no controller\"}");
                g_resp_status = 400;
            }
        }
        g_resp_body = (uint8_t *)b;
        g_resp_ctype = "application/json";
        break;
    }
    default: break;
    }
    g_req = REQ_NONE;
    g_resp_ready = 1;
    pthread_cond_broadcast(&g_cv);
    pthread_mutex_unlock(&g_lock);
}

int retro_control_running(void)
{
    if (!g_be) return 1;
    /* Lock-free hot path: a word-sized atomic read, no mutex. The gate
     * tolerates a one-iteration-stale value; /step correctness is enforced by
     * the mutex + condvar in on_frame/do_step, not by this read. */
    return atomic_load_explicit(&g_frames_to_run, memory_order_relaxed) != 0;
}

void retro_control_on_frame(void)
{
    if (!g_be) return;
    pthread_mutex_lock(&g_lock);
    if (g_frames_to_run > 0) {
        g_frames_to_run--;
        if (g_frames_to_run == 0 && g_step_waiting) {
            g_step_waiting = 0;
            g_resp_ready = 1;
            pthread_cond_broadcast(&g_cv);
        }
    }
    pthread_mutex_unlock(&g_lock);
}

/* ---- server-thread side -------------------------------------------------- */

static void write_all(int fd, const void *buf, size_t len)
{
    const char *p = (const char *)buf;
    while (len) {
        ssize_t w = send(fd, p, len, MSG_NOSIGNAL);
        if (w <= 0) return;
        p += w; len -= (size_t)w;
    }
}

/* `extra`, if non-NULL, is a complete header line ("Name: value\r\n"). */
static void send_http(int fd, int status, const char *ctype,
                      const uint8_t *body, size_t len, const char *extra)
{
    const char *reason =
        status == 200 ? "OK" :
        status == 400 ? "Bad Request" :
        status == 404 ? "Not Found" :
        status == 405 ? "Method Not Allowed" :
        status == 501 ? "Not Implemented" : "Error";
    char hdr[320];
    int hl = snprintf(hdr, sizeof hdr,
        "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\n"
        "Connection: close\r\nAccess-Control-Allow-Origin: *\r\n%s\r\n",
        status, reason, ctype, len, extra ? extra : "");
    write_all(fd, hdr, hl);
    if (body && len) write_all(fd, body, len);
}

static void send_json(int fd, int status, const char *json)
{
    send_http(fd, status, "application/json", (const uint8_t *)json, strlen(json), NULL);
}

/* value of query key, decimal or 0x-hex (strtol base 0); def if absent. */
static long query_long(const char *q, const char *key, long def)
{
    size_t kl = strlen(key);
    const char *p = q;
    while (p && *p) {
        if (!strncmp(p, key, kl) && p[kl] == '=')
            return strtol(p + kl + 1, NULL, 0);
        p = strchr(p, '&');
        if (p) p++;
    }
    return def;
}

/* copy the value of query key into out (NUL-terminated, truncated to cap).
 * Returns 1 if the key was present, 0 otherwise. */
static int hexval(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* PERCENT-DECODES, which it did not used to, and the omission was invisible
 * for exactly as long as every value a harness sent happened to be
 * alphanumeric. The first one that was not -- /key?text=%0D, a carriage
 * return, the single most useful key there is -- injected the literal '%'
 * (0x25) and answered {"injected":true}, so the harness recorded a RETURN it
 * had not sent and the machine sat there.
 *
 * A malformed escape ('%' not followed by two hex digits) is passed through
 * verbatim rather than dropped: that keeps a literal '%' in an un-encoded
 * value working, which is what a hand-typed curl is most likely to contain. */
static int query_str(const char *q, const char *key, char *out, size_t cap)
{
    size_t kl = strlen(key);
    const char *p = q;
    while (p && *p) {
        if (!strncmp(p, key, kl) && p[kl] == '=') {
            const char *v = p + kl + 1;
            const char *e = strchr(v, '&');
            size_t n = e ? (size_t)(e - v) : strlen(v);
            size_t w = 0;
            for (size_t i = 0; i < n && w + 1 < cap; i++) {
                int hi, lo;
                if (v[i] == '%' && i + 2 < n &&
                    (hi = hexval((unsigned char)v[i + 1])) >= 0 &&
                    (lo = hexval((unsigned char)v[i + 2])) >= 0) {
                    out[w++] = (char)((hi << 4) | lo);
                    i += 2;
                } else {
                    out[w++] = v[i];
                }
            }
            if (cap) out[w] = 0;
            return 1;
        }
        p = strchr(p, '&');
        if (p) p++;
    }
    if (cap) out[0] = 0;
    return 0;
}

/* marshal a read to the emulator thread, wait, send the response. */
static void do_marshalled(int fd, req_t r)
{
    pthread_mutex_lock(&g_lock);
    g_resp_ready = 0; g_resp_body = NULL; g_resp_len = 0;
    g_req = r;
    while (!g_resp_ready) pthread_cond_wait(&g_cv, &g_lock);
    uint8_t *body = g_resp_body; size_t len = g_resp_len;
    int st = g_resp_status; const char *ct = g_resp_ctype;
    char extra[64]; memcpy(extra, g_resp_extra, sizeof extra);
    g_resp_body = NULL;
    pthread_mutex_unlock(&g_lock);

    send_http(fd, st, ct, body, len, extra[0] ? extra : NULL);
    free(body);
}

static void do_status(int fd)
{
    pthread_mutex_lock(&g_lock);
    long f = g_frames_to_run;
    pthread_mutex_unlock(&g_lock);
    unsigned long long fc = g_be->get_frame_count ? g_be->get_frame_count() : 0;
    /* Advertise the highest level whose defining callbacks are all present. */
    const char *contract =
        (g_be->inject_key && g_be->write_mem && g_be->capture_audio &&
         g_be->set_pointer && g_be->get_pointer &&
         g_be->set_pad && g_be->get_pad) ? "0.5.0" :
        (g_be->inject_key && g_be->write_mem && g_be->capture_audio &&
         g_be->set_pointer && g_be->get_pointer) ? "0.4.0" :
        (g_be->inject_key && g_be->write_mem && g_be->capture_audio) ? "0.3.0" :
        g_be->inject_key ? "0.2.0" : "0.1.0";
    char buf[320];
    snprintf(buf, sizeof buf,
        "{\"contract\":\"%s\",\"emulator\":\"%s\",\"platform\":\"%s\","
        "\"frame\":%llu,\"paused\":%s,\"running\":true}",
        contract,
        g_be->emulator ? g_be->emulator : "",
        g_be->platform ? g_be->platform : "",
        fc, (f == 0) ? "true" : "false");
    send_json(fd, 200, buf);
}

static void do_step(int fd, const char *query)
{
    long n = query_long(query, "frames", 1);
    if (n < 1) n = 1;
    pthread_mutex_lock(&g_lock);
    g_frames_to_run = n;
    g_step_waiting = 1;
    g_resp_ready = 0;
    while (!g_resp_ready) pthread_cond_wait(&g_cv, &g_lock);
    pthread_mutex_unlock(&g_lock);

    unsigned long long fc = g_be->get_frame_count ? g_be->get_frame_count() : 0;
    char buf[64];
    snprintf(buf, sizeof buf, "{\"frame\":%llu}", fc);
    send_json(fd, 200, buf);
}

static void do_setrun(int fd, long v, int paused)
{
    pthread_mutex_lock(&g_lock);
    g_frames_to_run = v;
    pthread_mutex_unlock(&g_lock);
    send_json(fd, 200, paused ? "{\"paused\":true}" : "{\"paused\":false}");
}

/* Read a request body (POST /mem). `req`/`n` = what handle_conn already recv'd
 * (headers + maybe some body); reads the rest per Content-Length. Returns a
 * malloc'd buffer of *out_len bytes (caller frees), capped at MEM_CAP. */
static uint8_t *read_body(int fd, char *req, size_t n, size_t *out_len)
{
    *out_len = 0;
    char *hdr_end = strstr(req, "\r\n\r\n");
    if (!hdr_end) return NULL;
    char *body_start = hdr_end + 4;

    long clen = 0;
    char *cl = strstr(req, "Content-Length:");
    if (!cl) cl = strstr(req, "content-length:");
    if (cl) clen = strtol(cl + 15, NULL, 10);
    if (clen < 0) clen = 0;
    if (clen > (long)MEM_CAP) clen = (long)MEM_CAP;

    uint8_t *body = (uint8_t *)malloc((size_t)clen ? (size_t)clen : 1);
    if (!body) return NULL;

    size_t have = (size_t)(req + n - body_start);
    if (have > (size_t)clen) have = (size_t)clen;
    memcpy(body, body_start, have);
    size_t got = have;
    while (got < (size_t)clen) {
        ssize_t r = recv(fd, body + got, (size_t)clen - got, 0);
        if (r <= 0) break;
        got += (size_t)r;
    }
    *out_len = got;
    return body;
}

static void handle_conn(int fd)
{
    char req[2048];
    size_t n = 0;
    while (n < sizeof(req) - 1) {
        ssize_t r = recv(fd, req + n, sizeof(req) - 1 - n, 0);
        if (r <= 0) break;
        n += (size_t)r;
        if (strstr(req, "\r\n\r\n")) break;
    }
    req[n] = 0;

    char method[8] = {0}, target[1024] = {0};
    if (sscanf(req, "%7s %1023s", method, target) != 2) {
        send_json(fd, 400, "{\"error\":\"bad request\"}");
        return;
    }
    char *q = strchr(target, '?');
    const char *query = "";
    if (q) { *q = 0; query = q + 1; }
    int is_post = !strcmp(method, "POST");

    if (!strcmp(target, "/status"))          { do_status(fd); return; }
    if (!strcmp(target, "/regs"))            { do_marshalled(fd, REQ_REGS); return; }
    if (!strcmp(target, "/screenshot"))      { do_marshalled(fd, REQ_SHOT); return; }
    if (!strcmp(target, "/audio"))           { do_marshalled(fd, REQ_AUDIO); return; }
    if (!strcmp(target, "/mem")) {
        g_req_addr = (uint32_t)query_long(query, "addr", 0);
        g_req_bank = (int32_t)query_long(query, "bank", -1);
        if (is_post) {                        /* 0.3: write / poke */
            size_t blen = 0;
            uint8_t *body = read_body(fd, req, n, &blen);
            g_req_len = (uint32_t)blen;
            g_write_data = body;
            do_marshalled(fd, REQ_WRITE);
            free(body);
        } else {                              /* read */
            g_req_len = (uint32_t)query_long(query, "len", 0);
            do_marshalled(fd, REQ_MEM);
        }
        return;
    }
    if (!strcmp(target, "/step")) {
        if (!is_post) { send_json(fd, 405, "{\"error\":\"POST only\"}"); return; }
        do_step(fd, query); return;
    }
    if (!strcmp(target, "/pause")) {
        if (!is_post) { send_json(fd, 405, "{\"error\":\"POST only\"}"); return; }
        do_setrun(fd, 0, 1); return;
    }
    if (!strcmp(target, "/resume")) {
        if (!is_post) { send_json(fd, 405, "{\"error\":\"POST only\"}"); return; }
        do_setrun(fd, -1, 0); return;
    }
    if (!strcmp(target, "/key")) {
        if (!is_post) { send_json(fd, 405, "{\"error\":\"POST only\"}"); return; }
        char text[8];
        long code = query_long(query, "code", -1);
        long down = query_long(query, "down", -1);
        if (query_str(query, "text", text, sizeof text) && text[0]) {
            g_key_is_text = 1; g_key_value = (uint32_t)(unsigned char)text[0];
        } else if (code >= 0) {
            g_key_is_text = 0; g_key_value = (uint32_t)code;
        } else {
            send_json(fd, 400, "{\"error\":\"key needs text= or code=\"}"); return;
        }
        g_key_action = (down < 0) ? RETRO_KEY_TAP
                     : (down != 0) ? RETRO_KEY_DOWN : RETRO_KEY_UP;
        do_marshalled(fd, REQ_KEY);
        return;
    }
    if (!strcmp(target, "/jump")) {
        if (!is_post) { send_json(fd, 405, "{\"error\":\"POST only\"}"); return; }
        long a = query_long(query, "addr", -1);
        if (a < 0) { send_json(fd, 400, "{\"error\":\"jump needs addr=\"}"); return; }
        g_req_addr = (uint32_t)a;
        do_marshalled(fd, REQ_JUMP);
        return;
    }
    if (!strcmp(target, "/reset")) {
        if (!is_post) { send_json(fd, 405, "{\"error\":\"POST only\"}"); return; }
        do_marshalled(fd, REQ_RESET);
        return;
    }
    if (!strcmp(target, "/pointer")) {
        if (is_post) {                        /* 0.4: move/click the pointer */
            long x  = query_long(query, "x",  LONG_MIN);
            long y  = query_long(query, "y",  LONG_MIN);
            long dx = query_long(query, "dx", LONG_MIN);
            long dy = query_long(query, "dy", LONG_MIN);
            if (x != LONG_MIN && y != LONG_MIN) {
                g_ptr_absolute = 1; g_ptr_x = (int32_t)x; g_ptr_y = (int32_t)y;
            } else if (dx != LONG_MIN || dy != LONG_MIN) {
                g_ptr_absolute = 0;
                g_ptr_x = (int32_t)(dx == LONG_MIN ? 0 : dx);
                g_ptr_y = (int32_t)(dy == LONG_MIN ? 0 : dy);
            } else {
                send_json(fd, 400, "{\"error\":\"pointer needs x&y or dx&dy\"}"); return;
            }
            g_ptr_buttons = (int)query_long(query, "buttons", -1);   /* -1 = unchanged */
            do_marshalled(fd, REQ_PTR_SET);
        } else {                              /* 0.4: read back the pointer */
            do_marshalled(fd, REQ_PTR_GET);
        }
        return;
    }
    if (!strcmp(target, "/pad")) {
        g_pad_index = (int)query_long(query, "index", 0);
        if (is_post) {                        /* 0.5: present / set a pad */
            g_pad_buttons = (int)query_long(query, "buttons", -1);
            /* `connected` defaults to plugging it in whenever buttons were
             * given -- injecting a press on an absent pad is never what the
             * caller meant -- and to leaving it alone otherwise. */
            g_pad_connected = (int)query_long(query, "connected",
                                              g_pad_buttons >= 0 ? 1 : -1);
            do_marshalled(fd, REQ_PAD_SET);
        } else {                              /* 0.5: read a pad back */
            do_marshalled(fd, REQ_PAD_GET);
        }
        return;
    }
    send_json(fd, 404, "{\"error\":\"not found\"}");
}

static void *server_main(void *arg)
{
    (void)arg;
    for (;;) {
        int fd = accept(g_listen_fd, NULL, NULL);
        if (fd < 0) {
            if (errno == EINTR) continue;
            break;
        }
        handle_conn(fd);
        close(fd);
    }
    return NULL;
}

int retro_control_start(int port, const retro_control_backend_t *backend)
{
    g_be = backend;
    signal(SIGPIPE, SIG_IGN);

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof addr);
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   /* 127.0.0.1 only */
    addr.sin_port = htons((uint16_t)port);
    if (bind(fd, (struct sockaddr *)&addr, sizeof addr) < 0) {
        fprintf(stderr, "[retro_control] bind 127.0.0.1:%d failed: %s\n",
                port, strerror(errno));
        close(fd);
        return -1;
    }
    if (listen(fd, 4) < 0) { close(fd); return -1; }

    g_listen_fd = fd;
    if (pthread_create(&g_thread, NULL, server_main, NULL) != 0) {
        close(fd); g_listen_fd = -1; return -1;
    }
    pthread_detach(g_thread);
    fprintf(stderr, "[retro_control] listening on 127.0.0.1:%d (contract %s)\n",
            port, RETRO_CONTROL_CONTRACT);
    return 0;
}

void retro_control_stop(void)
{
    if (g_listen_fd >= 0) { close(g_listen_fd); g_listen_fd = -1; }
}

#endif /* !_WIN32 */
