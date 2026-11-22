// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Stephane Humbert, Aaron Giles, AJR, Couriersud
//
// pm.c -- the Namco Pac-Man board as MAME 0.288 emulates it, in C for the
// cart:
//
//   pacman.cpp, pacman_v.cpp (Nicola Salmoria): the memory map, inputs,
//     interrupt, palette, tilemap and sprites
//   namco.cpp (Nicola Salmoria, Aaron Giles): the 3-voice WSG
//   74259.cpp (AJR), watchdog.cpp (Aaron Giles): the latch and watchdog
//   resnet.cpp (Couriersud): the palette's resistor weights
//   schedule.cpp, diexec.cpp, screen.cpp, sound.cpp (Aaron Giles and
//     others): MAME's timeslices, timers, VBLANK, input line events and
//     its stream updates
//
// The Z80 runs in MAME's timeslices, up to the next timer, and a write that
// changes the interrupt line ends its timeslice as it does in MAME, so the
// CPU sees VBLANK on the same cycle. MAME's license (BSD-3-Clause) is in
// README.md.
//
// Built with PM_MS 1, it is Ms. Pac-Man's board (pacman.cpp's mspacman):
// the same, with Midway's aux board on it (below, "the aux board").
#include "pm.h"

#include <string.h>

#include "z80.h"

#if defined(PICO_ON_DEVICE) && PICO_ON_DEVICE
#define PM_HOT __attribute__((section(".time_critical.pm")))
#else
#define PM_HOT
#endif

// ---------------------------------------------------------------------------
// attotime (src/emu/attotime.h)

typedef struct { int32_t s; int64_t a; } at_t;
#define AS_PER_SEC 1000000000000000000LL
#define AT_NEVER_S 1000000000
static const at_t at_zero = { 0, 0 };
static const at_t at_never = { AT_NEVER_S, 0 };

static inline bool at_is_never(at_t t) { return t.s >= AT_NEVER_S; }
static inline at_t at_add(at_t x, at_t y) {
	if (at_is_never(x) || at_is_never(y)) return at_never;
	at_t r = { x.s + y.s, x.a + y.a };
	if (r.a >= AS_PER_SEC) { r.a -= AS_PER_SEC; r.s++; }
	if (r.s >= AT_NEVER_S) return at_never;
	return r;
}
static inline at_t at_sub(at_t x, at_t y) {
	if (at_is_never(x)) return at_never;
	at_t r = { x.s - y.s, x.a - y.a };
	if (r.a < 0) { r.a += AS_PER_SEC; r.s--; }
	return r;
}
static inline int at_cmp(at_t x, at_t y) {
	if (x.s != y.s) return x.s < y.s ? -1 : 1;
	if (x.a != y.a) return x.a < y.a ? -1 : 1;
	return 0;
}
static inline at_t at_as(int64_t a) { at_t r = { 0, a }; return r; }
// floor(a * f / 1e18) for 0 <= a < 1e18, without 128-bit sums
static inline uint64_t mul_frac(int64_t a, uint32_t f) {
	uint64_t ah = (uint64_t)a / 1000000000u, al = (uint64_t)a % 1000000000u;
	uint64_t x = ah * f, y = al * f;
	uint64_t q1 = x / 1000000000u, r1 = x % 1000000000u;
	return q1 + (r1 * 1000000000u + y) / (uint64_t)AS_PER_SEC;
}
static inline uint64_t at_ticks(at_t t, uint32_t f) { return (uint64_t)t.s * f + mul_frac(t.a, f); }

// ---------------------------------------------------------------------------
// the machine's constants

#define PIX_AS (AS_PER_SEC / 6144000)   // HZ_TO_ATTOSECONDS(XTAL(18'432'000) / 3)
#define HTOTAL 384
#define VTOTAL 264
#define VBSTART 224
#define SCAN_AS (PIX_AS * HTOTAL)
#define FRAME_AS (SCAN_AS * VTOTAL)
#define VBLANK_AS (SCAN_AS * (VTOTAL - VBSTART))

static const pm_roms_t *roms;
pm_inputs_t pm_in = { PM_IN0, PM_IN1, PM_DSW1, PM_DSW2 };
uint8_t pm_vram[0x400], pm_cram[0x400], pm_ram[0x400], pm_spr2[0x10];
uint8_t pm_latch;
uint32_t pm_frames, pm_resets;
void (*pm_on_vblank)(void);

static bool irq_mask;
static uint8_t irq_vector;
static int watchdog_counter;
static bool watchdog_fired;
static uint8_t sink[0x100];             // writes that go nowhere
static uint8_t nop_read[0x100];         // 0x4800-0x4bff: 0xBF (pacman_read_nop)

// ---------------------------------------------------------------------------
// the scheduler (src/emu/schedule.cpp, src/emu/diexec.cpp), for one CPU

typedef struct {
	at_t local;                         // m_localtime
	int64_t aspc;                       // m_attoseconds_per_cycle
	uint32_t cps;                       // m_cycles_per_second
	int divshift;
	uint32_t divisor;
	uint64_t total;                     // m_totalcycles
} exec_t;

static exec_t ex;
static bool executing;

enum {
	TM_VBSTART, TM_VBEND, TM_SCAN0, TM_SND50,
	TM_SYNC,                            // temporary timers (MAME's synchronize()) from here
	TM_COUNT = TM_SYNC + 8
};
enum { SY_IRQ };

typedef struct {
	at_t expire, period;
	uint32_t seq;
	bool enabled, used;
	int type, param;
} tmr_t;

static tmr_t tm[TM_COUNT];
static uint32_t tm_seq;
static at_t basetime;
static bool in_callback;
static int cb_timer;
static at_t cb_expire;
static bool cb_modified;
static const int64_t quantum = AS_PER_SEC / 60; // the 60 Hz default quantum

static void exec_init(exec_t *e, uint32_t clock) {
	e->local = at_zero;
	e->cps = clock;
	e->aspc = AS_PER_SEC / clock;
	int64_t attos = e->aspc;
	e->divshift = 0;
	while (attos >= (1LL << 31)) { e->divshift++; attos >>= 1; }
	e->divisor = (uint32_t)attos;
	e->total = 0;
}

static inline at_t cycles_to_at(const exec_t *e, uint64_t c) {
	if (c < e->cps) return at_as((int64_t)c * e->aspc);
	at_t r = { (int32_t)(c / e->cps), (int64_t)(c % e->cps) * e->aspc };
	return r;
}

// machine().time(): the timer's time in a callback, the CPU's while it runs, else the base
static at_t now(void) {
	if (in_callback) return cb_expire;
	if (executing) return at_add(ex.local, cycles_to_at(&ex, (uint64_t)z80_done()));
	return basetime;
}

static int head(void) {
	int h = -1;
	for (int i = 0; i < TM_COUNT; i++) {
		if (!tm[i].used || !tm[i].enabled) continue;
		if (h < 0) { h = i; continue; }
		int c = at_cmp(tm[i].expire, tm[h].expire);
		if (c < 0 || (c == 0 && tm[i].seq < tm[h].seq)) h = i;
	}
	return h;
}

static at_t head_expire(void) {
	int h = head();
	return h < 0 ? at_never : tm[h].expire;
}

static void tm_insert(int i) {
	tm[i].seq = ++tm_seq;
	if (in_callback && i == cb_timer) cb_modified = true;
	if (head() == i && executing) z80_abort();
}

static void tm_adjust(int i, at_t delay, int param) {
	tmr_t *t = &tm[i];
	t->used = true;
	t->enabled = true;
	t->param = param;
	t->period = at_never;
	t->expire = at_add(now(), delay);
	if (at_is_never(t->expire)) t->enabled = false;
	tm_insert(i);
}

static void synchronize(int type, int param) {
	for (int i = TM_SYNC; i < TM_COUNT; i++) {
		if (tm[i].used) continue;
		tm[i].type = type;
		tm_adjust(i, at_zero, param);
		return;
	}
}

// the IRQ line's event queue (device_input::set_state_synced): the first
// event of a batch sets a timer for now, which applies them all
static uint8_t irq_queue[32];
static int irq_qn;

static void set_irq_line(int state) {
	if (irq_qn == (int)sizeof irq_queue) {          // full: applied now (MAME logs it)
		for (int i = 0; i < irq_qn; i++) z80_set_irq(irq_queue[i]);
		irq_qn = 0;
	}
	irq_queue[irq_qn++] = (uint8_t)state;
	if (irq_qn == 1) synchronize(SY_IRQ, 0);
}

static void irq_events(void) {
	for (int i = 0; i < irq_qn; i++) z80_set_irq(irq_queue[i]);
	irq_qn = 0;
}

// ---------------------------------------------------------------------------
// the WSG (namco_wsg_device): its stream at 192 kHz, summed into bins

static struct { uint32_t freq, counter; uint8_t vol, sel; } voice[3];
static uint8_t soundregs[0x20];
static bool sound_enable;
static uint64_t wsg_next;               // the stream's next sample (its write position)
static int8_t wave[8][32];              // the 82s126's low nibble less 8, per waveform

#ifndef PM_NBIN
#define PM_NBIN 2048                    // the bins the sound can be ahead (the cart needs few)
#endif
#define NBIN PM_NBIN
static int32_t bin_sum[NBIN];
static uint16_t bin_n[NBIN];
static uint32_t bin_w, bin_r;
static uint64_t bin_step_fx;            // stream samples per bin, 32.32
static uint32_t bin_frac;               // where the current bin started, the fraction
static uint32_t bin_len, bin_have;      // the current bin's length, and its samples so far
static int32_t bin_acc;

static void bin_start(void) {
	uint64_t e = (uint64_t)bin_frac + bin_step_fx;
	bin_len = (uint32_t)(e >> 32);
	bin_frac = (uint32_t)e;
	bin_have = 0;
	bin_acc = 0;
}

void pm_audio_rate_q8(uint32_t hz_q8) {
	bin_step_fx = (((uint64_t)PM_WSG_RATE << 40) / hz_q8);
	bin_frac = 0;
	bin_start();
}

static PM_HOT void wsg_samples(uint32_t n) {
	while (n) {
		uint32_t m = bin_len - bin_have;
		if (m > n) m = n;
		if (sound_enable) {
			for (int v = 0; v < 3; v++) {
				if (!voice[v].vol) continue;
				const int8_t *w = wave[voice[v].sel];
				uint32_t c = voice[v].counter, f = voice[v].freq;
				int32_t s = 0;
				for (uint32_t i = 0; i < m; i++) {
					s += w[(c >> 16) & 31];
					c += f;
				}
				voice[v].counter = c;
				bin_acc += s * voice[v].vol;
			}
		}
		bin_have += m;
		n -= m;
		if (bin_have == bin_len) {
			if (bin_w - bin_r < NBIN) {
				bin_sum[bin_w % NBIN] = bin_acc;
				bin_n[bin_w % NBIN] = (uint16_t)bin_len;
				bin_w++;
			}
			bin_start();
		}
	}
}

// sound_stream::update(): the samples up to now, now's included
static void wsg_update(void) {
	at_t t = now();
	if (t.s == 0 && t.a == 0) return;
	uint64_t idx = at_ticks(t, PM_WSG_RATE);
	if (idx + 1 > wsg_next) {
		wsg_samples((uint32_t)(idx + 1 - wsg_next));
		wsg_next = idx + 1;
	}
}

static void wsg_write(int offset, uint8_t data) {       // pacman_sound_w
	data &= 0x0f;
	if (soundregs[offset] == data) return;
	wsg_update();
	soundregs[offset] = data;
	int ch;
	if (offset < 0x10) ch = (offset - 5) / 5;
	else if (offset == 0x10) ch = 0;
	else ch = (offset - 0x11) / 5;
	if (ch < 0 || ch >= 3) return;      // (C's division truncates as C++'s does: offsets 1-4 give 0)
	switch (offset - ch * 5) {
	case 0x05:
		voice[ch].sel = data & 7;
		break;
	case 0x10: case 0x11: case 0x12: case 0x13: case 0x14:
		voice[ch].freq = ch == 0 ? soundregs[0x10] : 0;
		voice[ch].freq += (uint32_t)soundregs[ch * 5 + 0x11] << 4;
		voice[ch].freq += (uint32_t)soundregs[ch * 5 + 0x12] << 8;
		voice[ch].freq += (uint32_t)soundregs[ch * 5 + 0x13] << 12;
		voice[ch].freq += (uint32_t)soundregs[ch * 5 + 0x14] << 16;
		break;
	case 0x15:
		voice[ch].vol = data;
		break;
	}
}

int pm_audio_take(int32_t *sum, uint16_t *count, int max) {
	int k = 0;
	while (k < max && bin_r != bin_w) {
		sum[k] = bin_sum[bin_r % NBIN];
		count[k] = bin_n[bin_r % NBIN];
		bin_r++;
		k++;
	}
	return k;
}

#if PM_MS
// ---------------------------------------------------------------------------
// the aux board (pacman.cpp's mspacman_map, init_mspacman,
// mspacman_install_patches and the decode latch's handlers)
//
// Two banks of ROM at 0x0000-0x3fff and 0x8000-0xbfff. Bank 0 is Pac-Man's
// four ROMs, again at 0x8000. Bank 1, the decoded one, is 6e, 6f and 6h
// with forty 8-byte patches from u5, then u7 at 0x3000, u5 and u6 at
// 0x8000, 6f's top half again at 0x9800 and 6h and 6j at 0xa000. A read
// or write of 3ff8-3fff turns to bank 1, of 0038-003f, 03b0-03b7,
// 1600-1607, 2120-2127, 3ff0-3ff7, 8000-8007 or 97f0-97f7 to bank 0, and
// those places read bank 1 (3ff8) or bank 0 (the rest) whichever is in.
// Their pages go through pm_rd and pm_wr; the others are the bank's.
// The board starts in bank 1 (set_entry(1) in the driver's init), and a
// reset leaves the latch as it is.

#define MS_PATCHED 21                   // the pages of 0x0000-0x2fff the patches are in
static uint8_t ms_pages[MS_PATCHED][256];
static const uint8_t *ms_bank[2][128];  // each bank's pages: 0x00-0x3f, then 0x80-0xbf
static uint8_t ms_diff[128];            // the pages the banks differ in (not the latch's)
static int ms_ndiff;
static int ms_cur;                      // the bank in
static uint8_t coin_lock;               // 0x60 (both coins) while locked out, as at the last VBLANK
uint32_t pm_bank_switches;

// mspacman_install_patches: 8 bytes from the decoded u5 (at 0x8000) to each
static const uint16_t ms_patch[40][2] = {
	{0x0410, 0x8008}, {0x08e0, 0x81d8}, {0x0a30, 0x8118}, {0x0bd0, 0x80d8}, {0x0c20, 0x8120},
	{0x0e58, 0x8168}, {0x0ea8, 0x8198}, {0x1000, 0x8020}, {0x1008, 0x8010}, {0x1288, 0x8098},
	{0x1348, 0x8048}, {0x1688, 0x8088}, {0x16b0, 0x8188}, {0x16d8, 0x80c8}, {0x16f8, 0x81c8},
	{0x19a8, 0x80a8}, {0x19b8, 0x81a8}, {0x2060, 0x8148}, {0x2108, 0x8018}, {0x21a0, 0x81a0},
	{0x2298, 0x80a0}, {0x23e0, 0x80e8}, {0x2418, 0x8000}, {0x2448, 0x8058}, {0x2470, 0x8140},
	{0x2488, 0x8080}, {0x24b0, 0x8180}, {0x24d8, 0x80c0}, {0x24f8, 0x81c0}, {0x2748, 0x8050},
	{0x2780, 0x8090}, {0x27b8, 0x8190}, {0x2800, 0x8028}, {0x2b20, 0x8100}, {0x2b30, 0x8110},
	{0x2bf0, 0x81d0}, {0x2cc0, 0x80d0}, {0x2cd8, 0x80e0}, {0x2cf0, 0x81e0}, {0x2d60, 0x8160},
};

// MAME's bitswap<N>(v, ...): bit i of the result is bit from[N - 1 - i] of v
static unsigned bitswap(unsigned v, const uint8_t *from, int n) {
	unsigned r = 0;
	for (int i = 0; i < n; i++) r |= ((v >> from[i]) & 1u) << (n - 1 - i);
	return r;
}

static uint8_t ms_data(uint8_t d) {     // bitswap<8>(d, 0, 4, 5, 7, 6, 3, 2, 1)
	static const uint8_t from[8] = { 0, 4, 5, 7, 6, 3, 2, 1 };
	return (uint8_t)bitswap(d, from, 8);
}

void pm_aux_decode(uint8_t *aux) {      // init_mspacman's loops (ms_pages lends its room)
	static const uint8_t a5[11] = { 8, 7, 5, 9, 10, 6, 3, 4, 2, 1, 0 };     // u5's address lines
	static const uint8_t a6[11] = { 3, 7, 9, 10, 8, 6, 5, 4, 2, 1, 0 };     // u6's, and u7's below A11
	uint8_t *t = &ms_pages[0][0];
	memcpy(t, aux + 0x1800, 0x1000);    // u7: the bank's 0x3000-0x3fff
	for (unsigned i = 0; i < 0x1000; i++) aux[0x1800 + i] = ms_data(t[(i & 0x800) | bitswap(i & 0x7ff, a6, 11)]);
	memcpy(t, aux + 0x800, 0x1000);     // u6: its top half to 0x8800, its bottom to 0x9000
	for (unsigned i = 0; i < 0x800; i++) {
		aux[0x800 + i] = ms_data(t[0x800 + bitswap(i, a6, 11)]);
		aux[0x1000 + i] = ms_data(t[bitswap(i, a6, 11)]);
	}
	memcpy(t, aux, 0x800);              // u5: 0x8000
	for (unsigned i = 0; i < 0x800; i++) aux[i] = ms_data(t[bitswap(i, a5, 11)]);
}

// the page index of 0x0000-0x3fff or 0x8000-0xbfff
static inline int ms_k(uint16_t a) { return (a >> 8 & 0x3f) | (a >> 9 & 0x40); }

// which bank an address turns the latch to (-1: neither)
static inline int ms_latch_at(uint16_t a) {
	switch (a & 0xfff8) {
	case 0x3ff8: return 1;
	case 0x0038: case 0x03b0: case 0x1600: case 0x2120: case 0x3ff0: case 0x8000: case 0x97f0: return 0;
	default: return -1;
	}
}

static bool ms_latch_page(int k) {
	static const uint8_t pages[] = { 0x00, 0x03, 0x16, 0x21, 0x3f, 0x40, 0x57 };
	for (unsigned i = 0; i < sizeof pages; i++)
		if (pages[i] == k) return true;
	return false;
}

static void ms_set(int b) {             // membank("bank1")->set_entry(b)
	if (b == ms_cur) return;
	ms_cur = b;
	pm_bank_switches++;
	for (int i = 0; i < ms_ndiff; i++) {
		int k = ms_diff[i];
		z80_rd_page[k < 0x40 ? k : k + 0x40] = ms_bank[b][k];
	}
}

static uint8_t ms_rd(uint16_t a) {
	int t = ms_latch_at(a);
	if (t < 0) return ms_bank[ms_cur][ms_k(a)][a & 0xff];
	ms_set(t);
	return ms_bank[t][ms_k(a)][a & 0xff];
}

static void ms_wr(uint16_t a) {
	int t = ms_latch_at(a);
	if (t >= 0) ms_set(t);
}

uint8_t pm_bank_peek(int bank, uint16_t a) { return ms_bank[bank & 1][ms_k(a)][a & 0xff]; }

static void ms_init(const pm_roms_t *r) {
	const uint8_t *rom = r->maincpu, *aux = r->aux;
	for (int k = 0; k < 128; k++) ms_bank[0][k] = rom + ((k & 0x3f) << 8);
	uint64_t patched = 0;
	for (int i = 0; i < 40; i++) patched |= 1ull << (ms_patch[i][0] >> 8);
	int n = 0;
	for (int p = 0; p < 0x30; p++) {
		ms_bank[1][p] = rom + (p << 8);
		if (!(patched >> p & 1) || n == MS_PATCHED) continue;
		memcpy(ms_pages[n], rom + (p << 8), 256);
		for (int i = 0; i < 40; i++)
			if (ms_patch[i][0] >> 8 == p) memcpy(&ms_pages[n][ms_patch[i][0] & 0xff], aux + (ms_patch[i][1] - 0x8000), 8);
		ms_bank[1][p] = ms_pages[n++];
	}
	for (int p = 0x30; p < 0x40; p++) ms_bank[1][p] = aux + 0x1800 + ((p - 0x30) << 8);    // u7
	for (int k = 0x40; k < 0x58; k++) ms_bank[1][k] = aux + ((k - 0x40) << 8);              // u5, u6
	for (int k = 0x58; k < 0x80; k++) ms_bank[1][k] = rom + ((k - 0x40) << 8);              // 6f's top, 6h, 6j
	ms_ndiff = 0;
	for (int k = 0; k < 128; k++)
		if (ms_bank[0][k] != ms_bank[1][k] && !ms_latch_page(k)) ms_diff[ms_ndiff++] = (uint8_t)k;
	ms_cur = 1;
	coin_lock = 0x60;
	pm_bank_switches = 0;
}
#endif

// ---------------------------------------------------------------------------
// the latch, the watchdog, memory

static void latch_bit(int bit, int state) {             // ls259_device::write_bit
	if (((pm_latch >> bit) & 1) == state) return;
	pm_latch ^= (uint8_t)(1u << bit);
	switch (bit) {
	case 0:                                             // irq_mask_w
		irq_mask = state;
		if (!state) set_irq_line(0);
		break;
	case 1:                                             // namco sound_enable_w (no stream update)
		sound_enable = state;
		break;
	default:                                            // flip, coin counter, unconnected
		break;
	}
}

uint8_t pm_rd(uint16_t a) {             // 0x5000-0x5fff and its mirrors
#if PM_MS
	if (!(a & 0x4000)) return ms_rd(a);                 // (and the aux board's latch)
	switch ((a >> 6) & 3) {
	case 0: return pm_in.in0 | coin_lock;
#else
	switch ((a >> 6) & 3) {
	case 0: return pm_in.in0;
#endif
	case 1: return pm_in.in1;
	case 2: return pm_in.dsw1;
	default: return pm_in.dsw2;
	}
}

void pm_wr(uint16_t a, uint8_t d) {
#if PM_MS
	if (!(a & 0x4000)) { ms_wr(a); return; }
#endif
	uint8_t o = (uint8_t)a;
	if (o < 0x40) latch_bit(o & 7, d & 1);
	else if (o < 0x60) wsg_write(o - 0x40, d);
	else if (o < 0x70) pm_spr2[o & 0x0f] = d;
	else if (o >= 0xc0) watchdog_counter = 16;          // watchdog reset_w
}

uint8_t pm_io_in(uint16_t port) { (void)port; return 0xff; }  // (no I/O read is mapped: not reached by Pac-Man)
void pm_io_out(uint16_t port, uint8_t d) { if ((port & 0xff) == 0) irq_vector = d; }
uint8_t pm_irq_ack(void) { return irq_vector; }

static void map_pages(void) {
	for (int p = 0; p < 256; p++) {
		z80_rd_page[p] = 0;
		z80_wr_page[p] = sink;
		int m = p & 0x7f;                               // A15 isn't decoded
#if PM_MS
		if (m < 0x40) {                                 // (it is on the aux board: the banks)
			int k = p < 0x40 ? p : p - 0x40;
			if (ms_latch_page(k)) z80_wr_page[p] = 0;   // pm_rd, pm_wr
			else z80_rd_page[p] = ms_bank[ms_cur][k];
			continue;
		}
#else
		if (m < 0x40) { z80_rd_page[p] = roms->maincpu + (m << 8); continue; }
#endif
		m &= 0x5f;                                      // nor is A13 above 0x4000
		if (m < 0x44) z80_rd_page[p] = z80_wr_page[p] = pm_vram + ((m & 3) << 8);
		else if (m < 0x48) z80_rd_page[p] = z80_wr_page[p] = pm_cram + ((m & 3) << 8);
		else if (m < 0x4c) z80_rd_page[p] = nop_read;
		else if (m < 0x50) z80_rd_page[p] = z80_wr_page[p] = pm_ram + ((m & 3) << 8);
		else z80_wr_page[p] = 0;                        // 0x5000-0x5fff: pm_rd, pm_wr
	}
}

uint8_t pm_peek(uint16_t a) {
	const uint8_t *p = z80_rd_page[a >> 8];
#if PM_MS
	if (!p && !(a & 0x4000)) {                          // (side_effects_disabled: the latch stays)
		int t = ms_latch_at(a);
		return ms_bank[t < 0 ? ms_cur : t][ms_k(a)][a & 0xff];
	}
#endif
	return p ? p[a & 0xff] : pm_rd(a);
}

// ---------------------------------------------------------------------------
// video (pacman_v.cpp)

uint32_t pm_rgb[16];
uint16_t pm_tile_rows[256][8];
uint32_t pm_sprite_rows[64][16];
uint8_t pm_clut[128];
static uint8_t sprite_opaque[32];       // per color: bit p set if pen p isn't see-through

static void video_init(void) {
	static const uint8_t l3[8] = { 0, 33, 71, 104, 151, 184, 222, 255 };   // resistor weights 1000, 470, 220
	static const uint8_t l2[4] = { 0, 81, 174, 255 };                       // 470, 220
	const uint8_t *prom = roms->proms;
	for (int i = 0; i < 16; i++) {
		uint8_t c = prom[i];
		pm_rgb[i] = (uint32_t)l3[c & 7] << 16 | (uint32_t)l3[(c >> 3) & 7] << 8 | l2[c >> 6];
	}
	for (int i = 0; i < 128; i++) pm_clut[i] = prom[0x20 + i] & 0x0f;
	for (int c = 0; c < 32; c++) {
		sprite_opaque[c] = 0;
		for (int p = 0; p < 4; p++)
			if (pm_clut[c * 4 + p] != 0) sprite_opaque[c] |= (uint8_t)(1u << p);
	}
	// tilelayout: planes at bits 0 and 4, x 64-67 then 0-3, y in bytes, 16 bytes a tile
	const uint8_t *g = roms->gfx;
	for (int t = 0; t < 256; t++)
		for (int y = 0; y < 8; y++) {
			uint16_t row = 0;
			for (int x = 0; x < 8; x++) {
				int bit = (x < 4 ? 64 + x : x - 4) + y * 8;
				uint8_t b = g[t * 16 + (bit >> 3)];
				int sh = bit & 7;
				int pen = ((b >> (7 - sh)) & 1) << 1 | ((b >> (3 - sh)) & 1);
				row |= (uint16_t)(pen << (2 * x));
			}
			pm_tile_rows[t][y] = row;
		}
	// spritelayout: x 64-67, 128-131, 192-195, 0-3; y 0-7 then 32-39 rows; 64 bytes a sprite
	static const int sx[16] = { 64, 65, 66, 67, 128, 129, 130, 131, 192, 193, 194, 195, 0, 1, 2, 3 };
	for (int s = 0; s < 64; s++)
		for (int y = 0; y < 16; y++) {
			uint32_t row = 0;
			int yo = (y < 8 ? y : y + 24) * 8;
			for (int x = 0; x < 16; x++) {
				int bit = sx[x] + yo;
				uint8_t b = g[0x1000 + s * 64 + (bit >> 3)];
				int sh = bit & 7;
				int pen = ((b >> (7 - sh)) & 1) << 1 | ((b >> (3 - sh)) & 1);
				row |= (uint32_t)pen << (2 * x);
			}
			pm_sprite_rows[s][y] = row;
		}
	memset(nop_read, 0xbf, sizeof nop_read);
}

// the tilemap's memory index of raw tile column col (0-35), row (0-27): pacman_scan_rows
static inline int tile_index(int col, int row) {
	row += 2;
	col -= 2;
	if (col & 0x20) return row + ((col & 0x1f) << 5);
	return col + (row << 5);
}

static void draw_sprite(uint8_t fb[PM_H][PM_W], int code, int color, int fx, int fy, int sx, int sy) {
	const uint32_t *rows = pm_sprite_rows[code & 63];
	const uint8_t *clut = pm_clut + (color & 0x1f) * 4;
	uint8_t opaque = sprite_opaque[color & 0x1f];
	for (int y = 0; y < 16; y++) {
		int py = sy + y;
		if (py < 0 || py > 28 * 8 - 1) continue;        // spriteclip: y 0-223
		uint32_t row = rows[fy ? 15 - y : y];
		for (int x = 0; x < 16; x++) {
			int px = sx + x;
			if (px < 2 * 8 || px > 34 * 8 - 1) continue; // x 16-271
			int pen = (row >> (2 * (fx ? 15 - x : x))) & 3;
			if (opaque & (1u << pen)) fb[py][px] = clut[pen];
		}
	}
}

void pm_render(uint8_t fb[PM_H][PM_W]) {            // screen_update_pacman (bgpriority 0)
	bool flip = (pm_latch >> 3) & 1;
	for (int y = 0; y < PM_H; y++)
		for (int x = 0; x < PM_W; x++) {
			int tx = flip ? PM_W - 1 - x : x, ty = flip ? PM_H - 1 - y : y;
			int i = tile_index(tx >> 3, ty >> 3);
			int pen = (pm_tile_rows[pm_vram[i]][ty & 7] >> (2 * (tx & 7))) & 3;
			fb[y][x] = pm_clut[(pm_cram[i] & 0x1f) * 4 + pen];
		}
	for (int offs = 14; offs > 4; offs -= 2) {
		int sx = 272 - pm_spr2[offs + 1], sy = pm_spr2[offs] - 31;
		int code = pm_spr[offs] >> 2, color = pm_spr[offs + 1] & 0x1f;
		draw_sprite(fb, code, color, pm_spr[offs] & 1, pm_spr[offs] & 2, sx, sy);
		draw_sprite(fb, code, color, pm_spr[offs] & 1, pm_spr[offs] & 2, sx - 256, sy);
	}
	for (int offs = 4; offs >= 0; offs -= 2) {
		int sx = 272 - pm_spr2[offs + 1], sy = pm_spr2[offs] - 31;
		int code = pm_spr[offs] >> 2, color = pm_spr[offs + 1] & 0x1f;
		draw_sprite(fb, code, color, pm_spr[offs] & 1, pm_spr[offs] & 2, sx, sy + 1);
		draw_sprite(fb, code, color, pm_spr[offs] & 1, pm_spr[offs] & 2, sx - 256, sy + 1);
	}
}

// ---------------------------------------------------------------------------
// timers

static bool frame_done;
static at_t vblank_start_time;

static at_t time_until_pos(int v) {
	v += VTOTAL - VBSTART;
	v %= VTOTAL;
	int64_t target = (int64_t)v * SCAN_AS;
	int64_t cur = at_sub(now(), vblank_start_time).a;
	if (target <= cur + PIX_AS / 2) target += FRAME_AS;
	while (target <= cur) target += FRAME_AS;
	return at_as(target - cur);
}

static void cb_vbstart(void) {
	vblank_start_time = now();
#if PM_MS
	// the inputs are read at the frame's update (ioport's frame_update), and a
	// coin is let through only while the latch's Q6 is high (coin_lockout_global_w(!state))
	coin_lock = (pm_latch & 0x40) ? 0 : 0x60;
#endif
	pm_frames++;                        // video().frame_update(): the picture is RAM as it is now
	frame_done = true;
	if (pm_on_vblank) pm_on_vblank();
	if (--watchdog_counter == 0) watchdog_fired = true;  // the screen's VBLANK callbacks
	if (irq_mask) set_irq_line(1);      // screen_vblank -> vblank_irq
	tm_adjust(TM_VBSTART, time_until_pos(VBSTART), 0);
	tm_adjust(TM_VBEND, at_sub(at_add(vblank_start_time, at_as(VBLANK_AS)), now()), 0);
}

static void execute_timers(void) {
	for (;;) {
		int h = head();
		if (h < 0 || at_cmp(tm[h].expire, basetime) > 0) break;
		tmr_t *t = &tm[h];
		bool was_enabled = t->enabled;
		if (at_is_never(t->period)) t->enabled = false;
		in_callback = true;
		cb_timer = h;
		cb_expire = t->expire;
		cb_modified = false;
		if (was_enabled) {
			switch (h) {
			case TM_VBSTART: cb_vbstart(); break;
			case TM_VBEND: break;
			case TM_SCAN0: tm_adjust(TM_SCAN0, time_until_pos(0), 0); break;
			case TM_SND50: wsg_update(); break;     // the sound manager's 50 Hz streams update
			default: if (t->type == SY_IRQ) irq_events(); break;
			}
		}
		if (!cb_modified) {
			if (h >= TM_SYNC) t->used = false;
			else if (!at_is_never(t->period)) {
				t->expire = at_add(t->expire, t->period);
				t->enabled = true;
				t->seq = ++tm_seq;
			}
		}
		in_callback = false;
	}
}

static PM_HOT void timeslice(void) {
	while (at_cmp(basetime, head_expire()) < 0) {
		at_t target = at_add(basetime, at_as(quantum));
		at_t he = head_expire();
		if (at_cmp(he, target) < 0) target = he;
		if (target.s >= ex.local.s) {
			int64_t delta = target.a - ex.local.a;
			if (delta < 0 && target.s > ex.local.s) delta += AS_PER_SEC;
			if (delta >= ex.aspc) {
				int cycles = (int)(((uint64_t)delta >> ex.divshift) / ex.divisor);
				executing = true;
				int ran = z80_run(cycles);
				executing = false;
				ex.total += (uint64_t)ran;
				ex.local = at_add(ex.local, cycles_to_at(&ex, (uint64_t)ran));
				if (at_cmp(ex.local, target) < 0)
					target = at_cmp(ex.local, basetime) > 0 ? ex.local : basetime;
			}
		}
		basetime = target;
	}
	execute_timers();
}

// ---------------------------------------------------------------------------

uint64_t pm_cycles(void) { return ex.total + (executing ? (uint64_t)z80_done() : 0); }
uint64_t pm_time(void) { return at_ticks(basetime, PM_CLOCK); }

void pm_wsg_voice(int v, uint32_t *freq, uint8_t *vol, uint8_t *wave) {
	*freq = voice[v].freq;
	*vol = voice[v].vol;
	*wave = voice[v].sel;
}

const int8_t *pm_wsg_wave(int w) { return wave[w & 7]; }

static void board_reset(void) {         // device_reset of the CPU, the latch and the watchdog
	z80_reset();
	for (int b = 0; b < 8; b++) latch_bit(b, 0);        // ls259: clear_outputs(0)
	watchdog_counter = 16;
}

void pm_init(const pm_roms_t *r) {
	roms = r;
	memset(tm, 0, sizeof tm);
	tm_seq = 0;
	basetime = at_zero;
	executing = false;
	in_callback = false;
	exec_init(&ex, PM_CLOCK);
	memset(pm_vram, 0, sizeof pm_vram);
	memset(pm_cram, 0, sizeof pm_cram);
	memset(pm_ram, 0, sizeof pm_ram);
	memset(pm_spr2, 0, sizeof pm_spr2);
	pm_frames = pm_resets = 0;
	irq_mask = false;
	irq_vector = 0;
	irq_qn = 0;
	watchdog_fired = false;
	vblank_start_time = at_zero;
	for (int s = 0; s < 8; s++)
		for (int i = 0; i < 32; i++) wave[s][i] = (int8_t)((r->wave[s * 32 + i] & 0x0f) - 8);
	memset(voice, 0, sizeof voice);
	memset(soundregs, 0, sizeof soundregs);
	sound_enable = true;                // namco device_start: on, until the latch says
	wsg_next = 0;
	bin_w = bin_r = 0;
	if (!bin_step_fx) pm_audio_rate_q8(48000u << 8);
	bin_frac = 0;
	bin_start();
	video_init();
#if PM_MS
	ms_init(r);
#endif
	map_pages();

	// device_start, in MAME's order: the CPU, the latch (outputs all 1s until its reset), the screen's timers
	z80_start();
	pm_latch = 0xff;                    // ls259's m_q
	tm_adjust(TM_VBSTART, time_until_pos(VBSTART), 0);
	tm_adjust(TM_SCAN0, time_until_pos(0), 0);
	tm[TM_SND50].used = true;
	tm[TM_SND50].enabled = true;
	tm[TM_SND50].period = at_as(AS_PER_SEC / 50);
	tm[TM_SND50].expire = tm[TM_SND50].period;
	tm[TM_SND50].seq = ++tm_seq;

	// device_reset (the latch's clear sets the IRQ line clear, through a timer for now)
	board_reset();
}

uint32_t pm_run_to(uint64_t until) {
	uint32_t f0 = pm_frames;
	while (at_ticks(basetime, PM_CLOCK) < until) timeslice();
	return pm_frames - f0;
}

bool pm_frame(void) {
	frame_done = false;
	while (!frame_done) timeslice();
	if (watchdog_fired) {               // schedule_soft_reset: every device's reset
		watchdog_fired = false;
		pm_resets++;
		board_reset();
		return false;
	}
	return true;
}
