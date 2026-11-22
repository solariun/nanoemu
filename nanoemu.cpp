// nanoemu - Arduino Nano (ATmega328P @ 16 MHz) emulator with a terminal UI.
//
// Single file, C++11 + ncurses.
//   macOS : c++ -std=c++11 -O2 -o nanoemu nanoemu.cpp -lncurses
//   Linux : g++ -std=c++11 -O2 -o nanoemu nanoemu.cpp -lncursesw -lutil
//
// Loads .bin (raw flash image), .hex (Intel HEX) or .elf firmware.
// Emulates: full AVRe+ instruction set, Timer0/1/2 (normal, CTC, fast PWM,
// phase-correct PWM, OC pin output), USART0 (bit-accurate TX waveform),
// ADC, GPIO with pull-ups, INT0/INT1, pin-change interrupts, EEPROM,
// watchdog, SPI/TWI stubs, sleep. UI: CPU/registers/disassembly (left),
// pins/peripherals (right), live board (center), serial console + PTY,
// logic-analyzer oscilloscope (bottom).

#define _XOPEN_SOURCE_EXTENDED 1
#define _DARWIN_C_SOURCE 1
#define _DEFAULT_SOURCE 1

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include <fcntl.h>
#include <langinfo.h>
#include <locale.h>
#include <termios.h>
#include <time.h>
#include <poll.h>
#include <signal.h>
#include <unistd.h>
#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h>
#endif
#include <curses.h>
#include <cxxabi.h>
#include <sys/stat.h>

// ============================================================================
//  Constants & helpers
// ============================================================================

static const double F_CPU = 16000000.0;
// Memory sizes: the ATmega328P's by default; --flash / --ram enlarge them for tests (set before Nano is built)
static int FLASH_WORDS = 16384;        // 32 KB
static uint32_t PC_MASK = 0x3FFF;      // FLASH_WORDS - 1
static int RAM_END = 0x900;            // regs + IO + 2 KB SRAM
static const int RAM_END_328P = 0x900; // the stack top the firmware's startup code sets (RAMEND + 1)
static const int BOOT_BYTES = 2048;    // the Nano's bootloader section at the top of flash
static const int EE_SIZE = 1024;
static const int NPINS = 20;           // D0..D13, A0..A5
static const uint32_t PLOG = 1u << 17; // pin-toggle history per pin

enum { FC = 1, FZ = 2, FN = 4, FV = 8, FS = 16, FH = 32, FT = 64, FI = 128 };

enum IoReg {
    PINB = 0x23, DDRB, PORTB, PINC, DDRC, PORTC, PIND, DDRD, PORTD,
    TIFR0 = 0x35, TIFR1, TIFR2, PCIFR = 0x3B, EIFR, EIMSK, GPIOR0,
    EECR = 0x3F, EEDR, EEARL, EEARH,
    SPCR = 0x4C, SPSR, SPDR, SMCR = 0x53, MCUSR, MCUCR, SPL = 0x5D, SPH, SREG_A,
    WDTCSR = 0x60, PCICR = 0x68, EICRA, PCMSK0 = 0x6B, PCMSK1, PCMSK2,
    TIMSK0 = 0x6E, TIMSK1, TIMSK2,
    ADCL = 0x78, ADCH, ADCSRA, ADCSRB, ADMUX,
    TCCR1A = 0x80, TCCR1B, TCCR1C, TCNT1L = 0x84, TCNT1H, ICR1L, ICR1H,
    OCR1AL, OCR1AH, OCR1BL, OCR1BH,
    TWBR = 0xB8, TWSR, TWAR, TWDR, TWCR,
    UCSR0A = 0xC0, UCSR0B, UCSR0C, UBRR0L = 0xC4, UBRR0H, UDR0
};

static double mono() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static std::string strf(const char *fmt, ...) {
    char b[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    return b;
}

static std::string fmt_time(double s) {
    double a = fabs(s);
    if (a >= 1) return strf("%.3gs", s);
    if (a >= 1e-3) return strf("%.3gms", s * 1e3);
    return strf("%.3gus", s * 1e6);
}

static const char *IRQ_NAMES[26] = {
    "RESET", "INT0", "INT1", "PCINT0", "PCINT1", "PCINT2", "WDT",
    "TIMER2_COMPA", "TIMER2_COMPB", "TIMER2_OVF", "TIMER1_CAPT",
    "TIMER1_COMPA", "TIMER1_COMPB", "TIMER1_OVF", "TIMER0_COMPA",
    "TIMER0_COMPB", "TIMER0_OVF", "SPI_STC", "USART_RX", "USART_UDRE",
    "USART_TX", "ADC", "EE_READY", "ANALOG_COMP", "TWI", "SPM_READY"};

// Arduino pin <-> port mapping (port index: 0=B, 1=C, 2=D)
static const char *PIN_NAME[NPINS] = {
    "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8", "D9",
    "D10", "D11", "D12", "D13", "A0", "A1", "A2", "A3", "A4", "A5"};
static int pin_port(int i) { return i < 8 ? 2 : i < 14 ? 0 : 1; }
static int pin_bit(int i) { return i < 8 ? i : i < 14 ? i - 8 : i - 14; }
static int port_pin(int p, int b) {
    if (p == 2) return b;
    if (b >= 6) return -1;
    return p == 0 ? 8 + b : 14 + b;
}

static int A_DDR(int p) { return 0x24 + 3 * p; }
static int A_PORT(int p) { return 0x25 + 3 * p; }

// ============================================================================
//  The microcontroller
// ============================================================================

class Nano {
public:
    // ---- core state ----
    std::vector<uint16_t> flash;
    std::vector<uint8_t> ram;
    std::vector<uint8_t> touched;  // per SRAM byte: written at least once since reset
    uint16_t min_sp = 0xFFFF;      // lowest stack pointer seen since reset (stack peak)
    uint8_t eeprom[EE_SIZE];
    uint16_t pc = 0;
    uint64_t cycles = 0, instrs = 0;
    bool sleeping = false, irq_inhibit = false;
    bool paused = false;
    std::string event;   // set when execution stops on its own

    std::vector<uint8_t> bpw; // breakpoint flag per flash word
    const uint8_t *user_map = 0;  // per flash word: 1 when its source is user code (set by the UI)
    int last_user_pc = -1;        // last word address executed in user code
    uint64_t irq_count[26] = {0};
    int last_irq = 0;

    // ---- pins ----
    uint8_t lv[3] = {0, 0, 0};       // level on the wire per port
    uint8_t ovr_mask[3] = {0, 0, 0}; // peripheral-driven bits
    uint8_t ovr_val[3] = {0, 0, 0};
    int8_t ext[NPINS];               // external drive: -1 float, 0, 1
    uint16_t adcv[8] = {512, 256, 768, 1023, 0, 100, 600, 900};
    std::vector<uint64_t> plog[NPINS]; // (cycle<<1)|level, ring
    uint64_t pcount[NPINS] = {0};

    // ---- timers ----
    enum { K_NORMAL, K_CTC, K_FAST, K_PC };
    struct Timer {
        const char *name;
        bool wide;
        uint8_t tccra, tccrb, tcnt, ocra, ocrb, timsk, tifr;
        int pa, ba, pb, bb; // OC pins
        const uint16_t *div;
        uint32_t pre;
        int dir;
        uint8_t oc;
    };
    Timer tm[3];
    uint8_t t16temp = 0;

    // ---- usart ----
    struct {
        uint8_t txbuf = 0, cur = 0, rxd = 0;
        bool txfull = false, shifting = false;
        uint16_t sh = 0;
        int bits = 0;
        int32_t cnt = 0;
        uint64_t rxnext = 0, txn = 0, rxn = 0;
    } us;
    std::deque<uint8_t> rxq;
    std::string txout; // bytes emitted since the UI last drained it

    // ---- misc peripherals ----
    int32_t adc_cnt = 0;
    bool adc_first = true;
    uint64_t ee_mpe_until = 0;
    int32_t ee_busy = 0;
    uint32_t ee_writes = 0;
    uint64_t wdt_cnt = 0;

    Nano() {
        static const uint16_t div01[8] = {0, 1, 8, 64, 256, 1024, 0, 0};
        static const uint16_t div2[8] = {0, 1, 8, 32, 64, 128, 256, 1024};
        Timer t0 = {"T0", false, 0x44, 0x45, 0x46, 0x47, 0x48, TIMSK0, TIFR0, 2, 6, 2, 5, div01, 0, 1, 0};
        Timer t1 = {"T1", true, 0x80, 0x81, 0x84, 0x88, 0x8A, TIMSK1, TIFR1, 0, 1, 0, 2, div01, 0, 1, 0};
        Timer t2 = {"T2", false, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, TIMSK2, TIFR2, 0, 3, 2, 3, div2, 0, 1, 0};
        tm[0] = t0; tm[1] = t1; tm[2] = t2;
        flash.assign(FLASH_WORDS, 0xFFFF);
        ram.assign(RAM_END, 0);
        touched.assign(RAM_END, 0);
        memset(eeprom, 0xFF, sizeof eeprom);
        for (int i = 0; i < NPINS; i++) { ext[i] = -1; plog[i].resize(PLOG); }
        bpw.assign(FLASH_WORDS, 0);
        reset(0x01, true);
    }

    // ------------------------------------------------------------------ reset
    void reset(uint8_t mcusr, bool hard) {
        std::fill(ram.begin(), ram.end(), 0);
        std::fill(touched.begin(), touched.end(), 0);
        ram[SPL] = (RAM_END - 1) & 0xFF; ram[SPH] = (RAM_END - 1) >> 8;
        min_sp = RAM_END - 1;
        ram[UCSR0A] = 0x20; ram[UCSR0C] = 0x06;
        ram[TWSR] = 0xF8;
        ram[MCUSR] = mcusr;
        pc = 0; sleeping = irq_inhibit = false;
        if (hard) last_user_pc = -1;
        for (int i = 0; i < 3; i++) { tm[i].pre = 0; tm[i].dir = 1; tm[i].oc = 0; }
        us = decltype(us)();
        rxq.clear();
        adc_cnt = 0; adc_first = true; ee_busy = 0; wdt_cnt = 0;
        memset(ovr_val, 0, 3);
        ovr_val[2] |= 2; // TX idles high
        if (hard) {
            cycles = instrs = 0;
            memset(pcount, 0, sizeof pcount);
            memset(irq_count, 0, sizeof irq_count);
            last_irq = 0;
            memset(lv, 0, 3);
        }
        update_overrides();
        pins_changed();
    }

    // ---------------------------------------------------------- firmware load
    bool load(const std::string &path, std::string &err, int &size) {
        FILE *f = fopen(path.c_str(), "rb");
        if (!f) { err = strf("cannot open %s: %s", path.c_str(), strerror(errno)); return false; }
        std::vector<uint8_t> buf;
        uint8_t tmp[65536];
        size_t n;
        while ((n = fread(tmp, 1, sizeof tmp, f)) > 0) buf.insert(buf.end(), tmp, tmp + n);
        fclose(f);
        std::vector<uint8_t> img(FLASH_WORDS * 2, 0xFF);
        int top = 0;
        std::string ext;
        size_t dot = path.rfind('.');
        if (dot != std::string::npos) ext = path.substr(dot);
        for (size_t i = 0; i < ext.size(); i++) ext[i] = tolower(ext[i]);

        if (buf.size() > 52 && buf[0] == 0x7F && buf[1] == 'E' && buf[2] == 'L' && buf[3] == 'F') {
            auto u16 = [&](size_t o) { return (uint32_t)buf[o] | (uint32_t)buf[o + 1] << 8; };
            auto u32 = [&](size_t o) { return u16(o) | u16(o + 2) << 16; };
            uint32_t phoff = u32(28), phes = u16(42), phn = u16(44);
            for (uint32_t i = 0; i < phn; i++) {
                size_t ph = phoff + i * phes;
                if (ph + 32 > buf.size()) break;
                if (u32(ph) != 1) continue; // PT_LOAD
                uint32_t off = u32(ph + 4), paddr = u32(ph + 12), fsz = u32(ph + 16);
                if (paddr >= 0x800000) continue;
                for (uint32_t k = 0; k < fsz && off + k < buf.size(); k++)
                    if (paddr + k < img.size()) { img[paddr + k] = buf[off + k]; top = std::max(top, (int)(paddr + k + 1)); }
            }
        } else if (ext == ".hex" || (!buf.empty() && buf[0] == ':')) {
            std::string s(buf.begin(), buf.end());
            uint32_t base = 0;
            size_t p = 0;
            int line = 0;
            while ((p = s.find(':', p)) != std::string::npos) {
                line++;
                auto hx = [&](size_t o) -> int {
                    if (o + 2 > s.size() || !isxdigit(s[o]) || !isxdigit(s[o + 1])) return -1;
                    return (int)strtol(s.substr(o, 2).c_str(), 0, 16);
                };
                int cnt = hx(p + 1), ah = hx(p + 3), al = hx(p + 5), ty = hx(p + 7);
                if (cnt < 0 || ah < 0 || al < 0 || ty < 0) { err = strf("bad HEX record at line %d", line); return false; }
                int sum = cnt + ah + al + ty;
                std::vector<uint8_t> d;
                for (int i = 0; i < cnt; i++) { int v = hx(p + 9 + 2 * i); if (v < 0) { err = strf("bad HEX data at line %d", line); return false; } d.push_back(v); sum += v; }
                int ck = hx(p + 9 + 2 * cnt);
                if (ck < 0 || ((sum + ck) & 0xFF)) { err = strf("HEX checksum error at line %d", line); return false; }
                uint32_t addr = (ah << 8) | al;
                if (ty == 0) {
                    for (int i = 0; i < cnt; i++) {
                        uint32_t a = base + addr + i;
                        if (a < img.size()) { img[a] = d[i]; top = std::max(top, (int)a + 1); }
                    }
                } else if (ty == 1) break;
                else if (ty == 2 && cnt == 2) base = ((d[0] << 8) | d[1]) << 4;
                else if (ty == 4 && cnt == 2) base = ((d[0] << 8) | d[1]) << 16;
                p += 11 + 2 * cnt;
            }
        } else {
            if (buf.size() > img.size()) { err = strf("image too large (%zu > 32768 bytes)", buf.size()); return false; }
            std::copy(buf.begin(), buf.end(), img.begin());
            top = (int)buf.size();
        }
        if (top == 0) { err = "firmware is empty"; return false; }
        for (int i = 0; i < FLASH_WORDS; i++) flash[i] = img[2 * i] | (img[2 * i + 1] << 8);
        size = top;
        return true;
    }

    // ================================================================== pins
    void plog_push(int i, int level) {
        plog[i][pcount[i] % PLOG] = (cycles << 1) | (uint64_t)level;
        pcount[i]++;
    }

    uint8_t port_level(int p) const {
        uint8_t ddr = ram[A_DDR(p)], port = ram[A_PORT(p)];
        uint8_t outm = ddr, outv = (port & ~ovr_mask[p]) | (ovr_val[p] & ovr_mask[p]);
        if (p == 2 && (ram[UCSR0B] & 0x08)) outm |= 2;
        uint8_t em = 0, ev = 0;
        for (int b = 0; b < 8; b++) {
            int i = port_pin(p, b);
            if (i >= 0 && ext[i] >= 0) { em |= 1 << b; if (ext[i]) ev |= 1 << b; }
        }
        uint8_t pu = (ram[MCUCR] & 0x10) ? 0 : port;
        uint8_t inv = (ev & em) | (pu & ~em);
        if (p == 2 && (ram[UCSR0B] & 0x10) && !(em & 1)) inv |= 1; // RX idles high
        return (outv & outm) | (inv & ~outm);
    }

    void pins_changed() {
        static const uint8_t msk[3] = {PCMSK0, PCMSK1, PCMSK2};
        for (int p = 0; p < 3; p++) {
            uint8_t n = port_level(p), ch = n ^ lv[p];
            if (!ch) continue;
            lv[p] = n;
            for (int b = 0; b < 8; b++)
                if (ch & (1 << b)) { int i = port_pin(p, b); if (i >= 0) plog_push(i, (n >> b) & 1); }
            if ((ch & ram[msk[p]]) && (ram[PCICR] & (1 << p))) ram[PCIFR] |= 1 << p;
            if (p == 2)
                for (int k = 0; k < 2; k++) {
                    uint8_t bit = 4 << k;
                    if (!(ch & bit)) continue;
                    int isc = (ram[EICRA] >> (2 * k)) & 3;
                    bool rising = n & bit;
                    if (isc == 1 || (isc == 2 && !rising) || (isc == 3 && rising)) ram[EIFR] |= 1 << k;
                }
        }
    }

    int level(int pin) const { return (lv[pin_port(pin)] >> pin_bit(pin)) & 1; }

    // ================================================================ timers
    void timer_mode(const Timer &t, int &kind, uint16_t &top, bool &toc) const {
        uint8_t a = ram[t.tccra], b = ram[t.tccrb];
        toc = false;
        if (!t.wide) {
            uint16_t ocra = ram[t.ocra];
            switch ((a & 3) | ((b >> 1) & 4)) {
            case 1: kind = K_PC; top = 0xFF; break;
            case 2: kind = K_CTC; top = ocra; toc = true; break;
            case 3: kind = K_FAST; top = 0xFF; break;
            case 5: kind = K_PC; top = ocra; toc = true; break;
            case 7: kind = K_FAST; top = ocra; toc = true; break;
            default: kind = K_NORMAL; top = 0xFF;
            }
        } else {
            static const uint8_t kinds[16] = {K_NORMAL, K_PC, K_PC, K_PC, K_CTC, K_FAST, K_FAST, K_FAST,
                                              K_PC, K_PC, K_PC, K_PC, K_CTC, K_NORMAL, K_FAST, K_FAST};
            int w = (a & 3) | ((b >> 1) & 0xC);
            uint16_t ocra = r16(t.ocra), icr = r16(ICR1L);
            const uint16_t tops[16] = {0xFFFF, 0xFF, 0x1FF, 0x3FF, ocra, 0xFF, 0x1FF, 0x3FF,
                                       icr, ocra, icr, ocra, icr, 0xFFFF, icr, ocra};
            kind = kinds[w];
            top = tops[w];
            toc = (w == 4 || w == 9 || w == 11 || w == 15);
        }
    }

    uint16_t r16(int a) const { return ram[a] | (ram[a + 1] << 8); }
    void w16(int a, uint16_t v) { ram[a] = v & 0xFF; ram[a + 1] = v >> 8; }

    void oc_set(Timer &t, int ch, int lvl) { // lvl: 0, 1, 2=toggle
        uint8_t bit = ch ? 2 : 1, old = t.oc;
        if (lvl == 2) t.oc ^= bit; else if (lvl) t.oc |= bit; else t.oc &= ~bit;
        if (t.oc == old) return;
        int p = ch ? t.pb : t.pa, b = ch ? t.bb : t.ba;
        if (t.oc & bit) ovr_val[p] |= 1 << b; else ovr_val[p] &= ~(1 << b);
        if (ovr_mask[p] & (1 << b)) pins_changed();
    }

    void compare_match(Timer &t, int ch, int kind, bool up, bool toc) {
        ram[t.tifr] |= ch ? 4 : 2;
        int com = (ram[t.tccra] >> (ch ? 4 : 6)) & 3;
        if (!com) return;
        if (kind <= K_CTC) oc_set(t, ch, com == 1 ? 2 : com == 2 ? 0 : 1);
        else if (com == 1) { if (ch == 0 && toc) oc_set(t, 0, 2); }
        else if (kind == K_FAST) oc_set(t, ch, com == 3);
        else oc_set(t, ch, com == 2 ? !up : up);
    }

    void timer_count(Timer &t, int kind, uint16_t top, bool toc) {
        uint16_t max = t.wide ? 0xFFFF : 0xFF;
        uint16_t cnt = t.wide ? r16(t.tcnt) : ram[t.tcnt];
        // compare match: TCNT == OCR during this timer clock, output updates on its edge
        bool up = kind != K_PC || t.dir >= 0;
        uint16_t oa = t.wide ? r16(t.ocra) : ram[t.ocra], ob = t.wide ? r16(t.ocrb) : ram[t.ocrb];
        if (cnt == oa) compare_match(t, 0, kind, up, toc);
        if (cnt == ob) compare_match(t, 1, kind, up, toc);
        if (kind == K_PC) {
            if (top == 0) cnt = 0;
            else if (t.dir >= 0) { cnt++; if (cnt >= top) { cnt = top; t.dir = -1; } }
            else { cnt--; if (cnt == 0) { t.dir = 1; ram[t.tifr] |= 1; } }
        } else if (cnt == top) {
            cnt = 0;
            if (kind != K_CTC || top == max) ram[t.tifr] |= 1;
            if (kind == K_FAST)
                for (int ch = 0; ch < 2; ch++) {
                    int com = (ram[t.tccra] >> (ch ? 4 : 6)) & 3;
                    if (com == 2) oc_set(t, ch, 1); else if (com == 3) oc_set(t, ch, 0);
                }
        } else if (cnt == max) { cnt = 0; ram[t.tifr] |= 1; }
        else cnt++;
        if (t.wide) w16(t.tcnt, cnt); else ram[t.tcnt] = (uint8_t)cnt;
    }

    void timer_run(Timer &t, int c) {
        int cs = ram[t.tccrb] & 7;
        if (!cs || !t.div[cs]) return;
        uint16_t dv = t.div[cs];
        t.pre += c;
        if (t.pre < dv) return;
        int kind; uint16_t top; bool toc;
        timer_mode(t, kind, top, toc);
        while (t.pre >= dv) { t.pre -= dv; timer_count(t, kind, top, toc); }
    }

    bool oc_connected(const Timer &t, int ch) const {
        int kind; uint16_t top; bool toc;
        timer_mode(t, kind, top, toc);
        int com = (ram[t.tccra] >> (ch ? 4 : 6)) & 3;
        return com && (kind <= K_CTC || com >= 2 || (ch == 0 && toc));
    }

    void update_overrides() {
        uint8_t m[3] = {0, 0, 0};
        for (int i = 0; i < 3; i++)
            for (int ch = 0; ch < 2; ch++)
                if (oc_connected(tm[i], ch)) m[ch ? tm[i].pb : tm[i].pa] |= 1 << (ch ? tm[i].bb : tm[i].ba);
        if (ram[UCSR0B] & 0x08) m[2] |= 2;
        memcpy(ovr_mask, m, 3);
    }

    // ================================================================= usart
    int32_t uart_bitcyc() const {
        int ubrr = ((ram[UBRR0H] & 0xF) << 8) | ram[UBRR0L];
        return (ram[UCSR0A] & 2 ? 8 : 16) * (ubrr + 1);
    }
    double baud() const { return F_CPU / uart_bitcyc(); }
    void tx_line(int l) {
        uint8_t o = ovr_val[2];
        if (l) ovr_val[2] |= 2; else ovr_val[2] &= ~2;
        if (o != ovr_val[2]) pins_changed();
    }
    void uart_start(uint8_t v) {
        us.cur = v; us.sh = (uint16_t)(v << 1) | 0x200; us.bits = 10;
        us.cnt = uart_bitcyc(); us.shifting = true; tx_line(0);
    }
    void uart_write(uint8_t v) {
        if (!(ram[UCSR0B] & 0x08)) return;
        if (!us.shifting) uart_start(v);
        else if (!us.txfull) { us.txbuf = v; us.txfull = true; ram[UCSR0A] &= ~0x20; }
    }
    void uart_tick(int c) {
        if (us.shifting) {
            us.cnt -= c;
            while (us.shifting && us.cnt <= 0) {
                us.sh >>= 1;
                if (--us.bits == 0) {
                    txout.push_back((char)us.cur);
                    us.txn++;
                    if (us.txfull) {
                        int32_t rem = us.cnt;
                        us.txfull = false; ram[UCSR0A] |= 0x20;
                        uart_start(us.txbuf);
                        us.cnt += rem;
                    } else { us.shifting = false; ram[UCSR0A] |= 0x40; tx_line(1); }
                } else { tx_line(us.sh & 1); us.cnt += uart_bitcyc(); }
            }
        }
        if (!rxq.empty() && (ram[UCSR0B] & 0x10) && !(ram[UCSR0A] & 0x80) && cycles >= us.rxnext) {
            us.rxd = rxq.front(); rxq.pop_front();
            ram[UCSR0A] |= 0x80;
            us.rxnext = cycles + 10 * uart_bitcyc();
            us.rxn++;
        }
    }

    // =================================================================== adc
    void adc_start() {
        static const int d[8] = {2, 2, 4, 8, 16, 32, 64, 128};
        adc_cnt = d[ram[ADCSRA] & 7] * (adc_first ? 25 : 13);
        adc_first = false;
    }
    void adc_write(uint8_t v) {
        uint8_t o = ram[ADCSRA], n = (v & ~0x10) | (o & 0x10);
        if (v & 0x10) n &= ~0x10;
        if (!(n & 0x80)) { n &= ~0x40; adc_cnt = 0; }
        if (o & 0x40) n |= 0x40; // can't abort a running conversion
        ram[ADCSRA] = n;
        if ((n & 0xC0) == 0xC0 && !(o & 0x40)) adc_start();
    }
    void adc_tick(int c) {
        adc_cnt -= c;
        if (adc_cnt > 0) return;
        int mux = ram[ADMUX] & 0xF;
        double vin = mux < 8 ? adcv[mux] * 5.0 / 1023 : mux == 8 ? 0.314 : mux == 14 ? 1.1 : 0;
        double vref = (ram[ADMUX] >> 6) == 3 ? 1.1 : 5.0;
        int r = (int)lround(vin / vref * 1023);
        if (r > 1023) r = 1023;
        if (ram[ADMUX] & 0x20) r <<= 6;
        ram[ADCL] = r & 0xFF; ram[ADCH] = r >> 8;
        ram[ADCSRA] |= 0x10;
        if ((ram[ADCSRA] & 0x20) && (ram[ADCSRB] & 7) == 0) adc_start();
        else { ram[ADCSRA] &= ~0x40; adc_cnt = 0; }
    }

    // ================================================================ eeprom
    void ee_write(uint8_t v) {
        uint8_t o = ram[EECR], n = (o & 0x02) | (v & 0x38);
        uint16_t a = (ram[EEARL] | ram[EEARH] << 8) & (EE_SIZE - 1);
        if (v & 0x04) { n |= 0x04; ee_mpe_until = cycles + 4; }
        if ((v & 0x02) && (o & 0x04) && !(o & 0x02) && cycles <= ee_mpe_until) {
            int mode = (n >> 4) & 3;
            eeprom[a] = mode == 0 ? ram[EEDR] : mode == 1 ? 0xFF : (eeprom[a] & ram[EEDR]);
            n = (n | 0x02) & ~0x04;
            ee_busy = 54400; // 3.4 ms
            ee_writes++;
        }
        if (v & 0x01) ram[EEDR] = eeprom[a];
        ram[EECR] = n;
    }

    // =================================================================== twi
    // A bus with no devices: every address gets NACK.
    void twi_write(uint8_t v) {
        if (!(v & 0x04)) { ram[TWCR] = v & 0x7F; return; }
        if (!(v & 0x80)) { ram[TWCR] = (ram[TWCR] & 0x80) | (v & 0x7F); return; }
        uint8_t pres = ram[TWSR] & 3, s;
        if (v & 0x20) s = 0x08;                                   // START
        else if (v & 0x10) { ram[TWCR] = v & ~0x90; ram[TWSR] = 0xF8 | pres; return; } // STOP
        else { s = ram[TWSR] & 0xF8; s = (s == 0x08 || s == 0x10) ? ((ram[TWDR] & 1) ? 0x48 : 0x20) : 0x30; }
        ram[TWSR] = s | pres;
        ram[TWCR] = v | 0x80;
    }

    // ============================================================ io access
    static bool w1c(uint16_t a) {
        return a == PINB || a == PINC || a == PIND || a == TIFR0 || a == TIFR1 || a == TIFR2 || a == PCIFR || a == EIFR;
    }

    uint8_t io_read(uint16_t a) {
        switch (a) {
        case PINB: return lv[0];
        case PINC: return lv[1];
        case PIND: return lv[2];
        case TCNT1L: case ICR1L: t16temp = ram[a + 1]; return ram[a];
        case TCNT1H: case ICR1H: return t16temp;
        case UDR0: ram[UCSR0A] &= ~0x80; return us.rxd;
        case SPDR: ram[SPSR] &= ~0x80; return 0xFF;
        default: return ram[a];
        }
    }

    void io_write(uint16_t a, uint8_t v) {
        switch (a) {
        case PINB: case PINC: case PIND: ram[a + 2] ^= v; pins_changed(); return;
        case DDRB: case PORTB: case DDRC: case PORTC: case DDRD: case PORTD: case MCUCR:
            ram[a] = v; pins_changed(); return;
        case TIFR0: case TIFR1: case TIFR2: case PCIFR: case EIFR: ram[a] &= ~v; return;
        case 0x44: case 0x45: case 0x80: case 0x81: case 0xB0: case 0xB1:
            ram[a] = (a == 0x45 || a == 0xB1) ? (v & 0x0F) : v;
            update_overrides(); pins_changed(); return;
        case TCNT1H: case ICR1H: case OCR1AH: case OCR1BH: t16temp = v; return;
        case TCNT1L: case ICR1L: case OCR1AL: case OCR1BL: ram[a] = v; ram[a + 1] = t16temp; return;
        case UDR0: uart_write(v); return;
        case UCSR0A: { uint8_t o = (ram[a] & 0xFC) | (v & 3); if (v & 0x40) o &= ~0x40; ram[a] = o; return; }
        case UCSR0B: ram[a] = (v & ~0x02) | (ram[a] & 0x02); update_overrides(); pins_changed(); return;
        case ADCSRA: adc_write(v); return;
        case ADCL: case ADCH: return;
        case EECR: ee_write(v); return;
        case WDTCSR: {
            uint8_t n = (v & 0x7F) | (ram[a] & 0x80);
            if (v & 0x80) n &= ~0x80;
            ram[a] = n; wdt_cnt = 0; return;
        }
        case SPDR: ram[a] = v; ram[SPSR] |= 0x80; return;
        case SPL: case SPH: ram[a] = v; sp_written(); return;
        case SPSR: ram[a] = (ram[a] & 0xFE) | (v & 1); return;
        case TWCR: twi_write(v); return;
        default: ram[a] = v;
        }
    }

    inline uint8_t rd(uint16_t a) {
        if (a < 0x20 || (a >= 0x100 && a < RAM_END)) return ram[a];
        if (a < 0x100) return io_read(a);
        return 0;
    }
    inline void wr(uint16_t a, uint8_t v) {
        if (a < 0x20) ram[a] = v;
        else if (a >= 0x100 && a < RAM_END) { ram[a] = v; touched[a] = 1; }
        else if (a < 0x100) io_write(a, v);
    }

    // ============================================================ cpu helpers
    uint8_t &SREG() { return ram[SREG_A]; }
    uint16_t sp() const { return ram[SPL] | (ram[SPH] << 8); }
    void set_sp(uint16_t s) { ram[SPL] = s & 0xFF; ram[SPH] = s >> 8; if (s < min_sp) min_sp = s; }
    // after a write to SPL/SPH: firmware built for the 328P sets SP to 0x08FF at startup; with --ram,
    // that becomes the real top so the stack and malloc() can use the extra RAM
    void sp_written() {
        uint16_t s = sp();
        if (RAM_END > RAM_END_328P && s == RAM_END_328P - 1) { s = RAM_END - 1; ram[SPL] = s & 0xFF; ram[SPH] = s >> 8; }
        if (s < min_sp && s >= 0x100) min_sp = s;
    }
    void push8(uint8_t v) { uint16_t s = sp(); wr(s, v); set_sp(s - 1); }
    uint8_t pop8() { uint16_t s = sp() + 1; set_sp(s); return rd(s); }
    void push16(uint16_t v) { push8(v & 0xFF); push8(v >> 8); }
    uint16_t pop16() { uint8_t h = pop8(), l = pop8(); return l | (h << 8); }
    uint16_t reg16(int n) const { return ram[n] | (ram[n + 1] << 8); }
    void set_reg16(int n, uint16_t v) { ram[n] = v & 0xFF; ram[n + 1] = v >> 8; }
    uint8_t lpm(uint16_t z) const { return (flash[(z >> 1) & PC_MASK] >> ((z & 1) * 8)) & 0xFF; }
    static bool is32(uint16_t op) { return (op & 0xFC0F) == 0x9000 || (op & 0xFE0C) == 0x940C; }

    static uint8_t nzs(uint8_t s, uint8_t r) {
        s &= ~(FN | FZ | FS);
        if (r & 0x80) s |= FN;
        if (!r) s |= FZ;
        if (((s >> 2) ^ (s >> 3)) & 1) s |= FS;
        return s;
    }
    uint8_t do_add(uint8_t d, uint8_t r, int c) {
        uint8_t R = d + r + c;
        uint8_t cr = (d & r) | (r & ~R) | (~R & d);
        uint8_t s = SREG() & (FI | FT);
        if (cr & 0x80) s |= FC;
        if (cr & 0x08) s |= FH;
        if (((d & r & ~R) | (~d & ~r & R)) & 0x80) s |= FV;
        SREG() = nzs(s, R);
        return R;
    }
    uint8_t do_sub(uint8_t d, uint8_t r, int c, bool keepz) {
        uint8_t R = d - r - c;
        uint8_t br = (~d & r) | (r & R) | (R & ~d);
        uint8_t z = SREG() & FZ;
        uint8_t s = SREG() & (FI | FT);
        if (br & 0x80) s |= FC;
        if (br & 0x08) s |= FH;
        if (((d & ~r & ~R) | (~d & r & R)) & 0x80) s |= FV;
        s = nzs(s, R);
        if (keepz && !z) s &= ~FZ;
        SREG() = s;
        return R;
    }
    void logic(uint8_t r) { SREG() = nzs(SREG() & ~FV, r); }
    void set_mul(uint16_t r, int c) {
        ram[0] = r & 0xFF; ram[1] = r >> 8;
        uint8_t s = SREG() & ~(FC | FZ);
        if (c) s |= FC;
        if (!r) s |= FZ;
        SREG() = s;
    }

    // =========================================================== instruction
    int step_cpu() {
        uint16_t op = flash[pc & PC_MASK];
        uint16_t npc = pc + 1;
        int cyc = 1;
        int d = (op >> 4) & 0x1F, r = (op & 0xF) | ((op >> 5) & 0x10);
        int dh = 16 + ((op >> 4) & 0xF);
        uint8_t K = ((op >> 4) & 0xF0) | (op & 0xF);
        instrs++;
#define SKIP() do { int w_ = is32(flash[npc & PC_MASK]) ? 2 : 1; npc += w_; cyc += w_; } while (0)
        switch (op >> 12) {
        case 0x0:
            switch ((op >> 10) & 3) {
            case 0:
                switch ((op >> 8) & 3) {
                case 0: break; // NOP
                case 1: { int a = ((op >> 4) & 0xF) * 2, b = (op & 0xF) * 2; ram[a] = ram[b]; ram[a + 1] = ram[b + 1]; break; }
                case 2: { int32_t p = (int8_t)ram[dh] * (int8_t)ram[16 + (op & 0xF)]; set_mul((uint16_t)p, (p >> 15) & 1); cyc = 2; break; }
                case 3: {
                    int a = 16 + ((op >> 4) & 7), b = 16 + (op & 7);
                    int32_t p;
                    switch (((op >> 6) & 2) | ((op >> 3) & 1)) {
                    case 0: p = (int8_t)ram[a] * (int32_t)ram[b]; set_mul((uint16_t)p, (p >> 15) & 1); break; // MULSU
                    case 1: p = ram[a] * ram[b]; set_mul((uint16_t)(p << 1), (p >> 15) & 1); break;              // FMUL
                    case 2: p = (int8_t)ram[a] * (int8_t)ram[b]; set_mul((uint16_t)(p << 1), (p >> 15) & 1); break; // FMULS
                    default: p = (int8_t)ram[a] * (int32_t)ram[b]; set_mul((uint16_t)(p << 1), (p >> 15) & 1); break; // FMULSU
                    }
                    cyc = 2;
                    break;
                }
                }
                break;
            case 1: do_sub(ram[d], ram[r], SREG() & FC, true); break;            // CPC
            case 2: ram[d] = do_sub(ram[d], ram[r], SREG() & FC, true); break;   // SBC
            case 3: ram[d] = do_add(ram[d], ram[r], 0); break;                   // ADD
            }
            break;
        case 0x1:
            switch ((op >> 10) & 3) {
            case 0: if (ram[d] == ram[r]) SKIP(); break;                        // CPSE
            case 1: do_sub(ram[d], ram[r], 0, false); break;                    // CP
            case 2: ram[d] = do_sub(ram[d], ram[r], 0, false); break;           // SUB
            case 3: ram[d] = do_add(ram[d], ram[r], SREG() & FC); break;        // ADC
            }
            break;
        case 0x2:
            switch ((op >> 10) & 3) {
            case 0: ram[d] &= ram[r]; logic(ram[d]); break;
            case 1: ram[d] ^= ram[r]; logic(ram[d]); break;
            case 2: ram[d] |= ram[r]; logic(ram[d]); break;
            case 3: ram[d] = ram[r]; break;
            }
            break;
        case 0x3: do_sub(ram[dh], K, 0, false); break;                          // CPI
        case 0x4: ram[dh] = do_sub(ram[dh], K, SREG() & FC, true); break;       // SBCI
        case 0x5: ram[dh] = do_sub(ram[dh], K, 0, false); break;                // SUBI
        case 0x6: ram[dh] |= K; logic(ram[dh]); break;                          // ORI
        case 0x7: ram[dh] &= K; logic(ram[dh]); break;                          // ANDI
        case 0x8: case 0xA: {                                                   // LDD/STD
            int q = (op & 7) | ((op >> 7) & 0x18) | ((op >> 8) & 0x20);
            uint16_t a = reg16((op & 8) ? 28 : 30) + q;
            if (op & 0x200) wr(a, ram[d]); else ram[d] = rd(a);
            cyc = 2;
            break;
        }
        case 0x9:
            switch ((op >> 9) & 7) {
            case 0: case 1: {
                bool st = op & 0x200;
                cyc = 2;
                switch (op & 0xF) {
                case 0x0: { uint16_t a = flash[npc & PC_MASK]; npc++; if (st) wr(a, ram[d]); else ram[d] = rd(a); break; }
                case 0x1: case 0x2: case 0x9: case 0xA: case 0xC: case 0xD: case 0xE: {
                    int pr = (op & 0xF) >= 0xC ? 26 : (op & 0x8) ? 28 : 30;
                    uint16_t a = reg16(pr);
                    int m = op & 3;
                    if (m == 2) a--;
                    if (st) wr(a, ram[d]); else ram[d] = rd(a);
                    if (m == 1) a++;
                    if (m) set_reg16(pr, a);
                    break;
                }
                case 0x4: case 0x5: case 0x6: case 0x7:
                    if (!st) { uint16_t z = reg16(30); ram[d] = lpm(z); if (op & 1) set_reg16(30, z + 1); cyc = 3; }
                    break;
                case 0xF: if (st) push8(ram[d]); else ram[d] = pop8(); break;
                default: break;
                }
                break;
            }
            case 2: {
                int lo = op & 0xF;
                if (lo < 8 && lo != 4) {
                    uint8_t v = ram[d], R = v, s = SREG();
                    switch (lo) {
                    case 0: R = ~v; s = (s & ~FV) | FC; break;                                       // COM
                    case 1: R = 0 - v; s &= ~(FH | FV | FC); if ((R | v) & 8) s |= FH; if (R == 0x80) s |= FV; if (R) s |= FC; break; // NEG
                    case 2: R = (v << 4) | (v >> 4); break;                                         // SWAP
                    case 3: R = v + 1; s &= ~FV; if (R == 0x80) s |= FV; break;                     // INC
                    case 5: case 6: case 7:                                                         // ASR LSR ROR
                        R = lo == 5 ? (v >> 1) | (v & 0x80) : lo == 6 ? v >> 1 : (v >> 1) | ((s & FC) << 7);
                        s &= ~(FC | FV);
                        if (v & 1) s |= FC;
                        if (((R >> 7) ^ v) & 1) s |= FV;
                        break;
                    }
                    if (lo != 2) s = nzs(s, R);
                    ram[d] = R;
                    SREG() = s;
                } else if (lo == 0xA) {                                                              // DEC
                    uint8_t R = ram[d] - 1, s = SREG() & ~FV;
                    if (R == 0x7F) s |= FV;
                    SREG() = nzs(s, R);
                    ram[d] = R;
                } else if (lo >= 0xC) {                                                              // JMP/CALL
                    uint32_t k = ((uint32_t)(((op >> 3) & 0x3E) | (op & 1)) << 16) | flash[npc & PC_MASK];
                    npc++;
                    if (lo & 2) { push16(npc); cyc = 4; } else cyc = 3;
                    npc = k;
                } else if (lo == 8) {
                    if (!(op & 0x100)) {                                                             // BSET/BCLR
                        int b = (op >> 4) & 7;
                        if (op & 0x80) SREG() &= ~(1 << b);
                        else { SREG() |= 1 << b; if (b == 7) irq_inhibit = true; }
                    } else switch (op) {
                        case 0x9508: npc = pop16(); cyc = 4; break;                                  // RET
                        case 0x9518: npc = pop16(); SREG() |= FI; irq_inhibit = true; cyc = 4; break; // RETI
                        case 0x9588: { if (ram[SMCR] & 1) sleeping = true; } break;                      // SLEEP
                        case 0x9598: paused = true; event = strf("BREAK instruction at 0x%04X", pc * 2); break;
                        case 0x95A8: wdt_cnt = 0; break;                                             // WDR
                        case 0x95C8: case 0x95D8: ram[0] = lpm(reg16(30)); cyc = 3; break;           // LPM
                        default: break;                                                              // SPM: ignored
                    }
                } else if (lo == 9) {
                    if (op == 0x9409) { npc = reg16(30); cyc = 2; }                                  // IJMP
                    else if (op == 0x9509) { push16(npc); npc = reg16(30); cyc = 3; }                // ICALL
                }
                break;
            }
            case 3: {                                                                                // ADIW/SBIW
                int rr = 24 + ((op >> 3) & 6);
                uint8_t k = ((op >> 2) & 0x30) | (op & 0xF);
                uint16_t v = reg16(rr), R;
                uint8_t s = SREG() & ~(FC | FZ | FN | FV | FS);
                if (!(op & 0x100)) { R = v + k; if (~v & R & 0x8000) s |= FV; if (~R & v & 0x8000) s |= FC; }
                else { R = v - k; if (v & ~R & 0x8000) s |= FV; if (R & ~v & 0x8000) s |= FC; }
                if (R & 0x8000) s |= FN;
                if (!R) s |= FZ;
                if (((s >> 2) ^ (s >> 3)) & 1) s |= FS;
                SREG() = s;
                set_reg16(rr, R);
                cyc = 2;
                break;
            }
            case 4: case 5: {                                                                        // CBI SBIC SBI SBIS
                uint16_t a = 0x20 + ((op >> 3) & 0x1F);
                uint8_t bit = 1 << (op & 7);
                switch ((op >> 8) & 3) {
                case 0: if (!w1c(a)) wr(a, rd(a) & ~bit); cyc = 2; break;
                case 1: if (!(rd(a) & bit)) SKIP(); break;
                case 2: if (w1c(a)) wr(a, bit); else wr(a, rd(a) | bit); cyc = 2; break;
                case 3: if (rd(a) & bit) SKIP(); break;
                }
                break;
            }
            default: { uint16_t p = ram[d] * ram[r]; set_mul(p, p >> 15); cyc = 2; break; }       // MUL
            }
            break;
        case 0xB: {                                                                                  // IN/OUT
            uint16_t a = 0x20 + (((op >> 5) & 0x30) | (op & 0xF));
            if (op & 0x800) wr(a, ram[d]); else ram[d] = rd(a);
            break;
        }
        case 0xC: npc = pc + 1 + ((int16_t)(op << 4) >> 4); cyc = 2; break;                        // RJMP
        case 0xD: push16(pc + 1); npc = pc + 1 + ((int16_t)(op << 4) >> 4); cyc = 3; break;        // RCALL
        case 0xE: ram[dh] = K; break;                                                                // LDI
        case 0xF: {
            int b = op & 7;
            switch ((op >> 10) & 3) {
            case 0: case 1: {                                                                        // BRBS/BRBC
                bool set = (SREG() >> b) & 1;
                if (set != (bool)((op >> 10) & 1)) { npc += (int8_t)(((op >> 3) & 0x7F) << 1) >> 1; cyc = 2; }
                break;
            }
            case 2:
                if (op & 0x200) { if (ram[d] & (1 << b)) SREG() |= FT; else SREG() &= ~FT; }          // BST
                else { if (SREG() & FT) ram[d] |= 1 << b; else ram[d] &= ~(1 << b); }                // BLD
                break;
            case 3: if (((ram[d] >> b) & 1) == ((op >> 9) & 1)) SKIP(); break;                     // SBRC/SBRS
            }
            break;
        }
        }
#undef SKIP
        pc = npc & PC_MASK;
        return cyc;
    }

    // ============================================================ interrupts
    int irq_pending() const {
        if ((ram[EIMSK] & 1) && (((ram[EICRA] & 3) == 0) ? !(lv[2] & 4) : (ram[EIFR] & 1))) return 1;
        if ((ram[EIMSK] & 2) && ((((ram[EICRA] >> 2) & 3) == 0) ? !(lv[2] & 8) : (ram[EIFR] & 2))) return 2;
        uint8_t f = ram[PCIFR] & ram[PCICR];
        if (f & 1) return 3;
        if (f & 2) return 4;
        if (f & 4) return 5;
        if ((ram[WDTCSR] & 0xC0) == 0xC0) return 6;
        f = ram[TIFR2] & ram[TIMSK2];
        if (f & 2) return 7;
        if (f & 4) return 8;
        if (f & 1) return 9;
        f = ram[TIFR1] & ram[TIMSK1];
        if (f & 0x20) return 10;
        if (f & 2) return 11;
        if (f & 4) return 12;
        if (f & 1) return 13;
        f = ram[TIFR0] & ram[TIMSK0];
        if (f & 2) return 14;
        if (f & 4) return 15;
        if (f & 1) return 16;
        if ((ram[SPCR] & 0x80) && (ram[SPSR] & 0x80)) return 17;
        uint8_t a = ram[UCSR0A], b = ram[UCSR0B];
        if ((a & 0x80) && (b & 0x80)) return 18;
        if ((a & 0x20) && (b & 0x20)) return 19;
        if ((a & 0x40) && (b & 0x40)) return 20;
        if ((ram[ADCSRA] & 0x18) == 0x18) return 21;
        if ((ram[EECR] & 0x08) && !(ram[EECR] & 0x02)) return 22;
        if ((ram[TWCR] & 0x81) == 0x81) return 24;
        return 0;
    }

    void take_irq(int v) {
        static const uint8_t tbit[] = {2, 4, 1, 0x20, 2, 4, 1, 2, 4, 1};
        if (v <= 2) { if ((ram[EICRA] >> (2 * (v - 1))) & 3) ram[EIFR] &= ~(1 << (v - 1)); }
        else if (v <= 5) ram[PCIFR] &= ~(1 << (v - 3));
        else if (v == 6) { ram[WDTCSR] &= ~0x80; if (ram[WDTCSR] & 0x08) ram[WDTCSR] &= ~0x40; }
        else if (v <= 9) ram[TIFR2] &= ~tbit[v - 7];
        else if (v <= 13) ram[TIFR1] &= ~tbit[v - 7];
        else if (v <= 16) ram[TIFR0] &= ~tbit[v - 7];
        else if (v == 17) ram[SPSR] &= ~0x80;
        else if (v == 20) ram[UCSR0A] &= ~0x40;
        else if (v == 21) ram[ADCSRA] &= ~0x10;
        push16(pc);
        SREG() &= ~FI;
        pc = v * 2;
        irq_count[v]++;
        last_irq = v;
    }

    // ================================================================= clock
    void tick(int c) {
        cycles += c;
        for (int i = 0; i < 3; i++) timer_run(tm[i], c);
        if (us.shifting || !rxq.empty()) uart_tick(c);
        if (adc_cnt > 0) adc_tick(c);
        if (ee_busy > 0 && (ee_busy -= c) <= 0) { ee_busy = 0; ram[EECR] &= ~0x02; }
        if ((ram[EECR] & 0x04) && cycles > ee_mpe_until) ram[EECR] &= ~0x04;
        uint8_t w = ram[WDTCSR];
        if (w & 0x48) {
            wdt_cnt += c;
            int p = (w & 7) | ((w >> 2) & 8);
            if (p > 9) p = 9;
            if (wdt_cnt >= (uint64_t)2048 * 125 << p) {
                wdt_cnt = 0;
                if (w & 0x40) ram[WDTCSR] |= 0x80;
                else { reset(0x08, false); event = "watchdog reset"; }
            }
        }
    }

    // executes one instruction (or one sleep slice) plus a pending interrupt
    void step() {
        bool slept = sleeping;
        tick(sleeping ? 4 : step_cpu());
        if (irq_inhibit) irq_inhibit = false;
        else if (SREG() & FI) {
            int v = irq_pending();
            if (v) { sleeping = false; take_irq(v); tick(slept ? 8 : 4); }
        }
        if (user_map && user_map[pc]) last_user_pc = pc;
        if (bpw[pc]) { paused = true; event = strf("breakpoint at 0x%04X", pc * 2); }
    }

    void run_until(uint64_t target) {
        while (cycles < target && !paused) step();
    }

    // ============================================================= pin log
    uint64_t log_idx(int i, uint64_t t) const { // #events with cycle <= t
        uint64_t lo = pcount[i] > PLOG ? pcount[i] - PLOG : 0, hi = pcount[i];
        while (lo < hi) {
            uint64_t m = (lo + hi) / 2;
            if ((plog[i][m % PLOG] >> 1) <= t) lo = m + 1; else hi = m;
        }
        return lo;
    }
    uint64_t ev_cyc(int i, uint64_t k) const { return plog[i][k % PLOG] >> 1; }
    int ev_lvl(int i, uint64_t k) const { return plog[i][k % PLOG] & 1; }
    int level_at(int i, uint64_t t) const {
        uint64_t oldest = pcount[i] > PLOG ? pcount[i] - PLOG : 0;
        if (pcount[i] == 0) return level(i);
        uint64_t k = log_idx(i, t);
        if (k == oldest) return !ev_lvl(i, oldest);
        return ev_lvl(i, k - 1);
    }
    // frequency (Hz) and duty (0..1) of a pin over [ta, tb]
    void measure(int i, uint64_t ta, uint64_t tb, double &freq, double &duty) const {
        freq = 0; duty = 0;
        if (tb <= ta) return;
        uint64_t k0 = log_idx(i, ta), k1 = log_idx(i, tb);
        if (k1 - k0 > 20000) { k0 = k1 - 20000; ta = ev_cyc(i, k0) ; k0++; }
        int lvl = level_at(i, ta);
        uint64_t t = ta, high = 0, first = 0, last = 0;
        int rises = 0;
        for (uint64_t k = k0; k < k1; k++) {
            uint64_t c = ev_cyc(i, k);
            if (lvl) high += c - t;
            t = c;
            lvl = ev_lvl(i, k);
            if (lvl) { if (!rises) first = c; last = c; rises++; }
        }
        if (lvl) high += tb - t;
        duty = (double)high / (tb - ta);
        if (rises >= 2 && last > first) freq = (rises - 1) * F_CPU / (last - first);
    }
    // Pulse statistics of a pin over [ta, tb]. Only pulses with both edges inside the window count
    // for the averages; duty includes the partial ones at the window's ends.
    struct Pulses {
        double freq, period, duty;         // Hz, seconds, 0..1
        double hi_avg, hi_min, hi_max;     // high time per pulse (the crest), seconds
        double lo_avg;                     // low time between pulses, seconds
        int rises, falls, highs, lows;
        uint64_t last_edge;                // cycle of the last edge in the window (0 = none)
    };
    Pulses pulses(int i, uint64_t ta, uint64_t tb) const {
        Pulses r;
        memset(&r, 0, sizeof r);
        if (tb <= ta) return r;
        uint64_t k0 = log_idx(i, ta), k1 = log_idx(i, tb);
        if (k1 - k0 > 20000) { k0 = k1 - 20000; ta = ev_cyc(i, k0); k0++; }
        int lvl = level_at(i, ta);
        uint64_t t = ta, high = 0, first_rise = 0, last_rise = 0, hsum = 0, lsum = 0;
        uint64_t hmin = UINT64_MAX, hmax = 0;
        bool bounded = false; // the current segment started at an edge inside the window
        for (uint64_t k = k0; k < k1; k++) {
            uint64_t c = ev_cyc(i, k);
            uint64_t len = c - t;
            if (lvl) high += len;
            if (bounded) {
                if (lvl) { hsum += len; r.highs++; hmin = std::min(hmin, len); hmax = std::max(hmax, len); }
                else { lsum += len; r.lows++; }
            }
            t = c;
            lvl = ev_lvl(i, k);
            bounded = true;
            r.last_edge = c;
            if (lvl) { if (!r.rises) first_rise = c; last_rise = c; r.rises++; } else r.falls++;
        }
        if (lvl) high += tb - t;
        r.duty = (double)high / (tb - ta);
        if (r.rises >= 2 && last_rise > first_rise) {
            r.period = (last_rise - first_rise) / F_CPU / (r.rises - 1);
            r.freq = 1 / r.period;
        }
        if (r.highs) { r.hi_avg = hsum / F_CPU / r.highs; r.hi_min = hmin / F_CPU; r.hi_max = hmax / F_CPU; }
        if (r.lows) r.lo_avg = lsum / F_CPU / r.lows;
        return r;
    }

    bool pwm_active(int i) const {
        int p = pin_port(i), b = pin_bit(i);
        return (ovr_mask[p] & ram[A_DDR(p)] & (1 << b)) && !(p == 2 && b == 1);
    }

    std::string pin_mode(int i) const {
        int p = pin_port(i), b = pin_bit(i);
        uint8_t m = 1 << b;
        if (i == 1 && (ram[UCSR0B] & 0x08)) return "TX";
        if (i == 0 && (ram[UCSR0B] & 0x10)) return "RX";
        if (pwm_active(i)) return "PWM";
        if (ram[A_DDR(p)] & m) return "OUT";
        if (i >= 14 && (ram[ADCSRA] & 0x80) && (ram[ADMUX] & 0xF) == i - 14) return "ADC";
        if (ram[A_PORT(p)] & m && !(ram[MCUCR] & 0x10)) return "IN_PU";
        return "IN";
    }
};

// ============================================================================
//  Debug info: ELF symbols + DWARF (v2..v5) line tables
// ============================================================================

struct Symbol {
    uint32_t addr, size; // flash byte address (functions) or data address (objects)
    std::string name, mangled;
};

static std::string demangle(const std::string &s) {
    int st = 0;
    char *d = abi::__cxa_demangle(s.c_str(), 0, 0, &st);
    if (st == 0 && d) { std::string r(d); free(d); return r; }
    return s;
}
static std::string base_name(const std::string &p) {
    size_t k = p.rfind('/');
    return k == std::string::npos ? p : p.substr(k + 1);
}
static std::string dir_name(const std::string &p) {
    size_t k = p.rfind('/');
    return k == std::string::npos ? "." : p.substr(0, k);
}
static bool file_exists(const std::string &p) {
    struct stat st;
    return stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}
static std::string short_name(const std::string &d) {
    size_t k = d.find('(');
    return k == std::string::npos ? d : d.substr(0, k);
}

class DebugInfo {
public:
    struct Row { uint32_t addr; int file, line; bool end; };
    std::string elf;
    std::vector<Symbol> funcs, objs;
    std::vector<Row> rows;               // sorted by address
    std::vector<std::string> files;      // global file table
    std::vector<int32_t> word_row;       // flash word -> row index or -1
    std::vector<uint8_t> user_word;      // flash word -> 1 when its line is in user code
    std::vector<uint8_t> file_user;      // per file: 1 = user code, 0 = Arduino/toolchain internals
    struct Region { uint32_t start, size; };
    std::map<std::string, Region> ram_sec; // .data .bss .noinit, as data-space addresses
    std::map<std::string, uint32_t> marks; // __heap_start, __brkval, ... (data-space addresses)
    std::vector<std::string> search;     // extra directories to look for sources
    std::map<int, std::vector<std::string> > src;
    std::map<int, std::string> src_path;

    void clear() {
        elf.clear(); funcs.clear(); objs.clear(); rows.clear(); files.clear();
        word_row.clear(); user_word.clear(); file_user.clear(); src.clear(); src_path.clear(); fidx.clear();
        ram_sec.clear(); marks.clear();
    }
    bool loaded() const { return !elf.empty(); }
    bool has_lines() const { return !rows.empty(); }

    // Arduino core and libraries, avr-libc, GCC runtime, generated sketch wrapper, or no real path
    static bool is_internal(const std::string &p) {
        static const char *pat[] = {"/Arduino15/", "/arduino15/", "/.arduino15/", "/arduino/sketches/",
                                    "/Arduino/libraries/", "/arduino/libraries/", "/arduino-sketch-", "/arduino_build_",
                                    "/avr/include/", "avr-libc", "/libgcc/", "/gcc/avr/"};
        if (p.empty() || p[0] != '/') return true;
        for (size_t i = 0; i < sizeof pat / sizeof pat[0]; i++)
            if (p.find(pat[i]) != std::string::npos) return true;
        return false;
    }
    bool is_user_row(int r) const { return r >= 0 && file_user[rows[r].file]; }

    int row_at(uint32_t byteaddr) const {
        uint32_t w = byteaddr >> 1;
        return w < word_row.size() ? word_row[w] : -1;
    }
    const Symbol *func_at(uint32_t a) const {
        size_t lo = 0, hi = funcs.size();
        while (lo < hi) { size_t m = (lo + hi) / 2; if (funcs[m].addr <= a) lo = m + 1; else hi = m; }
        if (!lo) return 0;
        const Symbol &s = funcs[lo - 1];
        return a < s.addr + std::max<uint32_t>(s.size, 2) ? &s : 0;
    }
    std::string where(uint32_t a) const {
        const Symbol *s = func_at(a);
        if (!s) return "";
        return a == s->addr ? s->name : strf("%s+0x%X", s->name.c_str(), a - s->addr);
    }
    static const Symbol *find_in(const std::vector<Symbol> &v, const std::string &n) {
        for (size_t i = 0; i < v.size(); i++)
            if (v[i].name == n || v[i].mangled == n || short_name(v[i].name) == n) return &v[i];
        return 0;
    }
    std::string loc(int r) const {
        if (r < 0) return "";
        return strf("%s:%d", base_name(files[rows[r].file]).c_str(), rows[r].line);
    }

    // source text of a file (cached); null when it can't be found
    const std::vector<std::string> *source(int f, std::string &path) {
        if (f < 0 || f >= (int)files.size()) return 0;
        std::map<int, std::vector<std::string> >::iterator it = src.find(f);
        if (it == src.end()) {
            std::vector<std::string> cand(1, files[f]);
            for (size_t i = 0; i < search.size(); i++) cand.push_back(search[i] + "/" + base_name(files[f]));
            std::string found;
            for (size_t i = 0; i < cand.size() && found.empty(); i++) if (file_exists(cand[i])) found = cand[i];
            std::vector<std::string> lines;
            if (!found.empty()) {
                FILE *fp = fopen(found.c_str(), "r");
                char buf[4096];
                while (fp && fgets(buf, sizeof buf, fp)) {
                    std::string l;
                    for (char *c = buf; *c && *c != '\n' && *c != '\r'; c++) {
                        if (*c == '\t') l.append(4 - l.size() % 4, ' ');
                        else l.push_back(*c);
                    }
                    lines.push_back(l);
                }
                if (fp) fclose(fp);
            }
            src_path[f] = found;
            it = src.insert(std::make_pair(f, lines)).first;
        }
        path = src_path[f].empty() ? files[f] : src_path[f];
        return src_path[f].empty() ? 0 : &it->second;
    }

    // breakpoint spec: 0xADDR | function | file:line | line (in curfile)
    bool resolve(const std::string &spec, int curfile, uint32_t &addr, std::string &err) const {
        if (spec.size() > 2 && spec[0] == '0' && (spec[1] == 'x' || spec[1] == 'X')) {
            addr = (uint32_t)strtoul(spec.c_str(), 0, 16);
            return true;
        }
        std::string file;
        int line = -1;
        size_t colon = spec.rfind(':');
        bool digits = !spec.empty() && spec.find_first_not_of("0123456789") == std::string::npos;
        if (digits) line = atoi(spec.c_str());
        else if (colon != std::string::npos && colon + 1 < spec.size() &&
                 spec.find_first_not_of("0123456789", colon + 1) == std::string::npos) {
            file = spec.substr(0, colon);
            line = atoi(spec.c_str() + colon + 1);
        }
        if (line > 0) {
            if (file.empty() && curfile < 0) { err = "no current source file for line " + spec; return false; }
            for (int l = line; l < line + 20; l++) {
                uint32_t best = UINT32_MAX;
                for (size_t i = 0; i < rows.size(); i++) {
                    const Row &r = rows[i];
                    if (r.end || r.line != l) continue;
                    bool match = file.empty() ? r.file == curfile
                                              : (files[r.file] == file || base_name(files[r.file]) == base_name(file));
                    if (match && r.addr < best) best = r.addr;
                }
                if (best != UINT32_MAX) { addr = best; return true; }
            }
            err = "no code at " + spec;
            return false;
        }
        const Symbol *s = find_in(funcs, spec);
        if (s) { addr = s->addr; return true; }
        err = "unknown location: " + spec;
        return false;
    }

    bool load(const std::string &path, std::string &err) {
        clear();
        std::vector<uint8_t> b;
        FILE *f = fopen(path.c_str(), "rb");
        if (!f) { err = strf("cannot open %s", path.c_str()); return false; }
        uint8_t tmp[65536];
        size_t n;
        while ((n = fread(tmp, 1, sizeof tmp, f)) > 0) b.insert(b.end(), tmp, tmp + n);
        fclose(f);
        if (b.size() < 52 || memcmp(&b[0], "\x7f" "ELF", 4) || b[4] != 1) { err = "not an ELF32 file"; return false; }
        auto u16 = [&](size_t o) -> uint32_t { return o + 2 <= b.size() ? (b[o] | b[o + 1] << 8) : 0; };
        auto u32 = [&](size_t o) -> uint32_t { return u16(o) | u16(o + 2) << 16; };
        struct Sec { uint32_t name, type, addr, off, size, link; std::string sname; };
        std::vector<Sec> sec;
        uint32_t shoff = u32(32), shes = u16(46), shn = u16(48), shstr = u16(50);
        if (!shes || shoff + (uint64_t)shn * shes > b.size()) { err = "bad ELF section table"; return false; }
        for (uint32_t i = 0; i < shn; i++) {
            size_t h = shoff + i * shes;
            Sec s = {u32(h), u32(h + 4), u32(h + 12), u32(h + 16), u32(h + 20), u32(h + 24), ""};
            if (s.type != 8 && (uint64_t)s.off + s.size > b.size()) s.size = 0; // SHT_NOBITS (.bss) has no file data
            sec.push_back(s);
        }
        auto cstr = [&](const Sec *s, uint64_t o) -> std::string {
            if (!s || o >= s->size) return "";
            const char *p = (const char *)&b[s->off + o];
            return std::string(p, strnlen(p, s->size - o));
        };
        if (shstr < sec.size())
            for (size_t i = 0; i < sec.size(); i++) sec[i].sname = cstr(&sec[shstr], sec[i].name);
        for (size_t i = 0; i < sec.size(); i++) {
            const Sec &x = sec[i];
            if ((x.sname == ".data" || x.sname == ".bss" || x.sname == ".noinit") && x.addr >= 0x800000 && x.addr < 0x810000) {
                Region r = {x.addr - 0x800000, x.size};
                ram_sec[x.sname] = r;
            }
        }
        auto find = [&](const char *nm) -> const Sec * {
            for (size_t i = 0; i < sec.size(); i++) if (sec[i].sname == nm && sec[i].size) return &sec[i];
            return 0;
        };

        // symbols
        for (size_t i = 0; i < sec.size(); i++) {
            if (sec[i].type != 2 || sec[i].link >= sec.size()) continue; // SHT_SYMTAB
            const Sec &st = sec[i], &ss = sec[sec[i].link];
            for (uint32_t o = st.off; o + 16 <= st.off + st.size; o += 16) {
                uint32_t val = u32(o + 4), sz = u32(o + 8);
                int type = b[o + 12] & 0xF;
                std::string nm = cstr(&ss, u32(o));
                if (nm.empty()) continue;
                Symbol s = {val, sz, demangle(nm), nm};
                if (val >= 0x800000 && val < 0x810000 && (nm == "__heap_start" || nm == "__brkval" || nm == "__malloc_margin" ||
                                                          nm == "__malloc_heap_start" || nm == "__malloc_heap_end"))
                    marks[nm] = val - 0x800000;
                if (type == 2 && val < 0x800000) funcs.push_back(s);
                else if (type == 1 && val >= 0x800000 && val < 0x810000) { s.addr = val - 0x800000; objs.push_back(s); }
            }
        }
        std::sort(funcs.begin(), funcs.end(), [](const Symbol &a, const Symbol &c) { return a.addr < c.addr || (a.addr == c.addr && a.size > c.size); });
        funcs.erase(std::unique(funcs.begin(), funcs.end(), [](const Symbol &a, const Symbol &c) { return a.addr == c.addr; }), funcs.end());

        // line tables
        const Sec *dl = find(".debug_line");
        auto blob = [&](const Sec *s) { Blob o = {s ? &b[s->off] : 0, s ? s->size : 0}; return o; };
        if (dl) parse_lines(&b[dl->off], dl->size, blob(find(".debug_line_str")), blob(find(".debug_str")));
        std::stable_sort(rows.begin(), rows.end(), [](const Row &a, const Row &c) { return a.addr < c.addr || (a.addr == c.addr && a.end && !c.end); });
        for (size_t i = 0; i < files.size(); i++) file_user.push_back(!is_internal(files[i]));
        word_row.assign(FLASH_WORDS, -1);
        for (size_t i = 0; i < rows.size();) {
            // rows sharing an address (inlined code: loop() into main(), millis() into loop()):
            // the last one in user code wins, otherwise the last one
            size_t j = i, pick = SIZE_MAX, last = SIZE_MAX;
            for (; j < rows.size() && rows[j].addr == rows[i].addr; j++) {
                if (rows[j].end || rows[j].file < 0) continue;
                last = j;
                if (file_user[rows[j].file]) pick = j;
            }
            if (pick == SIZE_MAX) pick = last;
            if (pick != SIZE_MAX) {
                uint32_t a1 = j < rows.size() ? rows[j].addr : rows[i].addr + 2;
                for (uint32_t a = rows[i].addr; a < a1 && a / 2 < (uint32_t)FLASH_WORDS; a += 2) word_row[a / 2] = (int32_t)pick;
            }
            i = j;
        }
        user_word.assign(FLASH_WORDS, 0);
        for (int w = 0; w < FLASH_WORDS; w++) user_word[w] = is_user_row(word_row[w]);
        elf = path;
        return true;
    }

private:
    std::map<std::string, int> fidx;
    int file_id(const std::string &p) {
        std::map<std::string, int>::iterator it = fidx.find(p);
        if (it != fidx.end()) return it->second;
        files.push_back(p);
        return fidx[p] = (int)files.size() - 1;
    }

    struct Rd {
        const uint8_t *p, *e;
        uint8_t u8() { return p < e ? *p++ : (p = e, 0); }
        uint32_t u16() { uint32_t v = u8(); return v | (uint32_t)u8() << 8; }
        uint32_t u32() { uint32_t v = u16(); return v | u16() << 16; }
        uint64_t u64() { uint64_t v = u32(); return v | (uint64_t)u32() << 32; }
        uint64_t uleb() { uint64_t r = 0; int s = 0; uint8_t c; do { c = u8(); if (s < 64) r |= (uint64_t)(c & 0x7F) << s; s += 7; } while ((c & 0x80) && p < e); return r; }
        int64_t sleb() { int64_t r = 0; int s = 0; uint8_t c; do { c = u8(); if (s < 64) r |= (int64_t)(c & 0x7F) << s; s += 7; } while ((c & 0x80) && p < e); if (s < 64 && (c & 0x40)) r |= -((int64_t)1 << s); return r; }
        std::string str() { std::string s; while (p < e && *p) s += (char)*p++; if (p < e) p++; return s; }
        void skip(uint64_t n) { p = (uint64_t)(e - p) < n ? e : p + n; }
    };

    struct Blob {
        const uint8_t *p;
        uint32_t n;
        std::string at(uint64_t o) const {
            if (!p || o >= n) return "";
            const char *c = (const char *)p + o;
            return std::string(c, strnlen(c, n - o));
        }
    };

    void parse_lines(const uint8_t *data, uint32_t size, Blob lstr, Blob dstr) {
        Rd all = {data, data + size};
        while (all.p + 4 <= all.e) {
            uint64_t len = all.u32();
            bool off64 = false;
            if (len == 0xFFFFFFFFu) { len = all.u64(); off64 = true; }
            if (len > (uint64_t)(all.e - all.p)) break;
            const uint8_t *end = all.p + len;
            Rd r = {all.p, end};
            all.p = end;
            int ver = r.u16();
            if (ver < 2 || ver > 5) continue;
            if (ver >= 5) { r.u8(); r.u8(); }
            uint64_t hlen = off64 ? r.u64() : r.u32();
            const uint8_t *prog = r.p + hlen;
            if (prog > end) continue;
            int minlen = r.u8();
            if (ver >= 4) r.u8();
            bool def_stmt = r.u8();
            int lbase = (int8_t)r.u8();
            int lrange = r.u8(), obase = r.u8();
            if (!lrange) continue;
            std::vector<int> oplen(obase > 0 ? obase : 1, 0);
            for (int i = 1; i < obase; i++) oplen[i] = r.u8();
            std::vector<std::string> dirs;
            std::vector<int> fmap;
            auto join = [](const std::string &d, const std::string &f) {
                return (f.empty() || f[0] == '/' || d.empty()) ? f : d + "/" + f;
            };
            if (ver >= 5) {
                // entry-format driven tables
                auto table = [&](std::vector<std::pair<std::string, uint64_t> > &out) {
                    int nf = r.u8();
                    std::vector<std::pair<uint64_t, uint64_t> > fmt;
                    for (int i = 0; i < nf; i++) { uint64_t ct = r.uleb(); fmt.push_back(std::make_pair(ct, r.uleb())); }
                    uint64_t cnt = r.uleb();
                    for (uint64_t k = 0; k < cnt && r.p < r.e; k++) {
                        std::string path;
                        uint64_t dir = 0;
                        for (size_t j = 0; j < fmt.size(); j++) {
                            std::string s;
                            uint64_t num = 0;
                            switch (fmt[j].second) {
                            case 0x08: s = r.str(); break;                                                     // string
                            case 0x1f: s = lstr.at(off64 ? r.u64() : r.u32()); break;                         // line_strp
                            case 0x0e: s = dstr.at(off64 ? r.u64() : r.u32()); break;                         // strp
                            case 0x0f: num = r.uleb(); break;
                            case 0x0b: num = r.u8(); break;
                            case 0x05: num = r.u16(); break;
                            case 0x06: num = r.u32(); break;
                            case 0x07: num = r.u64(); break;
                            case 0x1e: r.skip(16); break;
                            case 0x09: r.skip(r.uleb()); break;
                            default: r.p = r.e; break; // unsupported form: give up on this unit
                            }
                            if (fmt[j].first == 1) path = s;
                            else if (fmt[j].first == 2) dir = num;
                        }
                        out.push_back(std::make_pair(path, dir));
                    }
                };
                std::vector<std::pair<std::string, uint64_t> > d5, f5;
                table(d5);
                for (size_t i = 0; i < d5.size(); i++) dirs.push_back(d5[i].first);
                table(f5);
                for (size_t i = 0; i < f5.size(); i++)
                    fmap.push_back(file_id(join(f5[i].second < dirs.size() ? dirs[f5[i].second] : "", f5[i].first)));
            } else {
                dirs.push_back("");
                for (;;) { std::string d = r.str(); if (d.empty()) break; dirs.push_back(d); }
                fmap.push_back(-1);
                for (;;) {
                    std::string fn = r.str();
                    if (fn.empty()) break;
                    uint64_t di = r.uleb(); r.uleb(); r.uleb();
                    fmap.push_back(file_id(join(di < dirs.size() ? dirs[di] : "", fn)));
                }
            }
            // line number program
            r.p = prog;
            uint64_t addr = 0;
            int file = 1, line = 1;
            bool stmt = def_stmt;
            (void)stmt;
            auto emit = [&](bool endseq) {
                int g = (file >= 0 && file < (int)fmap.size()) ? fmap[file] : -1;
                if (g >= 0 || endseq) { Row w = {(uint32_t)addr, g, line, endseq}; rows.push_back(w); }
            };
            while (r.p < r.e) {
                int op = r.u8();
                if (op >= obase) {
                    int adj = op - obase;
                    addr += (uint64_t)(adj / lrange) * minlen;
                    line += lbase + adj % lrange;
                    emit(false);
                } else if (op == 0) {
                    uint64_t l = r.uleb();
                    const uint8_t *next = r.p + l;
                    if (next > r.e) break;
                    int sub = l ? r.u8() : 0;
                    if (sub == 1) { emit(true); addr = 0; file = 1; line = 1; stmt = def_stmt; }
                    else if (sub == 2) { addr = 0; for (uint64_t k = 0; k + 1 < l && k < 8; k++) addr |= (uint64_t)r.u8() << (8 * k); }
                    r.p = next;
                } else switch (op) {
                    case 1: emit(false); break;
                    case 2: addr += r.uleb() * minlen; break;
                    case 3: line += (int)r.sleb(); break;
                    case 4: file = (int)r.uleb(); break;
                    case 5: r.uleb(); break;
                    case 6: stmt = !stmt; break;
                    case 7: break;
                    case 8: addr += (uint64_t)((255 - obase) / lrange) * minlen; break;
                    case 9: addr += r.u16(); break;
                    default: { for (int k = 0; k < oplen[op]; k++) r.uleb(); } break;
                }
            }
        }
    }
};

// ============================================================================
//  Disassembler
// ============================================================================

static std::string io_name(int a) {
    static const struct { int a; const char *n; } t[] = {
        {0x23, "PINB"}, {0x24, "DDRB"}, {0x25, "PORTB"}, {0x26, "PINC"}, {0x27, "DDRC"}, {0x28, "PORTC"},
        {0x29, "PIND"}, {0x2A, "DDRD"}, {0x2B, "PORTD"}, {0x35, "TIFR0"}, {0x36, "TIFR1"}, {0x37, "TIFR2"},
        {0x3B, "PCIFR"}, {0x3C, "EIFR"}, {0x3D, "EIMSK"}, {0x3E, "GPIOR0"}, {0x3F, "EECR"}, {0x40, "EEDR"},
        {0x41, "EEARL"}, {0x42, "EEARH"}, {0x44, "TCCR0A"}, {0x45, "TCCR0B"}, {0x46, "TCNT0"}, {0x47, "OCR0A"},
        {0x48, "OCR0B"}, {0x4C, "SPCR"}, {0x4D, "SPSR"}, {0x4E, "SPDR"}, {0x53, "SMCR"}, {0x54, "MCUSR"},
        {0x55, "MCUCR"}, {0x5D, "SPL"}, {0x5E, "SPH"}, {0x5F, "SREG"}};
    for (size_t i = 0; i < sizeof t / sizeof t[0]; i++) if (t[i].a == a) return t[i].n;
    return strf("0x%02X", a - 0x20);
}

static int disasm(const Nano &m, uint16_t a, std::string &o, const DebugInfo *dbg = 0) {
    auto sym = [&](uint32_t byteaddr) -> std::string {
        if (!dbg) return "";
        std::string w = dbg->where(byteaddr);
        return w.empty() ? "" : " <" + w + ">";
    };
    uint16_t op = m.flash[a & PC_MASK], nx = m.flash[(a + 1) & PC_MASK];
    int d = (op >> 4) & 0x1F, r = (op & 0xF) | ((op >> 5) & 0x10), dh = 16 + ((op >> 4) & 0xF);
    int K = ((op >> 4) & 0xF0) | (op & 0xF);
    static const char *alu[4][4] = {{"", "cpc", "sbc", "add"}, {"cpse", "cp", "sub", "adc"},
                                    {"and", "eor", "or", "mov"}, {"", "", "", ""}};
    static const char *imm[5] = {"cpi", "sbci", "subi", "ori", "andi"};
    static const char *one[8] = {"com", "neg", "swap", "inc", "?", "asr", "lsr", "ror"};
    static const char *brs[2][8] = {{"brcs", "breq", "brmi", "brvs", "brlt", "brhs", "brts", "brie"},
                                    {"brcc", "brne", "brpl", "brvc", "brge", "brhc", "brtc", "brid"}};
    static const char *fl = "cznvshti";
    int words = 1;
    switch (op >> 12) {
    case 0x0: case 0x1: case 0x2: {
        int g = op >> 12, s = (op >> 10) & 3;
        if (g == 0 && s == 0) {
            if (op == 0) o = "nop";
            else if ((op >> 8) == 1) o = strf("movw r%d,r%d", ((op >> 4) & 0xF) * 2, (op & 0xF) * 2);
            else if ((op >> 8) == 2) o = strf("muls r%d,r%d", dh, 16 + (op & 0xF));
            else {
                static const char *mm[4] = {"mulsu", "fmul", "fmuls", "fmulsu"};
                o = strf("%s r%d,r%d", mm[((op >> 6) & 2) | ((op >> 3) & 1)], 16 + ((op >> 4) & 7), 16 + (op & 7));
            }
        } else if (d == r && g == 0 && s == 3) o = strf("lsl r%d", d);
        else if (d == r && g == 1 && s == 3) o = strf("rol r%d", d);
        else if (d == r && g == 2 && s == 1) o = strf("clr r%d", d);
        else if (d == r && g == 2 && s == 0) o = strf("tst r%d", d);
        else o = strf("%s r%d,r%d", alu[g][s], d, r);
        break;
    }
    case 0x3: case 0x4: case 0x5: case 0x6: case 0x7:
        o = strf("%s r%d,0x%02X", imm[(op >> 12) - 3], dh, K);
        break;
    case 0x8: case 0xA: {
        int q = (op & 7) | ((op >> 7) & 0x18) | ((op >> 8) & 0x20);
        char p = (op & 8) ? 'Y' : 'Z';
        std::string ptr = q ? strf("%c+%d", p, q) : std::string(1, p);
        o = (op & 0x200) ? strf("st%s %s,r%d", q ? "d" : "", ptr.c_str(), d) : strf("ld%s r%d,%s", q ? "d" : "", d, ptr.c_str());
        break;
    }
    case 0x9: {
        int g = (op >> 9) & 7, lo = op & 0xF;
        if (g <= 1) {
            bool st = g;
            static const char *pm[16] = {"", "Z+", "-Z", "", "Z", "Z+", "Z", "Z+", "", "Y+", "-Y", "", "X", "X+", "-X", ""};
            if (lo == 0) { o = st ? strf("sts 0x%04X,r%d", nx, d) : strf("lds r%d,0x%04X", d, nx); words = 2; }
            else if (lo == 0xF) o = strf("%s r%d", st ? "push" : "pop", d);
            else if (lo >= 4 && lo <= 7 && !st) o = strf("%s r%d,%s", lo < 6 ? "lpm" : "elpm", d, pm[lo]);
            else if (*pm[lo]) o = st ? strf("st %s,r%d", pm[lo], d) : strf("ld r%d,%s", d, pm[lo]);
            else o = strf(".word 0x%04X", op);
        } else if (g == 2) {
            if (lo < 8 && lo != 4) o = strf("%s r%d", one[lo], d);
            else if (lo == 0xA) o = strf("dec r%d", d);
            else if (lo >= 0xC) {
                uint32_t k = ((uint32_t)(((op >> 3) & 0x3E) | (op & 1)) << 16) | nx;
                o = strf("%s 0x%04X", (lo & 2) ? "call" : "jmp", k * 2) + sym(k * 2);
                words = 2;
            } else if (lo == 8 && !(op & 0x100)) o = strf("%s%c", (op & 0x80) ? "cl" : "se", fl[(op >> 4) & 7]);
            else switch (op) {
                case 0x9508: o = "ret"; break;
                case 0x9518: o = "reti"; break;
                case 0x9588: o = "sleep"; break;
                case 0x9598: o = "break"; break;
                case 0x95A8: o = "wdr"; break;
                case 0x95C8: o = "lpm"; break;
                case 0x95E8: o = "spm"; break;
                case 0x9409: o = "ijmp"; break;
                case 0x9509: o = "icall"; break;
                default: o = strf(".word 0x%04X", op);
            }
        } else if (g == 3) o = strf("%s r%d,%d", (op & 0x100) ? "sbiw" : "adiw", 24 + ((op >> 3) & 6), ((op >> 2) & 0x30) | (op & 0xF));
        else if (g <= 5) {
            static const char *bi[4] = {"cbi", "sbic", "sbi", "sbis"};
            o = strf("%s %s,%d", bi[(op >> 8) & 3], io_name(0x20 + ((op >> 3) & 0x1F)).c_str(), op & 7);
        } else o = strf("mul r%d,r%d", d, r);
        break;
    }
    case 0xB: {
        std::string io = io_name(0x20 + (((op >> 5) & 0x30) | (op & 0xF)));
        o = (op & 0x800) ? strf("out %s,r%d", io.c_str(), d) : strf("in r%d,%s", d, io.c_str());
        break;
    }
    case 0xC: case 0xD: {
        int k = (int16_t)(op << 4) >> 4;
        uint32_t t = ((a + 1 + k) & PC_MASK) * 2;
        o = strf("%s 0x%04X", (op >> 12) == 0xC ? "rjmp" : "rcall", t) + ((op >> 12) == 0xD ? sym(t) : std::string());
        break;
    }
    case 0xE: o = K == 0xFF ? strf("ser r%d", dh) : strf("ldi r%d,0x%02X", dh, K); break;
    case 0xF: {
        int s = (op >> 10) & 3, b = op & 7;
        if (s <= 1) o = strf("%s 0x%04X", brs[s][b], ((a + 1 + ((int8_t)(((op >> 3) & 0x7F) << 1) >> 1)) & PC_MASK) * 2);
        else if (s == 2) o = strf("%s r%d,%d", (op & 0x200) ? "bst" : "bld", d, b);
        else o = strf("%s r%d,%d", (op & 0x200) ? "sbrs" : "sbrc", d, b);
        break;
    }
    }
    return words;
}

// ============================================================================
//  Terminal UI
// ============================================================================

enum { C_BORDER = 1, C_TITLE, C_SEL, C_PCB, C_CHIP, C_USB, C_LEDON, C_LEDOFF, C_KEY, C_WARN, C_DIMTXT, C_SCOPEBG };
enum { K_HI, K_LO, K_PWM, K_IN, K_INHI, K_DIM, K_PWR, K_SER, NK };
static int kp(int k, bool pcb) { return 20 + k + (pcb ? NK : 0); }

struct HPin { const char *lbl; int pin; };
static const HPin TOP_HDR[15] = {{"D12", 12}, {"D11", 11}, {"D10", 10}, {"D9", 9}, {"D8", 8}, {"D7", 7}, {"D6", 6},
                                 {"D5", 5}, {"D4", 4}, {"D3", 3}, {"D2", 2}, {"GND", -1}, {"RST", -2}, {"RX0", 0}, {"TX1", 1}};
static const HPin BOT_HDR[15] = {{"D13", 13}, {"3V3", -3}, {"REF", -4}, {"A0", 14}, {"A1", 15}, {"A2", 16}, {"A3", 17},
                                 {"A4", 18}, {"A5", 19}, {"A6", -10}, {"A7", -11}, {"5V", -5}, {"RST", -2}, {"GND", -1}, {"VIN", -6}};

static const double SPEEDS[] = {0.001, 0.01, 0.1, 0.25, 0.5, 1, 2, 4, 10, 0};
static const int NSPEEDS = sizeof SPEEDS / sizeof SPEEDS[0];
static const double TIMEBASES[] = {1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000}; // us/column
static const int NTB = sizeof TIMEBASES / sizeof TIMEBASES[0];

class App {
public:
    Nano m;
    std::string fw_path;
    int fw_size = 0;
    bool quit = false;
    int speed_i = 5, tb_i = 6;
    bool trigger = false;
    enum Focus { F_PINS, F_SCOPE } focus = F_PINS; // TAB switches where the arrow keys go
    int tcur = -1;                                  // analyzer time cursor: columns left of "now", -1 = hidden
    int sel = 13;
    enum Mode { NORMAL, SERIAL, PROMPT_FILE, PROMPT_BREAK } mode = NORMAL;
    // debugger
    DebugInfo dbg;
    std::vector<std::string> bp_specs, src_dirs;
    bool show_source = true;
    bool show_internal = false; // 'a': also show Arduino/toolchain code in the SOURCE panel
    // 's': a tap steps one instruction; holding it runs at the current speed. A terminal sends no key-release,
    // so holding is seen through the keyboard's auto-repeat: presses closer than HOLD_GAP apart.
    static constexpr double HOLD_GAP = 0.20, RELEASE_GAP = 0.25;
    double s_last = -1;
    bool s_hold = false, s_block = false; // running while held / ignoring repeats after a breakpoint
    int last_row = -1;
    std::string input;
    std::string msg;
    double msg_t = 0;
    std::string stext; // serial console text
    double led_tx = -1, led_rx = -1, now = 0;
    double rate = 0, rate_t = 0;
    uint64_t rate_c = 0;
    int ptm = -1, pts = -1;
    std::string ptyname;
    bool headless = false;

    void set_msg(const std::string &s) { msg = s; msg_t = mono(); }

    bool load(const std::string &path) {
        std::string err;
        int size = 0;
        if (!m.load(path, err, size)) { set_msg("load failed: " + err); return false; }
        fw_path = path;
        fw_size = size;
        m.reset(0x01, true);
        m.paused = false;
        stext.clear();
        std::string info = load_symbols(path);
        set_msg(strf("loaded %s (%d bytes)%s", path.c_str(), size, info.c_str()));
        return true;
    }

    // ------------------------------------------------------------ debugger
    static bool is_elf(const std::string &p) {
        FILE *f = fopen(p.c_str(), "rb");
        char h[4] = {0, 0, 0, 0};
        if (f) { size_t n = fread(h, 1, 4, f); (void)n; fclose(f); }
        return !memcmp(h, "\x7f" "ELF", 4);
    }
    // symbols/lines from the .elf itself, or from a sibling .elf of a .hex/.bin
    std::string load_symbols(const std::string &path) {
        m.user_map = 0;
        dbg.clear();
        last_row = -1;
        std::string ep = path;
        if (!is_elf(ep)) {
            size_t dot = path.rfind('.');
            ep = (dot == std::string::npos ? path : path.substr(0, dot)) + ".elf";
            if (!file_exists(ep) || !is_elf(ep)) ep.clear();
        }
        std::string info;
        if (!ep.empty()) {
            std::string err;
            if (dbg.load(ep, err))
                info = strf(", %zu symbols, %zu lines%s", dbg.funcs.size() + dbg.objs.size(), dbg.rows.size(),
                            ep != path ? strf(" from %s", base_name(ep).c_str()).c_str() : "");
            else info = ", no debug info: " + err;
        }
        if (!dbg.user_word.empty()) m.user_map = &dbg.user_word[0];
        dbg.search = src_dirs;
        dbg.search.push_back(dir_name(path));
        apply_breakpoints();
        return info;
    }
    int cur_file() const {
        int r = dbg.row_at(m.pc * 2);
        if (r < 0) r = last_row;
        return r >= 0 ? dbg.rows[r].file : -1;
    }
    std::string apply_breakpoints() {
        std::fill(m.bpw.begin(), m.bpw.end(), 0);
        std::string errs;
        for (size_t i = 0; i < bp_specs.size(); i++) {
            uint32_t a;
            std::string e;
            if (dbg.resolve(bp_specs[i], cur_file(), a, e)) m.bpw[(a / 2) & PC_MASK] = 1;
            else errs += (errs.empty() ? "" : "; ") + e;
        }
        return errs;
    }
    int bp_count() const { int n = 0; for (size_t i = 0; i < m.bpw.size(); i++) n += m.bpw[i]; return n; }
    void toggle_breakpoint(const std::string &spec) {
        uint32_t a;
        std::string e;
        if (!dbg.resolve(spec, cur_file(), a, e)) { set_msg("breakpoint: " + e); return; }
        uint16_t w = (a / 2) & PC_MASK;
        if (m.bpw[w]) { // remove every spec that lands on this address
            std::vector<std::string> keep;
            for (size_t i = 0; i < bp_specs.size(); i++) {
                uint32_t b2;
                std::string e2;
                if (!(dbg.resolve(bp_specs[i], cur_file(), b2, e2) && ((b2 / 2) & PC_MASK) == w)) keep.push_back(bp_specs[i]);
            }
            bp_specs = keep;
            m.bpw[w] = 0;
            set_msg(strf("breakpoint removed at 0x%04X", w * 2));
        } else {
            // store file:line specs with the file name so they survive a change of current file
            std::string sp = spec;
            if (spec.find_first_not_of("0123456789") == std::string::npos && cur_file() >= 0)
                sp = base_name(dbg.files[cur_file()]) + ":" + spec;
            bp_specs.push_back(sp);
            m.bpw[w] = 1;
            std::string at = dbg.where(w * 2), ln = dbg.loc(dbg.row_at(w * 2));
            set_msg(strf("breakpoint at 0x%04X %s %s", w * 2, at.c_str(), ln.c_str()));
        }
    }
    void restart() { m.reset(0x01, true); m.paused = false; set_msg("restarted (hard reset)"); }

    // --------------------------------------------------------------- serial
    void drain_tx() {
        if (m.txout.empty()) return;
        led_tx = now;
        if (headless) { fwrite(m.txout.data(), 1, m.txout.size(), stdout); fflush(stdout); }
        if (ptm >= 0) { ssize_t w = write(ptm, m.txout.data(), m.txout.size()); (void)w; }
        for (size_t i = 0; i < m.txout.size(); i++) {
            unsigned char c = m.txout[i];
            if (c == '\r') continue;
            stext.push_back((c == '\n' || c == '\t' || (c >= 32 && c < 127)) ? (char)c : '.');
        }
        if (stext.size() > 65536) stext.erase(0, stext.size() - 49152);
        m.txout.clear();
    }
    void send_serial(const std::string &s) {
        for (size_t i = 0; i < s.size(); i++) m.rxq.push_back((uint8_t)s[i]);
        led_rx = now;
    }
    void pty_open() {
        char name[256];
        if (openpty(&ptm, &pts, name, NULL, NULL) < 0) { ptm = pts = -1; return; }
        termios t;
        tcgetattr(pts, &t);
        cfmakeraw(&t);
        tcsetattr(pts, TCSANOW, &t);
        fcntl(ptm, F_SETFL, fcntl(ptm, F_GETFL) | O_NONBLOCK);
        ptyname = name;
    }
    void pty_poll() {
        if (ptm < 0) return;
        char b[512];
        ssize_t n = read(ptm, b, sizeof b);
        if (n > 0) send_serial(std::string(b, n));
    }

    // ------------------------------------------------------------- drawing
    void put(int y, int x, int maxw, int attr, const std::string &s) {
        if (maxw <= 0 || y < 0 || y >= LINES) return;
        // clip to maxw columns, counting UTF-8 characters, not bytes
        size_t n = 0;
        for (int cols = 0; n < s.size() && cols < maxw; cols++) {
            n++;
            while (n < s.size() && ((unsigned char)s[n] & 0xC0) == 0x80) n++;
        }
        attron(attr);
        mvaddnstr(y, x, s.c_str(), (int)n);
        attroff(attr);
    }
    void uput(int y, int x, int attr, const char *s) { attron(attr); mvaddstr(y, x, s); attroff(attr); }

    void box_at(int y, int x, int h, int w, const std::string &title, bool focus = false) {
        if (h < 2 || w < 2) return;
        int a = COLOR_PAIR(focus ? C_WARN : C_BORDER) | (focus ? A_BOLD : 0);
        attron(a);
        mvhline(y, x + 1, ACS_HLINE, w - 2);
        mvhline(y + h - 1, x + 1, ACS_HLINE, w - 2);
        mvvline(y + 1, x, ACS_VLINE, h - 2);
        mvvline(y + 1, x + w - 1, ACS_VLINE, h - 2);
        mvaddch(y, x, ACS_ULCORNER);
        mvaddch(y, x + w - 1, ACS_URCORNER);
        mvaddch(y + h - 1, x, ACS_LLCORNER);
        mvaddch(y + h - 1, x + w - 1, ACS_LRCORNER);
        attroff(a);
        if (!title.empty()) put(y, x + 2, w - 4, COLOR_PAIR(C_TITLE) | A_BOLD, " " + title + " ");
    }

    int pin_kind(int pin) {
        if (pin < 0) return (pin == -3 || pin == -5 || pin == -6) ? K_PWR : K_DIM;
        std::string md = m.pin_mode(pin);
        if (md == "PWM") return K_PWM;
        if (md == "TX" || md == "RX") return K_SER;
        if (md == "OUT") return m.level(pin) ? K_HI : K_LO;
        return m.level(pin) ? K_INHI : K_IN;
    }

    void draw_header(int y, int x0, int sp, const HPin *h, int lblrow, int pinrow, int valrow, bool top) {
        for (int i = 0; i < 15; i++) {
            int px = x0 + i * sp, pin = h[i].pin;
            int k = pin_kind(pin);
            const char *glyph = (pin >= 0 && m.level(pin)) ? "●" : (pin >= 0 ? "○" : "■");
            if (pin == -10 || pin == -11) glyph = "○";
            uput(pinrow, px, COLOR_PAIR(kp(k, true)) | A_BOLD, glyph);
            int len = strlen(h[i].lbl);
            put(lblrow, px - (len - 1) / 2, len, COLOR_PAIR(kp(k, false)) | (pin == sel ? A_REVERSE : 0), h[i].lbl);
            std::string v;
            if (!top && (pin >= 14 || pin <= -10)) {
                int ch = pin >= 14 ? pin - 14 : (pin == -10 ? 6 : 7);
                v = strf("%.1fV", m.adcv[ch] * 5.0 / 1023);
            } else if (pin >= 0 && m.pwm_active(pin)) {
                double f, du;
                m.measure(pin, m.cycles > 160000 ? m.cycles - 160000 : 0, m.cycles, f, du);
                v = strf("%d%%", (int)lround(du * 100));
            } else if (pin == 0 || pin == 1) v = pin ? "TX" : "RX";
            if (!v.empty()) put(valrow, px - ((int)v.size() - 1) / 2, sp, COLOR_PAIR(kp(pin_kind(pin), false)) | A_DIM, v);
        }
    }

    void draw_board(int y, int x, int w, int h) {
        box_at(y, x, h, w, "ARDUINO NANO  ATmega328P @ 16 MHz");
        int sp = std::max(3, std::min(5, (w - 14) / 15));
        int bw = 15 * sp + 4;
        int bx = x + (w - bw) / 2 + 2;
        int px0 = bx + 3;
        int by = y + 1;
        // PCB rows by+2 .. by+8
        for (int r = 0; r < 7; r++) put(by + 2 + r, bx, bw, COLOR_PAIR(C_PCB), std::string(bw, ' '));
        // USB connector
        for (int r = 0; r < 3; r++) put(by + 4 + r, bx - 4, 6, COLOR_PAIR(C_USB), r == 1 ? " USB  " : "      ");
        // chip
        int cx = bx + bw / 2 - 7;
        put(by + 4, cx, 14, COLOR_PAIR(C_CHIP), "              ");
        put(by + 5, cx, 14, COLOR_PAIR(C_CHIP) | A_BOLD, "  ATMEGA328P  ");
        put(by + 6, cx, 14, COLOR_PAIR(C_CHIP), " o            ");
        put(by + 4, cx - 9, 7, COLOR_PAIR(C_CHIP), "[16MHz]");
        // LEDs
        bool L = m.level(13);
        bool tx = now - led_tx < 0.06, rx = now - led_rx < 0.06;
        put(by + 3, bx + 3, 2, COLOR_PAIR(L ? C_LEDON : C_LEDOFF) | A_BOLD, "  ");
        put(by + 3, bx + 6, 2, COLOR_PAIR(C_PCB), "L");
        put(by + 3, bx + 9, 2, COLOR_PAIR(m.paused ? C_LEDOFF : C_LEDON), "  ");
        put(by + 3, bx + 12, 3, COLOR_PAIR(C_PCB), "PWR");
        put(by + 7, bx + 3, 2, COLOR_PAIR(tx ? C_LEDON : C_LEDOFF), "  ");
        put(by + 7, bx + 6, 2, COLOR_PAIR(C_PCB), "TX");
        put(by + 7, bx + 9, 2, COLOR_PAIR(rx ? C_LEDON : C_LEDOFF), "  ");
        put(by + 7, bx + 12, 2, COLOR_PAIR(C_PCB), "RX");
        // reset button & ICSP
        int rx0 = bx + bw - 14;
        put(by + 4, rx0, 5, COLOR_PAIR(C_USB), "     ");
        put(by + 5, rx0, 5, COLOR_PAIR(C_USB) | A_BOLD, " (o) ");
        put(by + 6, rx0, 5, COLOR_PAIR(C_PCB), "RESET");
        for (int r = 0; r < 3; r++) uput(by + 4 + r, bx + bw - 6, COLOR_PAIR(kp(K_DIM, true)), "○ ○");
        put(by + 3, bx + bw - 6, 4, COLOR_PAIR(C_PCB), "ICSP");
        // headers
        draw_header(by + 1, px0, sp, TOP_HDR, by + 1, by + 2, by, true);
        draw_header(by + 9, px0, sp, BOT_HDR, by + 9, by + 8, by + 10, false);
        // legend
        int ly = by + 11;
        if (ly < y + h - 1) {
            int lx = x + 2;
            const struct { int k; const char *t; } lg[] = {{K_HI, "out HIGH"}, {K_LO, "out LOW"}, {K_PWM, "PWM"}, {K_INHI, "in HIGH"}, {K_IN, "in LOW"}, {K_SER, "UART"}};
            for (int i = 0; i < 6; i++) {
                uput(ly, lx, COLOR_PAIR(kp(lg[i].k, false)) | A_BOLD, "●");
                put(ly, lx + 2, 10, COLOR_PAIR(C_DIMTXT), lg[i].t);
                lx += strlen(lg[i].t) + 4;
            }
        }
    }

    // name used in source for a symbol: "loop()::count" -> "count", "count.0" -> "count"
    static std::string var_key(const std::string &n) {
        std::string k = n;
        size_t c = k.rfind("::");
        if (c != std::string::npos) k = k.substr(c + 2);
        size_t d = k.find('.');
        return d == std::string::npos ? k : k.substr(0, d);
    }
    std::string var_value(const Symbol &s) {
        if (s.addr + s.size > (uint32_t)RAM_END || !s.size) return "?";
        const uint8_t *p = &m.ram[s.addr];
        if (s.size <= 4 && s.size != 3) {
            uint32_t v = 0;
            for (uint32_t i = 0; i < s.size; i++) v |= (uint32_t)p[i] << (8 * i);
            return strf("%u (0x%X)", v, v);
        }
        std::string o = "{";
        for (uint32_t i = 0; i < s.size && i < 8; i++) o += strf("%s%02X", i ? " " : "", p[i]);
        return o + (s.size > 8 ? " ...}" : "}");
    }
    // globals/statics named on a source line, with their current values
    std::string line_vars(const std::string &line) {
        std::string out, seen = " ";
        for (size_t i = 0; i < line.size();) {
            if (!(isalpha((unsigned char)line[i]) || line[i] == '_')) { i++; continue; }
            size_t j = i;
            while (j < line.size() && (isalnum((unsigned char)line[j]) || line[j] == '_')) j++;
            std::string id = line.substr(i, j - i);
            i = j;
            if (seen.find(" " + id + " ") != std::string::npos) continue;
            seen += id + " ";
            for (size_t k = 0; k < dbg.objs.size(); k++)
                if (var_key(dbg.objs[k].name) == id) {
                    out += (out.empty() ? "" : "   ") + id + " = " + var_value(dbg.objs[k]);
                    break;
                }
        }
        return out;
    }

    void draw_source(int y, int x, int w, int h) {
        std::string fn = dbg.where(m.pc * 2);
        int r = dbg.row_at(m.pc * 2);
        bool here = r >= 0;
        std::string note;
        if (!show_internal && !dbg.is_user_row(r)) {
            // executing Arduino/toolchain code: stay on the last user line that ran
            here = false;
            r = m.last_user_pc >= 0 ? dbg.row_at(m.last_user_pc * 2) : -1;
            note = "  now in " + (fn.empty() ? strf("0x%04X", m.pc * 2) : fn) + " (a: show)";
            fn.clear();
        } else if (here) last_row = r;
        else { r = last_row; note = " (last line with info)"; }
        if (r < 0) {
            box_at(y, x, h, w, "SOURCE");
            put(y + 1, x + 2, w - 4, COLOR_PAIR(C_DIMTXT),
                show_internal ? strf("no line info at 0x%04X %s", m.pc * 2, fn.c_str())
                              : "no user code has run yet" + note);
            return;
        }
        const DebugInfo::Row &row = dbg.rows[r];
        std::string path;
        const std::vector<std::string> *lines = dbg.source(row.file, path);
        box_at(y, x, h, w, strf("SOURCE %s:%d  %s%s", base_name(path).c_str(), row.line, fn.c_str(), note.c_str()));
        int iw = w - 2, ih = h - 2;
        if (!lines) { put(y + 1, x + 2, iw - 2, COLOR_PAIR(C_WARN), "source not found: " + path + "  (use -S DIR)"); return; }
        std::string vars = row.line >= 1 && row.line <= (int)lines->size() ? line_vars((*lines)[row.line - 1]) : "";
        if (!vars.empty()) ih--;
        int top = std::max(1, std::min(row.line - ih / 2, (int)lines->size() - ih + 1));
        for (int i = 0; i < ih; i++) {
            int ln = top + i;
            if (ln > (int)lines->size()) break;
            bool cur = ln == row.line;
            std::string t = strf("%c%5d  %s", cur ? '>' : ' ', ln, (*lines)[ln - 1].c_str());
            put(y + 1 + i, x + 1, iw, cur ? (COLOR_PAIR(C_SEL) | A_BOLD) : 0, t + (cur ? std::string(std::max(0, iw - (int)t.size()), ' ') : ""));
        }
        if (!vars.empty()) put(y + h - 2, x + 1, iw, COLOR_PAIR(C_WARN) | A_BOLD, " " + vars);
    }

    void draw_serial(int y, int x, int w, int h) {
        std::string t = strf("SERIAL  %.0f baud  TX %llu  RX %llu", (ram_baud()), (unsigned long long)m.us.txn,
                             (unsigned long long)m.us.rxn);
        if (!ptyname.empty()) t += "  pty " + ptyname;
        box_at(y, x, h, w, t, mode == SERIAL);
        int iw = w - 2, ih = h - 3;
        if (iw < 4 || ih < 1) return;
        // wrap text into rows, take the last ih
        std::vector<std::string> rows;
        size_t end = stext.size();
        while ((int)rows.size() < ih) {
            size_t start = end == 0 ? std::string::npos : stext.rfind('\n', end - 1);
            size_t s = start == std::string::npos ? 0 : start + 1;
            std::string line = stext.substr(s, end - s);
            std::vector<std::string> parts;
            for (size_t i = 0; i < line.size() || parts.empty(); i += iw) parts.push_back(line.substr(i, iw));
            for (int i = (int)parts.size() - 1; i >= 0 && (int)rows.size() < ih; i--) rows.push_back(parts[i]);
            if (start == std::string::npos) break;
            end = start;
        }
        for (int i = 0; i < (int)rows.size(); i++) put(y + ih - i, x + 1, iw, COLOR_PAIR(K_HI + 20), rows[i]);
        std::string pr = mode == SERIAL ? "> " + input + "_" : "  i: type to serial   ESC: stop typing   c: clear";
        put(y + h - 2, x + 1, iw, mode == SERIAL ? (COLOR_PAIR(C_WARN) | A_BOLD) : COLOR_PAIR(C_DIMTXT), pr);
    }
    double ram_baud() { return (m.ram[UCSR0B] & 0x18) ? m.baud() : 0; }

    void draw_cpu(int y, int x, int w, int h) {
        box_at(y, x, 9, w, "CPU");
        int iw = w - 2;
        put(y + 1, x + 1, iw, A_BOLD, strf(" PC 0x%04X   SP 0x%04X", m.pc * 2, m.sp()));
        put(y + 2, x + 1, iw, 0, " SREG ");
        const char *fl = "ITHSVNZC";
        for (int i = 0; i < 8; i++) {
            bool on = m.ram[SREG_A] & (0x80 >> i);
            put(y + 2, x + 7 + i * 2, 1, on ? (COLOR_PAIR(kp(K_HI, false)) | A_BOLD) : (COLOR_PAIR(C_DIMTXT) | A_DIM), std::string(1, fl[i]));
        }
        put(y + 3, x + 1, iw, 0, strf(" CYC  %llu", (unsigned long long)m.cycles));
        put(y + 4, x + 1, iw, 0, strf(" TIME %.6f s", m.cycles / F_CPU));
        put(y + 5, x + 1, iw, 0, strf(" INST %llu", (unsigned long long)m.instrs));
        double sp = SPEEDS[speed_i];
        put(y + 6, x + 1, iw, 0, strf(" RATE %.2f MHz %s", rate / 1e6, sp ? strf("(x%g)", sp).c_str() : "(max)"));
        std::string st = m.paused ? "PAUSED" : s_hold ? "RUNNING (s held)" : m.sleeping ? "SLEEP" : "RUNNING";
        put(y + 7, x + 1, iw, COLOR_PAIR(m.paused ? C_WARN : kp(K_HI, false)) | A_BOLD, " " + st);
        int nbp = bp_count();
        if (nbp) put(y + 7, x + 12, iw - 11, COLOR_PAIR(kp(K_LO, false)), strf("%d breakpoint%s", nbp, nbp > 1 ? "s" : ""));

        int ry = y + 9;
        box_at(ry, x, 11, w, "REGISTERS");
        for (int r = 0; r < 8; r++) {
            std::string s;
            for (int c = 0; c < 4; c++) s += strf("%s%02d %02X ", c ? "" : " ", r + c * 8, m.ram[r + c * 8]);
            put(ry + 1 + r, x + 1, iw, 0, s);
        }
        put(ry + 9, x + 1, iw, COLOR_PAIR(C_DIMTXT), strf(" X %04X  Y %04X  Z %04X", m.reg16(26), m.reg16(28), m.reg16(30)));

        int cy = ry + 11, ch = h - 20;
        if (ch < 3) return;
        std::string fn = dbg.where(m.pc * 2);
        box_at(cy, x, ch, w, fn.empty() ? "CODE" : "CODE " + fn);
        uint16_t a = m.pc;
        for (int i = 0; i < ch - 2; i++) {
            const Symbol *s = dbg.func_at(a * 2);
            if (s && s->addr == a * 2u && i > 0) { // function label
                put(cy + 1 + i, x + 1, iw, COLOR_PAIR(C_WARN), " <" + s->name + ">:");
                if (++i >= ch - 2) break;
            }
            std::string dis;
            int n = disasm(m, a, dis, &dbg);
            bool cur = (a == m.pc);
            std::string line = strf("%s%04X %s", m.bpw[a] ? "*" : " ", a * 2, dis.c_str());
            put(cy + 1 + i, x + 1, iw, cur ? (COLOR_PAIR(C_SEL) | A_BOLD) : (m.bpw[a] ? COLOR_PAIR(kp(K_LO, false)) : 0),
                line + (cur ? std::string(std::max(0, iw - (int)line.size()), ' ') : ""));
            a = (a + n) & PC_MASK;
        }
    }

    // ------------------------------------------------------------ memory
    enum { M_FREE, M_PEAK, M_DATA, M_BSS, M_HEAP, M_STACK };
    struct MemInfo {
        int sram, data, bss, heap, stack, peak, used, avail;
        uint32_t data_lo, data_hi, bss_lo, bss_hi, heap_lo, heap_hi, sp;
        bool symbols;
    };
    MemInfo mem_info() const {
        MemInfo r;
        memset(&r, 0, sizeof r);
        r.sram = RAM_END - 0x100;
        r.sp = m.sp();
        r.stack = r.sp < (uint32_t)RAM_END ? (RAM_END - 1) - (int)r.sp : 0;
        r.peak = (RAM_END - 1) - m.min_sp;
        std::map<std::string, DebugInfo::Region>::const_iterator d = dbg.ram_sec.find(".data"),
            b = dbg.ram_sec.find(".bss"), n = dbg.ram_sec.find(".noinit");
        r.symbols = d != dbg.ram_sec.end() || b != dbg.ram_sec.end();
        if (r.symbols) {
            if (d != dbg.ram_sec.end()) { r.data_lo = d->second.start; r.data_hi = d->second.start + d->second.size; }
            r.bss_lo = r.bss_hi = r.data_hi;
            if (b != dbg.ram_sec.end()) { r.bss_lo = b->second.start; r.bss_hi = b->second.start + b->second.size; }
            if (n != dbg.ram_sec.end()) r.bss_hi = std::max(r.bss_hi, n->second.start + n->second.size); // .noinit counted with .bss
            std::map<std::string, uint32_t>::const_iterator hs = dbg.marks.find("__heap_start"), bv = dbg.marks.find("__brkval");
            r.heap_lo = hs != dbg.marks.end() ? hs->second : r.bss_hi;
            uint32_t brk = (bv != dbg.marks.end() && bv->second + 1 < (uint32_t)RAM_END) ? (m.ram[bv->second] | m.ram[bv->second + 1] << 8) : 0;
            r.heap_hi = brk > r.heap_lo && brk < (uint32_t)RAM_END ? brk : r.heap_lo; // __brkval is 0 until the first malloc()
            r.data = r.data_hi - r.data_lo;
            r.bss = r.bss_hi - r.bss_lo;
            r.heap = r.heap_hi - r.heap_lo;
            r.used = r.data + r.bss + r.heap + r.stack;
            r.avail = r.sp + 1 > r.heap_hi ? (int)(r.sp + 1 - r.heap_hi) : 0; // gap between the heap and the stack
        } else {
            int used = 0;
            for (int a = 0x100; a < RAM_END; a++) used += (m.touched[a] || (uint32_t)a > r.sp);
            r.used = used;
            r.avail = r.sram - used;
        }
        return r;
    }
    int mem_class(uint32_t a, const MemInfo &mi) const {
        if (a > mi.sp) return M_STACK;
        if (mi.symbols) {
            if (a >= mi.data_lo && a < mi.data_hi) return M_DATA;
            if (a >= mi.bss_lo && a < mi.bss_hi) return M_BSS;
            if (a >= mi.heap_lo && a < mi.heap_hi) return M_HEAP;
        } else if (m.touched[a]) return M_DATA;
        return a > m.min_sp ? M_PEAK : M_FREE;
    }
    static std::string kb(int bytes) { return bytes >= 1024 && bytes % 1024 == 0 ? strf("%dK", bytes / 1024) : strf("%d", bytes); }

    int memory_height() const { return 10; }
    void draw_memory(int y, int x, int w, int h) {
        bool test = RAM_END != RAM_END_328P || FLASH_WORDS != 16384;
        int iw = w - 2, row = y + 1, last = y + h - 2;
        // block map of the SRAM, lowest address top-left; each cell is a few bytes
        const int cols = std::min(32, iw - 2);
        int rows = 2;
        int sram = RAM_END - 0x100, bpc = (sram + cols * rows - 1) / (cols * rows);
        box_at(y, x, h, w, test ? strf("MEMORY  test RAM %s ROM %s  %dB/cell", kb(sram).c_str(), kb(FLASH_WORDS * 2).c_str(), bpc)
                                : strf("MEMORY  %dB/cell", bpc));
        MemInfo mi = mem_info();
        put(row++, x + 1, iw, A_BOLD, strf(" RAM %d B used %d free %d %d%%", mi.sram, mi.used, mi.avail,
                                           mi.sram ? (int)lround(100.0 * mi.used / mi.sram) : 0));
        static const int kcol[6] = {K_DIM, K_LO, K_INHI, K_IN, K_PWM, K_LO};
        for (int r = 0; r < rows && row <= last; r++, row++) {
            for (int c = 0; c < cols; c++) {
                uint32_t a0 = 0x100 + (uint32_t)(r * cols + c) * bpc;
                if (a0 >= (uint32_t)RAM_END) break;
                int cls = M_FREE;
                for (uint32_t a = a0; a < a0 + bpc && a < (uint32_t)RAM_END; a++) cls = std::max(cls, mem_class(a, mi));
                const char *g = cls == M_FREE ? "\u00B7" : cls == M_PEAK ? "\u2592" : "\u2588";
                int attr = COLOR_PAIR(kp(kcol[cls], false)) | (cls == M_FREE || cls == M_PEAK ? A_DIM : A_BOLD);
                uput(row, x + 2 + c, attr, g);
            }
        }
        if (row <= last) { // legend
            int lx = x + 2;
            const struct { int cls; const char *g, *t; } lg[] = {{M_DATA, "\u2588", mi.symbols ? "data" : "used"}, {M_BSS, "\u2588", "bss"},
                {M_HEAP, "\u2588", "heap"}, {M_STACK, "\u2588", "stack"}, {M_PEAK, "\u2592", "peak"}, {M_FREE, "\u00B7", "free"}};
            for (int i = 0; i < 6; i++) {
                if (!mi.symbols && (lg[i].cls == M_BSS || lg[i].cls == M_HEAP)) continue;
                uput(row, lx, COLOR_PAIR(kp(kcol[lg[i].cls], false)) | A_BOLD, lg[i].g);
                put(row, lx + 1, 6, COLOR_PAIR(C_DIMTXT), lg[i].t);
                lx += 2 + strlen(lg[i].t);
            }
            row++;
        }
        if (row <= last)
            put(row++, x + 1, iw, COLOR_PAIR(C_DIMTXT), mi.symbols
                ? strf(" stack %d (peak %d)  heap %d", mi.stack, mi.peak, mi.heap)
                : strf(" stack %d (peak %d)  no .elf", mi.stack, mi.peak));
        // flash
        int total = FLASH_WORDS * 2, maxsk = total - BOOT_BYTES;
        if (row <= last)
            put(row++, x + 1, iw, A_BOLD, strf(" ROM %d/%d B %d%%  boot %s", fw_size, maxsk,
                                               (int)lround(100.0 * fw_size / maxsk), kb(BOOT_BYTES).c_str()));
        if (row <= last) {
            for (int c = 0; c < cols; c++) {
                int a0 = (int)((int64_t)c * total / cols), a1 = (int)((int64_t)(c + 1) * total / cols);
                bool boot = a0 >= maxsk, used = a0 < fw_size;
                const char *g = boot ? "\u2593" : used ? "\u2588" : "\u00B7";
                int attr = boot ? (COLOR_PAIR(kp(K_DIM, false)) | A_DIM) : used ? (COLOR_PAIR(kp(K_HI, false)) | A_BOLD)
                                                                          : (COLOR_PAIR(kp(K_DIM, false)) | A_DIM);
                (void)a1;
                uput(row, x + 2 + c, attr, g);
            }
            row++;
        }
        if (row <= last) {
            int eu = 0;
            for (int i = 0; i < EE_SIZE; i++) eu += m.eeprom[i] != 0xFF;
            put(row++, x + 1, iw, COLOR_PAIR(C_DIMTXT), strf(" EEPROM %d/%d B used  writes %u", eu, EE_SIZE, m.ee_writes));
        }
    }

    void draw_right(int y, int x, int w, int h) {
        int iw = w - 2;
        int memh = h >= 26 ? memory_height() : 0;
        int ph = std::min(h - 12 - memh, 25);
        if (ph < 5) ph = std::max(3, h - 3 - memh);
        box_at(y, x, ph, w, "PINS  \u2191\u2193 \u23CE \u2190\u2192  TAB", focus == F_PINS);
        put(y + 1, x + 1, iw, COLOR_PAIR(C_DIMTXT) | A_UNDERLINE, " PIN  PRT MODE  LV DRV  ANALOG     ");
        int vis = ph - 3, off = 0;
        if (sel >= vis) off = sel - vis + 1;
        for (int r = 0; r < vis && r + off < 22; r++) {
            int i = r + off;
            std::string s;
            int k;
            if (i < NPINS) {
                const char *drv = m.ext[i] < 0 ? " -" : m.ext[i] ? " H" : " L";
                s = strf("%c%-4s P%c%d %-5s %d  %s", i == sel ? '>' : ' ', PIN_NAME[i], "BCD"[pin_port(i)], pin_bit(i),
                         m.pin_mode(i).c_str(), m.level(i), drv);
                if (i >= 14) s += strf("  %4d %.2fV", m.adcv[i - 14], m.adcv[i - 14] * 5.0 / 1023);
                k = pin_kind(i);
            } else {
                int ch = i - 14;
                s = strf("%cA%d   ADC6/7 only       %4d %.2fV", i == sel ? '>' : ' ', ch, m.adcv[ch], m.adcv[ch] * 5.0 / 1023);
                k = K_IN;
            }
            int attr = COLOR_PAIR(kp(k, false));
            if (i == sel) attr = COLOR_PAIR(C_SEL) | A_BOLD;
            put(y + 2 + r, x + 1, iw, attr, s + (i == sel ? std::string(std::max(0, iw - (int)s.size()), ' ') : ""));
        }

        if (memh) draw_memory(y + ph, x, w, memh);
        int py = y + ph + memh, phh = h - ph - memh;
        if (phh < 3) return;
        box_at(py, x, phh, w, "PERIPHERALS");
        std::vector<std::pair<std::string, int> > L;
        static const char *kn[4] = {"NORM", "CTC", "FPWM", "PCPWM"};
        for (int i = 0; i < 3; i++) {
            const Nano::Timer &t = m.tm[i];
            int kind; uint16_t top; bool toc;
            m.timer_mode(t, kind, top, toc);
            int cs = m.ram[t.tccrb] & 7;
            std::string pre = cs ? (t.div[cs] ? strf("/%d", t.div[cs]) : std::string("ext")) : std::string("off");
            uint16_t cnt = t.wide ? m.r16(t.tcnt) : m.ram[t.tcnt];
            uint16_t oa = t.wide ? m.r16(t.ocra) : m.ram[t.ocra], ob = t.wide ? m.r16(t.ocrb) : m.ram[t.ocrb];
            L.push_back(std::make_pair(strf("%s %-5s %-5s C%-5u A%-5u B%u", t.name, kn[kind], pre.c_str(), cnt, oa, ob), cs ? 0 : A_DIM));
        }
        uint8_t b = m.ram[UCSR0B];
        L.push_back(std::make_pair(strf("UART %s%s %.0f baud%s", b & 8 ? "TX " : "", b & 0x10 ? "RX " : "", m.baud(), m.ram[UCSR0A] & 2 ? " U2X" : ""), (b & 0x18) ? 0 : A_DIM));
        static const char *refs[4] = {"AREF", "AVCC", "?", "1V1"};
        uint8_t ad = m.ram[ADCSRA];
        L.push_back(std::make_pair(strf("ADC  %s mux %d ref %s%s", ad & 0x80 ? "on " : "off", m.ram[ADMUX] & 0xF, refs[m.ram[ADMUX] >> 6], ad & 0x40 ? " busy" : ""), ad & 0x80 ? 0 : A_DIM));
        uint8_t wd = m.ram[WDTCSR];
        L.push_back(std::make_pair(strf("EEPROM wr %u  WDT %s", m.ee_writes, wd & 0x48 ? strf("%s%s p%d", wd & 8 ? "R" : "", wd & 0x40 ? "I" : "", (wd & 7) | ((wd >> 2) & 8)).c_str() : "off"), 0));
        L.push_back(std::make_pair(strf("SPI %s  TWI %s  SLEEP %s", m.ram[SPCR] & 0x40 ? "on" : "off", m.ram[TWCR] & 4 ? "on" : "off", m.ram[SMCR] & 1 ? "en" : "off"), A_DIM));
        L.push_back(std::make_pair(strf("IRQ  I=%d last %s", (m.ram[SREG_A] >> 7) & 1, m.last_irq ? IRQ_NAMES[m.last_irq] : "-"), 0));
        std::string ic;
        for (int v = 1; v < 26; v++)
            if (m.irq_count[v]) ic += strf("%s:%llu ", IRQ_NAMES[v], (unsigned long long)m.irq_count[v]);
        while (!ic.empty()) {
            L.push_back(std::make_pair(" " + ic.substr(0, iw - 1), A_DIM));
            ic = ic.size() > (size_t)iw - 1 ? ic.substr(iw - 1) : "";
        }
        for (int i = 0; i < (int)L.size() && i < phh - 2; i++) put(py + 1 + i, x + 1, iw, L[i].second, " " + L[i].first);
    }

    // Logic analyzer: all 20 digital pins, D0..D9 on the left half, D10..D13 and A0..A5 on the right,
    // one row each. High is a raised bar, low a baseline, an edge a vertical bar, faster activity a shade.
    // Each label shows the pin, its mode, frequency, duty and average crest (high time); a box on the right
    // details the highlighted pin.
    static const char *short_mode(const std::string &md) { return md == "IN_PU" ? "PU" : md.c_str(); }
    static std::string fmt_hz(double f) {
        if (f <= 0) return "-";
        if (f >= 100000) return strf("%.0fkHz", f / 1000);
        if (f >= 10000) return strf("%.1fkHz", f / 1000);
        if (f >= 1000) return strf("%.2fkHz", f / 1000);
        return f >= 10 ? strf("%.0fHz", f) : strf("%.1fHz", f);
    }
    static std::string fmt_dt(double s) { // compact duration, at most 6 characters
        if (s <= 0) return "-";
        if (s < 1e-3) return strf("%.0fus", s * 1e6);
        if (s < 1e-2) return strf("%.2fms", s * 1e3);
        if (s < 1) return strf("%.0fms", s * 1e3);
        return strf("%.2fs", s);
    }
    bool is_uart_pin(int i) const { return (i == 0 && (m.ram[UCSR0B] & 0x10)) || (i == 1 && (m.ram[UCSR0B] & 0x08)); }

    void draw_pin_detail(int y, int x, int w, int h, int i, uint64_t t0, uint64_t tend) {
        int p = pin_port(i), b = pin_bit(i);
        std::string md = m.pin_mode(i);
        box_at(y, x, h, w, strf("%s  P%c%d  %s", PIN_NAME[i], "BCD"[p], b, md.c_str()));
        int iw = w - 2, row = y + 1, last = y + h - 2;
        auto line = [&](const std::string &k, const std::string &v, int attr) {
            if (row <= last) put(row++, x + 1, iw, attr, strf(" %-7s %s", k.c_str(), v.c_str()));
        };
        Nano::Pulses r = m.pulses(i, t0, tend);
        double win = (tend - t0) / F_CPU;
        line("level", strf("%d  %s", m.level(i), is_uart_pin(i) ? (i ? "UART TX (USART drives it)" : "UART RX")
                                                 : m.ram[A_DDR(p)] & (1 << b) ? "output" : m.ext[i] >= 0 ? "input, driven" : "input"), A_BOLD);
        if (is_uart_pin(i)) {
            int bc = m.uart_bitcyc();
            line("baud", strf("%.0f  8N1%s", m.baud(), m.ram[UCSR0A] & 2 ? " U2X" : ""), 0);
            line("bit", fmt_time(bc / F_CPU) + strf("  byte %s", fmt_time(10 * bc / F_CPU).c_str()), 0);
            line("bytes", strf("TX %llu  RX %llu", (unsigned long long)m.us.txn, (unsigned long long)m.us.rxn), 0);
        } else {
            line("freq", r.freq > 0 ? strf("%.2f Hz  T %s", r.freq, fmt_time(r.period).c_str()) : std::string("- (under 2 periods)"), A_BOLD);
            line("duty", strf("%.1f %%", r.duty * 100), A_BOLD);
            double dev = std::max(r.hi_max - r.hi_avg, r.hi_avg - r.hi_min);
            line("crest", r.highs ? fmt_dt(r.hi_avg) + " avg" + (dev > r.hi_avg * 0.01 ? " \u00B1" + fmt_dt(dev) : std::string(""))
                                  : std::string("-"), A_BOLD);
            line("low", r.lows ? fmt_dt(r.lo_avg) + " avg" : "-", 0);
        }
        line("edges", strf("%d\u2191 %d\u2193  last %s", r.rises, r.falls,
                           r.last_edge ? (fmt_dt((m.cycles - r.last_edge) / F_CPU) + " ago").c_str() : ("none in " + fmt_time(win)).c_str()),
             COLOR_PAIR(C_DIMTXT));
        // the timer behind a PWM pin, and the frequency its settings give
        for (int t = 0; t < 3; t++)
            for (int ch = 0; ch < 2; ch++) {
                const Nano::Timer &tm = m.tm[t];
                if ((ch ? tm.pb : tm.pa) != p || (ch ? tm.bb : tm.ba) != b || !m.oc_connected(tm, ch)) continue;
                int kind; uint16_t top; bool toc;
                m.timer_mode(tm, kind, top, toc);
                static const char *kn[4] = {"NORM", "CTC", "FPWM", "PCPWM"};
                int cs = m.ram[tm.tccrb] & 7, dv = cs ? tm.div[cs] : 0;
                uint16_t ocr = tm.wide ? m.r16(ch ? tm.ocrb : tm.ocra) : m.ram[ch ? tm.ocrb : tm.ocra];
                line("timer", strf("%s OC%c %s /%d", tm.name, ch ? 'B' : 'A', kn[kind], dv), COLOR_PAIR(kp(K_PWM, false)));
                line("ocr", strf("%u of TOP %u", ocr, top), COLOR_PAIR(kp(K_PWM, false)));
                if (dv) {
                    double ef = kind == Nano::K_PC ? F_CPU / (2.0 * dv * top) : kind == Nano::K_CTC ? F_CPU / (2.0 * dv * (top + 1))
                                                                                              : F_CPU / ((double)dv * (top + 1));
                    line("expect", strf("%.2f Hz", ef), COLOR_PAIR(kp(K_PWM, false)));
                }
            }
    }

    void draw_scope(int y, int x, int w, int h) {
        double us = TIMEBASES[tb_i];
        const int lw = 28;
        bool detail = sel < NPINS && w >= 150 && h >= 8;
        const int dw = 34;
        int cw = detail ? w - dw - 1 : w;      // width used by the channels
        int half = (cw - 2) / 2, tw = half - lw - 3;
        if (tw < 8 || h < 4) return;
        uint64_t cpc = (uint64_t)(us * 16); // cycles per column
        uint64_t win = cpc * tw;
        uint64_t tend = m.cycles;
        std::string trig = "off";
        if (trigger && sel < NPINS) {
            int p = sel;
            trig = strf("%s rising", PIN_NAME[p]);
            uint64_t lim = tend > win * 9 / 10 ? tend - win * 9 / 10 : 0;
            uint64_t k = m.log_idx(p, lim), oldest = m.pcount[p] > PLOG ? m.pcount[p] - PLOG : 0;
            while (k > oldest) { k--; if (m.ev_lvl(p, k)) { tend = m.ev_cyc(p, k) + win * 9 / 10; break; } }
        } else if (trigger) trig = "(select a pin)";
        uint64_t t0 = tend > win ? tend - win : 0;
        if (tcur >= tw) tcur = tw - 1;
        int ccol = tcur >= 0 ? tw - 1 - tcur : -1; // cursor column inside each half
        std::string cur = ccol >= 0 ? strf("  cursor %s", fmt_time(-(double)(tw - ccol) * cpc / F_CPU).c_str()) : "";
        box_at(y, x, h, w, strf("LOGIC ANALYZER  %s/col  window %s  trigger %s%s   TAB focus  ↑↓ pin  ←→ cursor  [ ] timebase  t trigger",
                                fmt_time(us * 1e-6).c_str(), fmt_time(win / F_CPU).c_str(), trig.c_str(), cur.c_str()),
               focus == F_SCOPE);
        int rows = std::min(10, h - 3);
        attron(COLOR_PAIR(C_BORDER) | A_DIM);
        mvvline(y + 1, x + 1 + half - 1, ACS_VLINE, rows + 1);
        attroff(COLOR_PAIR(C_BORDER) | A_DIM);
        for (int i = 0; i < NPINS; i++) {
            int row = i % 10;
            if (row >= rows) continue;
            int bx = x + 1 + (i / 10) * half, ry = y + 1 + row;
            std::string md = m.pin_mode(i);
            int k = pin_kind(i);
            std::string lab;
            if (is_uart_pin(i)) lab = strf("%-3s %-3s", PIN_NAME[i], short_mode(md)); // TX/RX: the detail box has the UART figures
            else {
                Nano::Pulses r = m.pulses(i, t0, tend);
                lab = strf("%-3s %-3s %7s %3d%% %6s", PIN_NAME[i], short_mode(md), fmt_hz(r.freq).c_str(),
                           (int)lround(r.duty * 100), r.highs ? fmt_dt(r.hi_avg).c_str() : "-");
            }
            put(ry, bx, lw, i == sel ? (COLOR_PAIR(C_SEL) | A_BOLD) : (COLOR_PAIR(kp(k, false)) | A_BOLD), lab);
            for (int col = 0; col < tw; col++) {
                if (tend < win && (uint64_t)col * cpc < win - tend) continue; // before t=0
                uint64_t a = tend - win + (uint64_t)col * cpc, b2 = a + cpc;
                uint64_t n = m.log_idx(i, b2) - m.log_idx(i, a);
                const char *g;
                int attr = COLOR_PAIR(kp(k, false)) | A_BOLD;
                if (n == 0) {
                    if (m.level_at(i, a)) g = "▀";                   // high
                    else { g = "▁"; attr = COLOR_PAIR(kp(k, false)) | A_DIM; } // low
                } else if (n <= 2) g = "│";                          // edge / short pulse
                else g = "▒";                                        // toggling faster than a column
                if (col == ccol) attr = (attr & ~A_DIM) | A_REVERSE;
                uput(ry, bx + lw + 1 + col, attr, g);
            }
        }
        // time axis under both halves
        int ay = y + 1 + rows;
        if (ay < y + h - 1)
            for (int hlf = 0; hlf < 2; hlf++) {
                int bx = x + 1 + hlf * half;
                put(ay, bx, lw, COLOR_PAIR(C_DIMTXT), trigger ? " t=trigger-10%" : "     Hz   duty  crest  t=now");
                for (int col = 0; col + 8 < tw; col += 10) {
                    double rel = -(double)(tw - col) * cpc / F_CPU;
                    uput(ay, bx + lw + 1 + col, COLOR_PAIR(C_DIMTXT), "╵");
                    put(ay, bx + lw + 2 + col, 8, COLOR_PAIR(C_DIMTXT) | A_DIM, fmt_time(rel));
                }
                if (ccol >= 0) uput(ay, bx + lw + 1 + ccol, COLOR_PAIR(C_WARN) | A_BOLD, "▲");
            }
        if (detail) draw_pin_detail(y + 1, x + w - dw - 1, dw, h - 2, sel, t0, tend);
    }

    void draw() {
        erase();
        int W = COLS, H = LINES;
        if (W < 110 || H < 34) {
            put(0, 0, W, A_BOLD, strf("nanoemu: terminal too small (%dx%d), need at least 110x34", W, H));
            refresh();
            return;
        }
        // top bar
        std::string st = m.paused ? "⏸ PAUSED" : "▶ RUNNING";
        std::string top = strf(" NANOEMU │ %s (%d bytes) │ %s │ speed %s │ %.0f%% realtime ",
                               fw_path.empty() ? "(no firmware)" : fw_path.c_str(), fw_size, st.c_str(),
                               SPEEDS[speed_i] ? strf("x%g", SPEEDS[speed_i]).c_str() : "max", rate / F_CPU * 100);
        attron(COLOR_PAIR(C_TITLE));
        mvhline(0, 0, ' ', W);
        attroff(COLOR_PAIR(C_TITLE));
        uput(0, 0, COLOR_PAIR(C_TITLE) | A_BOLD, top.c_str());
        if (!msg.empty() && now - msg_t < 6) put(0, W - std::min(W / 2, (int)msg.size() + 2), W / 2, COLOR_PAIR(C_TITLE), msg);

        int SH = H >= 45 ? 13 : std::max(5, H - 32); // 10 channel rows + axis + borders
        int LW = 30, RW = 38, MW = W - LW - RW;
        int MH = H - 2 - SH;
        draw_cpu(1, 0, LW, MH);
        draw_right(1, W - RW, RW, MH);
        int bh = std::min(MH - 5, 14);
        draw_board(1, LW, MW, bh);
        int srch = (show_source && dbg.has_lines()) ? std::max(5, (MH - bh) / 2) : 0;
        if (srch) draw_source(1 + bh, LW, MW, srch);
        draw_serial(1 + bh + srch, LW, MW, MH - bh - srch);
        draw_scope(1 + MH, 0, W, SH);

        // key bar
        const char *keys[][2] = {{"SPC", "run/pause"}, {"s", "step"}, {"r", "restart"}, {"l", "reload"}, {"f", "open"},
                                 {"b", "breakpt"}, {"v", "source"}, {"a", "internal"}, {"+-", "speed"}, {"[]", "timebase"}, {"t", "trig"}, {"⏎←→", "drive"},
                                 {"TAB", "focus"}, {"i", "serial"}, {"q", "quit"}};
        int kx = 0;
        move(H - 1, 0);
        clrtoeol();
        for (size_t i = 0; i < sizeof keys / sizeof keys[0]; i++) {
            uput(H - 1, kx, COLOR_PAIR(C_KEY) | A_BOLD, (std::string(" ") + keys[i][0] + " ").c_str());
            kx += (int)strlen(keys[i][0]) + 2 - (strstr(keys[i][0], "⏎") ? 6 : 0);
            put(H - 1, kx, W - kx, 0, std::string(" ") + keys[i][1] + " ");
            kx += strlen(keys[i][1]) + 2;
            if (kx >= W - 4) break;
        }
        if (mode == PROMPT_FILE || mode == PROMPT_BREAK) {
            std::string p = mode == PROMPT_FILE ? "Open firmware (.bin/.hex/.elf): "
                                                : "Toggle breakpoint (function | file:line | line | 0xADDR; empty = clear all): ";
            attron(COLOR_PAIR(C_WARN) | A_BOLD | A_REVERSE);
            mvhline(H - 1, 0, ' ', W);
            mvaddnstr(H - 1, 1, (p + input + "_").c_str(), W - 2);
            attroff(COLOR_PAIR(C_WARN) | A_BOLD | A_REVERSE);
        }
        refresh();
    }

    // -------------------------------------------------------------- input
    void handle_key(int ch) {
        if (mode != NORMAL) {
            if (ch == 27) { mode = NORMAL; input.clear(); return; }
            if (ch == KEY_BACKSPACE || ch == 127 || ch == 8) { if (!input.empty()) input.erase(input.size() - 1); return; }
            if (ch == '\n' || ch == '\r' || ch == KEY_ENTER) {
                if (mode == SERIAL) { send_serial(input + "\n"); input.clear(); return; }
                if (mode == PROMPT_FILE && !input.empty()) load(input);
                if (mode == PROMPT_BREAK) {
                    if (input.empty()) { bp_specs.clear(); apply_breakpoints(); set_msg("all breakpoints cleared"); }
                    else toggle_breakpoint(input);
                }

                mode = NORMAL; input.clear();
                return;
            }
            if (ch >= 32 && ch < 127) input.push_back((char)ch);
            return;
        }
        switch (ch) {
        case 'q': case 'Q': quit = true; break;
        case ' ': m.paused = !m.paused; break;
        case 's': case 'n': {
            double t = mono();
            bool repeat = s_last >= 0 && t - s_last < HOLD_GAP;
            s_last = t;
            if (s_block) break;                                   // stopped at a breakpoint: wait for the release
            if (repeat) { s_hold = true; m.paused = false; break; } // held down: run
            if (!s_hold) { m.paused = true; m.step(); }             // a tap: one instruction
            break;
        }
        case 'v': show_source = !show_source; break;
        case 'a':
            show_internal = !show_internal;
            set_msg(show_internal ? "source: showing all code (Arduino core, libraries, avr-libc)" : "source: user code only");
            break;
        case 'r': case 'R': restart(); break;
        case 'l': case 'L': case KEY_F(5): if (!fw_path.empty()) load(fw_path); else set_msg("no firmware loaded, press f"); break;
        case 'f': mode = PROMPT_FILE; input = fw_path; break;
        case 'b': mode = PROMPT_BREAK; input.clear(); break;
        case '+': case '=': if (speed_i < NSPEEDS - 1) speed_i++; break;
        case '-': case '_': if (speed_i > 0) speed_i--; break;
        case ']': if (tb_i < NTB - 1) tb_i++; break;
        case '[': if (tb_i > 0) tb_i--; break;
        case 't': trigger = !trigger; break;
        case 'c': stext.clear(); break;
        case 'i': mode = SERIAL; input.clear(); break;
        case '\t': focus = focus == F_PINS ? F_SCOPE : F_PINS; break;
        case 27: tcur = -1; break;
        case KEY_UP: case 'k': if (sel > 0) sel--; break;
        case KEY_DOWN: case 'j': if (sel < (focus == F_SCOPE ? NPINS - 1 : 21)) sel++; break;
        case '\n': case '\r': case KEY_ENTER:
            if (sel < NPINS) { m.ext[sel] = m.ext[sel] < 0 ? 1 : m.ext[sel] == 1 ? 0 : -1; m.pins_changed(); }
            break;
        case KEY_LEFT: case KEY_RIGHT: case 'h': case ',': case '.': {
            int dir = (ch == KEY_RIGHT || ch == '.') ? 1 : -1;
            if (focus == F_SCOPE) { // time cursor: left goes back in time
                tcur = tcur < 0 ? 0 : std::max(0, tcur - dir);
                break;
            }
            if (sel >= 14) { int a = sel - 14; m.adcv[a] = (uint16_t)std::max(0, std::min(1023, m.adcv[a] + dir * 32)); }
            else { m.ext[sel] = dir > 0 ? 1 : 0; m.pins_changed(); }
            break;
        }
        default: break;
        }
    }

    // --------------------------------------------------------------- loops
    int run_tui() {
        const char *loc = setlocale(LC_ALL, "");
        if (!loc || !strstr(nl_langinfo(CODESET), "UTF-8")) setlocale(LC_ALL, "en_US.UTF-8"); // box glyphs need UTF-8
        setenv("ESCDELAY", "25", 1);
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        curs_set(0);
        timeout(10);
        if (has_colors()) {
            start_color();
            use_default_colors();
            init_pair(C_BORDER, COLOR_CYAN, -1);
            init_pair(C_TITLE, COLOR_BLACK, COLOR_CYAN);
            init_pair(C_SEL, COLOR_BLACK, COLOR_WHITE);
            init_pair(C_PCB, COLOR_WHITE, COLOR_BLUE);
            init_pair(C_CHIP, COLOR_WHITE, COLOR_BLACK);
            init_pair(C_USB, COLOR_BLACK, COLOR_WHITE);
            init_pair(C_LEDON, COLOR_BLACK, COLOR_YELLOW);
            init_pair(C_LEDOFF, COLOR_BLACK, COLOR_BLUE);
            init_pair(C_KEY, COLOR_BLACK, COLOR_GREEN);
            init_pair(C_WARN, COLOR_YELLOW, -1);
            init_pair(C_DIMTXT, COLOR_WHITE, -1);
            static const short fg[NK] = {COLOR_GREEN, COLOR_RED, COLOR_YELLOW, COLOR_MAGENTA, COLOR_CYAN, COLOR_WHITE, COLOR_RED, COLOR_CYAN};
            for (int k = 0; k < NK; k++) { init_pair(kp(k, false), fg[k], -1); init_pair(kp(k, true), fg[k], COLOR_BLUE); }
        }
        pty_open();
        double last = mono(), lastdraw = 0;
        rate_t = last;
        rate_c = m.cycles;
        while (!quit) {
            now = mono();
            double dt = std::min(now - last, 0.1);
            last = now;
            if (!m.paused) {
                double sp = SPEEDS[speed_i];
                uint64_t target = sp > 0 ? m.cycles + (uint64_t)(dt * F_CPU * sp) : UINT64_MAX;
                double deadline = now + 0.03;
                while (m.cycles < target && !m.paused && mono() < deadline)
                    m.run_until(std::min(target, m.cycles + 50000));
            }
            if (s_hold && m.paused) { s_hold = false; s_block = true; } // a breakpoint stopped the held run
            if ((s_hold || s_block) && mono() - s_last > RELEASE_GAP) { // 's' released
                if (s_hold) m.paused = true;
                s_hold = s_block = false;
            }
            if (!m.event.empty()) { set_msg(m.event); m.event.clear(); }
            drain_tx();
            pty_poll();
            if (now - rate_t >= 0.5) {
                rate = m.cycles >= rate_c ? (m.cycles - rate_c) / (now - rate_t) : 0;
                rate_t = now;
                rate_c = m.cycles;
            }
            if (now - lastdraw >= 1.0 / 30) { draw(); lastdraw = now; }
            int ch = getch();
            while (ch != ERR) {
                handle_key(ch);
                timeout(0);
                ch = getch();
                timeout(10);
            }
        }
        endwin();
        return 0;
    }

    // headless: run N simulated seconds as fast as possible, serial -> stdout
    int run_headless(double secs, const std::string &inject) {
        headless = true;
        if (!inject.empty()) send_serial(inject);
        uint64_t end = (uint64_t)(secs * F_CPU);
        double t0 = mono();
        while (m.cycles < end && !m.paused) { m.run_until(std::min(end, m.cycles + 100000)); now = mono(); drain_tx(); }
        drain_tx();
        print_stats(mono() - t0);
        return 0;
    }

    void print_stats(double el) {
        fprintf(stderr, "\n--- nanoemu: %.3f s simulated in %.3f s (%.1fx realtime), %llu instructions\n",
                m.cycles / F_CPU, el, el > 0 ? m.cycles / F_CPU / el : 0.0, (unsigned long long)m.instrs);
        if (!m.event.empty()) fprintf(stderr, "--- %s\n", m.event.c_str());
        MemInfo mi = mem_info();
        fprintf(stderr, "--- memory: RAM %d B, used %d, free %d, stack %d (peak %d)%s; ROM %d of %d B\n", mi.sram, mi.used,
                mi.avail, mi.stack, mi.peak, mi.symbols ? strf(", data %d, bss %d, heap %d", mi.data, mi.bss, mi.heap).c_str() : "",
                fw_size, FLASH_WORDS * 2 - BOOT_BYTES);
        fprintf(stderr, "--- pins:");
        for (int i = 0; i < NPINS; i++) {
            double f, d;
            m.measure(i, m.cycles > 1600000 ? m.cycles - 1600000 : 0, m.cycles, f, d);
            fprintf(stderr, " %s=%d%s", PIN_NAME[i], m.level(i), f > 0 ? strf("(%.0fHz,%d%%)", f, (int)lround(d * 100)).c_str() : "");
        }
        fprintf(stderr, "\n");
    }

    // serial only, no TUI: real-time run, UART TX -> stdout, stdin -> UART RX, until Ctrl-C
    // (or SECS simulated seconds, or a breakpoint). speed 0 = as fast as possible.
    static volatile sig_atomic_t stop_req;
    static void on_sigint(int) { stop_req = 1; }
    int run_serial(double secs, double speed, const std::string &inject) {
        headless = true;
        stop_req = 0;
        signal(SIGINT, on_sigint);
        signal(SIGTERM, on_sigint);
        if (!inject.empty()) send_serial(inject);
        uint64_t end = secs > 0 ? (uint64_t)(secs * F_CPU) : UINT64_MAX;
        bool in_open = true;
        double t0 = mono(), last = t0;
        while (!stop_req && m.cycles < end && !m.paused) {
            now = mono();
            double dt = std::min(now - last, 0.1);
            last = now;
            uint64_t target = speed > 0 ? m.cycles + (uint64_t)(dt * F_CPU * speed) : m.cycles + 400000;
            m.run_until(std::min(end, target));
            drain_tx();
            if (in_open) { // stdin -> RX
                pollfd pfd = {0, POLLIN, 0};
                if (poll(&pfd, 1, 0) > 0) {
                    char b[256];
                    ssize_t n = read(0, b, sizeof b);
                    if (n > 0) send_serial(std::string(b, n));
                    else in_open = false; // EOF: keep running, stop reading
                }
            }
            if (speed > 0) usleep(2000);
        }
        drain_tx();
        print_stats(mono() - t0);
        return 0;
    }
};
volatile sig_atomic_t App::stop_req = 0;

static void usage(const char *p) {
    fprintf(stderr,
            "usage: %s [options] firmware.{bin,hex,elf}\n"
            "  -c, --serial run without the TUI, in real time: serial TX to stdout, stdin to serial RX;\n"
            "               stops on Ctrl-C, a breakpoint, or after -H SECONDS\n"
            "  --speed X    with --serial: speed factor (1 = real time, 0 = as fast as possible)\n"
            "  -H SECONDS   headless: run SECONDS of simulated time as fast as possible, serial to stdout\n"
            "  -i TEXT      (headless/serial) bytes to send to the serial RX line at start\n"
            "  -b WHERE     breakpoint: function | file:line | 0xADDR (repeatable)\n"
            "  -S DIR       extra directory to search for source files (repeatable)\n"
            "  -p           start paused\n"
            "  --ram BYTES  test with more SRAM than the 328P's 2K (e.g. 4096, 8K; multiple of 256, max 65280)\n"
            "  --flash SIZE test with more flash than 32K: 64K or 128K\n",
            p);
}

// size in bytes: 4096, 8K, 64k
static long parse_size(const char *t) {
    char *e = 0;
    long v = strtol(t, &e, 10);
    if (e && (*e == 'k' || *e == 'K')) v *= 1024;
    return v;
}

int main(int argc, char **argv) {
    double hsecs = -1;
    std::string inject, file;
    std::vector<std::string> bps, dirs;
    bool start_paused = false, serial_only = false;
    double speed = 1;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "-H" && i + 1 < argc) hsecs = atof(argv[++i]);
        else if (a == "-i" && i + 1 < argc) inject = argv[++i];
        else if (a == "-b" && i + 1 < argc) bps.push_back(argv[++i]);
        else if (a == "-S" && i + 1 < argc) dirs.push_back(argv[++i]);
        else if (a == "-p") start_paused = true;
        else if (a == "-c" || a == "--serial") serial_only = true;
        else if (a == "--speed" && i + 1 < argc) speed = atof(argv[++i]);
        else if (a == "--ram" && i + 1 < argc) {
            long v = parse_size(argv[++i]);
            if (v < 2048 || v > 0xFF00 || v % 256) { fprintf(stderr, "--ram: 2048 to 65280 bytes, a multiple of 256 (e.g. 4096, 8K)\n"); return 1; }
            RAM_END = 0x100 + (int)v;
        } else if (a == "--flash" && i + 1 < argc) {
            long v = parse_size(argv[++i]);
            if (v != 32768 && v != 65536 && v != 131072) { fprintf(stderr, "--flash: 32K, 64K or 128K\n"); return 1; }
            FLASH_WORDS = (int)(v / 2);
            PC_MASK = FLASH_WORDS - 1;
        }
        else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else if (a[0] == '-') { usage(argv[0]); return 1; }
        else file = a;
    }
    static App *papp = new App; // built after the sizes are known; large (pin logs), so on the heap
    App &app = *papp;
    app.bp_specs = bps;
    app.src_dirs = dirs;
    if (!file.empty() && !app.load(file)) {
        fprintf(stderr, "%s\n", app.msg.c_str());
        return 1;
    }
    if (serial_only) {
        if (file.empty()) { usage(argv[0]); return 1; }
        return app.run_serial(hsecs, speed, inject);
    }
    if (hsecs >= 0) {
        if (file.empty()) { usage(argv[0]); return 1; }
        return app.run_headless(hsecs, inject);
    }
    app.m.paused = start_paused || file.empty();
    return app.run_tui();
}
