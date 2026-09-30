// Minimal Arduino API mock so wheelodom.ino compiles and runs on the host.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
using std::max;
using std::min;

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define RISING 3
#define CHANGE 1

extern unsigned long g_now;   // fake millis()
extern int g_pins[64];        // last value written per pin
extern int g_mode[64];        // last pinMode per pin
extern std::string g_log;     // everything Serial.println'd

inline unsigned long millis() { return g_now; }
extern unsigned long g_now_us;   // sub-ms part of fake micros()
inline unsigned long micros() { return g_now * 1000UL + g_now_us; }
inline void delay(unsigned long ms) { g_now += ms; }
inline void delayMicroseconds(unsigned long) {}
inline void digitalWrite(int p, int v) { g_pins[p] = v; }
inline int digitalRead(int p) { return g_pins[p]; }
inline void analogWrite(int p, int v) { g_pins[p] = v; }
inline void pinMode(int p, int m) { g_mode[p] = m; }
inline void noInterrupts() {}
inline void interrupts() {}
inline int digitalPinToInterrupt(int p) { return p; }
inline void attachInterrupt(int, void (*)(), int) {}
template <class T> T constrain(T x, T a, T b) { return x < a ? a : (x > b ? b : x); }

struct String {
  std::string s;
  String(const char* c = "") : s(c) {}
  String(std::string x) : s(x) {}
  int indexOf(const char* c) const {
    auto i = s.find(c);
    return i == std::string::npos ? -1 : (int)i;
  }
  String substring(int a, int b = -1) const {
    return b < 0 ? String(s.substr(a)) : String(s.substr(a, b - a));
  }
  float toFloat() const { return atof(s.c_str()); }
  unsigned length() const { return s.size(); }
  char operator[](unsigned i) const { return i < s.size() ? s[i] : 0; }
  String& operator+=(char c) { s += c; return *this; }
  bool operator==(const char* c) const { return s == c; }
};

struct SerialT {
  void begin(int) {}
  int available() { return 0; }
  int read() { return -1; }
  template <class T> void print(T) {}
  template <class T> void print(T, int) {}
  void println(const char* c) { g_log += c; g_log += "\n"; }
  template <class T> void println(T) {}
  template <class T> void println(T, int) {}
};
extern SerialT Serial;
