/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "util.h"
#include <math.h>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <stdarg.h>
#if defined RAI_Linux || defined RAI_Cygwin || defined RAI_Darwin
#  include <limits.h>
#  include <sys/time.h>
#  include <sys/times.h>
#  include <sys/resource.h>
#  include <sys/inotify.h>
#  include <sys/stat.h>
#  include <poll.h>
#  include <execinfo.h>
#if defined RAI_X11
#  include <X11/Xlib.h>
#  include <X11/Xutil.h>
#endif
#endif
#ifdef __CYGWIN__
#include "cygwin_compat.h"
#endif
#if defined RAI_MSVC
#  include <time.h>
#  include <sys/timeb.h>
#  include <windows.h>
#  undef min
#  undef max
#  define RAI_TIMEB
#  ifdef RAI_QT
#    undef  NOUNICODE
#    define NOUNICODE
#  endif
#  pragma warning(disable: 4305 4244 4250 4355 4786 4996)
#endif
#if defined RAI_MinGW
#  include <unistd.h>
#  include <sys/time.h>
#  include <sys/timeb.h>
#  define RAI_TIMEB
#endif

#ifdef RAI_QT
#  undef scroll
#  undef Unsorted
#  include <Qt/qmetatype.h>
#  include <Qt/qdatastream.h>
#  include <Qt/qapplication.h>
#  include <QThread>
#endif

#ifndef RAI_ConfigFileName
#  define RAI_ConfigFileName "rai.cfg"
#  define RAI_LogFileName "rai.log"
#endif

#include <errno.h>
#ifndef RAI_MSVC
#  include <unistd.h>
#  include <sys/syscall.h>
#endif

//===========================================================================
//
// Bag container
//

const char *rai::String::readSkipSymbols = " \t";
const char *rai::String::readStopSymbols = "\n\r";
int   rai::String::readEatStopSymbol     = 1;
rai::String rai::errString;
Mutex coutMutex;
rai::LogObject _log("global", 2, 3);

//===========================================================================
//
// utilities in rai namespace
//

namespace rai {
int argc;
char** argv;
bool IOraw=false;
bool noLog=true;
uint lineCount=1;
int verboseLevel=-1;
int interactivity=-1;

double startTime;
double timerStartTime=0.;
double timerPauseTime=-1.;
bool timerUseRealTime=false;

#ifdef RAI_QT
QApplication *myApp=NULL;
#endif

/// running a system command and checking return value
void system(const char *cmd) {
  cout <<"SYSTEM CMD: " <<cmd <<endl;
  int r = ::system(cmd);
  rai::wait(.1);
  if(r) HALT("system return error " <<r);
}

/// open an output-file with name '\c name'
void open(std::ofstream& fs, const char *name, const char *errmsg) {
  fs.clear();
  fs.open(name);
  LOG(3) <<"opening output file '" <<name <<"'" <<std::endl;
  if(!fs.good()) RAI_MSG("could not open file '" <<name <<"' for output" <<errmsg);
}

/// open an input-file with name '\c name'
void open(std::ifstream& fs, const char *name, const char *errmsg) {
  fs.clear();
  fs.open(name);
  LOG(3) <<"opening input file '" <<name <<"'" <<std::endl;
  if(!fs.good()) HALT("could not open file '" <<name <<"' for input" <<errmsg);
}

/// returns true if the (0-terminated) string s contains c
bool contains(const char *s, char c) {
  if(!s) return false;
  for(uint i=0; s[i]; i++) if(s[i]==c) return true;
  return false;
}

/// skips the chars (typically white characters) when parsing from the istream, returns first non-skipped char
char skip(std::istream& is, const char *skipSymbols, const char *stopSymbols, bool skipCommentLines) {
  char c;
  for(;;) {
    c=is.get();
    if(skipCommentLines && c=='#') { skipRestOfLine(is); continue; }
    if(skipSymbols && !contains(skipSymbols, c)) break;
    if(stopSymbols && contains(stopSymbols, c)) break;
    if(c=='\n') lineCount++;
  }
  is.putback(c);
  return c;
}

/// skips a newline character (same as skip(is, "\n");)
void skipRestOfLine(std::istream& is) {
  char c;
  do { c=is.get(); } while(c!='\n' && is.good());
  if(c=='\n') is.putback(c);
}

/// skips the next character
void skipOne(std::istream& is) {
  is.get();
}

/// tell you about the next char (after skip()) but puts it back in the stream
char getNextChar(std::istream& is, const char *skipSymbols, bool skipCommentLines) {
  char c;
  skip(is, skipSymbols, NULL, skipCommentLines);
  is.get(c);
  if(!is.good()) return 0;
  return c;
}

/// tell you about the next char (after skip()) but puts it back in the stream
char peerNextChar(std::istream& is, const char *skipSymbols, bool skipCommentLines) {
  char c=getNextChar(is, skipSymbols, skipCommentLines);
  if(!is.good()) return 0;
  is.putback(c);
  return c;
}

/// skip throught a stream until the tag is found (which is eaten)
bool skipUntil(std::istream& is, const char *tag) {
  unsigned n=strlen(tag);
  char *buf=new char [n+1];
  memset(buf, 0, n+1);
  while(is.good()) {
    memmove(buf, buf+1, n);
    buf[n-1]=is.get();
    if(buf[n-1]=='\n') lineCount++;
    buf[n]=0;
    if(!strcmp(tag, buf)) { delete[] buf; return true; }
  };
  delete[] buf;
  return false;
}

/// a global operator to scan (parse) strings from a stream
bool parse(std::istream& is, const char *str, bool silent) {
  if(!is.good()) { if(!silent) RAI_MSG("bad stream tag when scanning for '" <<str <<"'"); return false; }  //is.clear(); }
  uint i, n=strlen(str);
  char buf[n+1]; buf[n]=0;
  rai::skip(is, " \n\r\t");
  is.read(buf, n);
  if(!is.good() || strcmp(str, buf)) {
    for(i=n; i--;) is.putback(buf[i]);
    is.setstate(std::ios::failbit);
    if(!silent)  RAI_MSG("(LINE=" <<rai::lineCount <<") parsing of constant string '" <<str
                           <<"' failed! (read instead: '" <<buf <<"')");
    return false;
  }
  return true;
}

/// returns the i-th of str
byte bit(byte *str, uint i) { return (str[i>>3] >>(7-(i&7))) & 1; }
//byte bit(byte b, uint i){ return (b >>(7-(i&7))) & 1; }
//void set(byte *state, uint i){ state[i>>3] |= 1 <<(7-(i&7)); }
//void del(byte *state, uint i){ state[i>>3] &= (byte)~(1 <<(7-(i&7))); }
//void flip(byte *str, uint i){ str[i>>3] ^= 1 <<(7-(i&7)); }

/// flips the i-th bit of b
void flip(byte& b, uint i) { b ^= 1 <<(7-(i&7)); }

/// filps the i-th bit of b
void flip(int& b, uint i) { b ^= 1 <<(7-(i&7)); }

double MIN(double a, double b) { return a<b?a:b; }
double MAX(double a, double b) { return a>b?a:b; }
uint MAX(uint a, uint b) { return a>b?a:b; }
int MAX(int a, int b) { return a>b?a:b; }

double indicate(bool expr) { if(expr) return 1.; return 0.; }

/** @brief the distance between x and y w.r.t.\ a circular topology
    (e.g. modMetric(1, 8, 10)=3) */
double modMetric(double x, double y, double mod) {
  double d=fabs(x-y);
  d=fmod(d, mod);
  if(d>mod/2.) d=mod-d;
  return d;
}

/// the sign (+/-1) of x (+1 for zero)
double sign(double x) { if(x<0.) return -1.; return 1.; }

/// the sign (+/-1) of x (0 for zero)
double sign0(double x) { if(x<0.) return -1.; if(!x) return 0.; return 1.; }

/// returns 0 for x<0, 1 for x>1, x for 0<x<1
double linsig(double x) { if(x<0.) return 0.; if(x>1.) return 1.; return x; }

/// x ends up in the interval [a, b]
//void clip(double& x, double a, double b) { if(x<a) x=a; if(x>b) x=b; }

/// the angle of the vector (x, y) in [-pi, pi]
double phi(double x, double y) {
  if(x==0. || ::fabs(x)<1e-10) { if(y>0.) return RAI_PI/2.; else return -RAI_PI/2.; }
  double p=::atan(y/x);
  if(x<0.) { if(y<0.) p-=RAI_PI; else p+=RAI_PI; }
  if(p>RAI_PI)  p-=2.*RAI_PI;
  if(p<-RAI_PI) p+=2.*RAI_PI;
  return p;
}

/// the change of angle of the vector (x, y) when displaced by (dx, dy)
double dphi(double x, double y, double dx, double dy) {
  //return (dy*x - dx*y)/sqrt(x*x+y*y);
  if(x==0. || ::fabs(x)<1e-10) { if(y>0.) return -dx/y; else return dx/y; }
  double f=y/x;
  return 1./(1.+f*f)*(dy/x - f/x*dx);
}

/** @brief save division, checks for division by zero; force=true will return
  zero if y=0 */
double DIV(double x, double y, bool force) {
  if(x==0.) return 0.;
  if(force) { if(y==0.) return 0.; } else CHECK(y!=0, "Division by Zero!");
  return x/y;
}

double sigmoid11(double x) {
  return x/(1.+::fabs(x));
}

double sigmoid(double x) {
  return 1./(1.+exp(-x));
}

double dsigmoid(double x) {
  double y=sigmoid(x);
  return y*(1.-y);
}

#define AXETS 1280
#define AXETR 10.
/// approximate exp (sets up a static value table)
double approxExp(double x) {
  static bool initialized=false;
  static double ExpTable [AXETS]; //table ranges from x=-10 to x=10
  int i;
  if(!initialized) {
    for(i=0; i<AXETS; i++) ExpTable[i]=::exp(AXETR*(2*i-AXETS)/AXETS);
    initialized=true;
  }
  x*=.5*AXETS/AXETR;
  i=(int)x;
  x-=(double)i; //x = residual
  i+=AXETS/2; //zero offset
  if(i>=AXETS-1) return ExpTable[AXETS-1];
  if(i<=0) return 0.; //ExpTable[0];
  return (1.-x)*ExpTable[i] + x*ExpTable[i+1];
}

/// ordinary Log, but cutting off for small values
double Log(double x) {
  if(x<.001) x=.001;
  return ::log(x);
}

/// integer log2
uint Log2(uint n) {
  uint l=0;
  n=n>>1;
  while(n) { l++; n=n>>1; }
  return l;
}

/// square of a double
double sqr(double x) { return x*x; }

double sinc(double x) {
  if(fabs(x)<1e-10) return 1.-.167*x*x;
  return ::sin(x)/x;
}

double cosc(double x) {
  if(fabs(x)<1e-10) return 1.-.167*x*x;
  return ::cos(x)/x;
}

#define EXP ::exp //rai::approxExp

double NNsdv(const double& a, const double& b, double sdv) {
  double d=(a-b)/sdv;
  double norm = 1./(::sqrt(RAI_2PI)*sdv);
  return norm*EXP(-.5*d*d);
}

double NNsdv(double x, double sdv) {
  x/=sdv;
  double norm = 1./(::sqrt(RAI_2PI)*sdv);
  return norm*EXP(-.5*x*x);
}

/* gnuplot:
heavy(x) = (1+sgn(x))/2
eps = 0.1
g(x) = heavy(x-eps)*(x-eps/2) + (1-heavy(x-eps))*x**2/(2*eps)
plot [-.5:.5] g(abs(x))
*/
double POW(double x, double power) { if(power==1.) return x; if(power==2.) return x*x; return pow(x,power); }
double smoothRamp(double x, double eps, double power) {
  if(x<0.) return 0.;
  if(power!=1.) return pow(smoothRamp(x,eps,1.),power);
  if(!eps) return x;
  if(x>eps) return x - .5*eps;
  return x*x/(2*eps);
}

double d_smoothRamp(double x, double eps, double power) {
  if(x<0.) return 0.;
  if(power!=1.) return power*pow(smoothRamp(x,eps,1.),power-1.)*d_smoothRamp(x,eps,1.);
  if(!eps || x>eps) return 1.;
  return x/eps;
}

/*
heavy(x) = (1+sgn(x))/2
power = 1.5
margin = 1.5
f(x) = heavy(x)*x**power
plot f(x/margin+1), 1
*/
double ineqConstraintCost(double g, double margin, double power) {
  double y=g+margin;
  if(y<0.) return 0.;
  if(power==1.) return y;
  if(power==2.) return y*y;
  return pow(y,power);
}

double d_ineqConstraintCost(double g, double margin, double power) {
  double y=g+margin;
  if(y<0.) return 0.;
  if(power==1.) return 1.;
  if(power==2.) return 2.*y;
  return power*pow(y,power-1.);
}

double eqConstraintCost(double h, double margin, double power) {
  double y=h/margin;
  if(power==1.) return fabs(y);
  if(power==2.) return y*y;
  return pow(fabs(y),power);
}

double d_eqConstraintCost(double h, double margin, double power) {
  double y=h/margin;
  if(power==1.) return rai::sign(y)/margin;
  if(power==2.) return 2.*y/margin;
  return power*pow(y,power-1.)*rai::sign(y)/margin;
}

/** @brief double time on the clock
  (probably in micro second resolution) -- Windows checked! */
double clockTime(bool today) {
#ifndef RAI_TIMEB
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  if(today) ts.tv_sec = ts.tv_sec%86400; //modulo TODAY
  return ((double)(ts.tv_sec) + 1e-9d*(double)(ts.tv_nsec));
//  static timeval t; gettimeofday(&t, 0);
//  return ((double)(t.tv_sec%86400) + //modulo TODAY
//          (double)(t.tv_usec)/1000000.);
#else
  static _timeb t; _ftime(&t);
  return ((double)(t.time%86400) + //modulo TODAY
          (double)(t.millitm-startTime.millitm)/1000.);
#endif
}

timespec clockTime2() {
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return ts;
}

double toTime(const tm& t) {
  return (double)(mktime(const_cast<tm*>(&t)) % 86400);
}

/** @brief double time since start of the process in floating-point seconds
  (probably in micro second resolution) -- Windows checked! */
double realTime() {
  return clockTime(false)-startTime;
}

/** @brief user CPU time of this process in floating-point seconds (pure
  processor time) -- Windows checked! */
double cpuTime() {
#ifndef RAI_TIMEB
  tms t; times(&t);
  return ((double)t.tms_utime)/sysconf(_SC_CLK_TCK);
#else
  return ((double)clock())/CLOCKS_PER_SEC; //MSVC: CLOCKS_PER_SEC=1000
#endif
}

/** @brief system CPU time of this process in floating-point seconds (the
  time spend for file input/output, x-server stuff, etc.)
  -- not implemented for Windows! */
double sysTime() {
#ifndef RAI_TIMEB
  tms t; times(&t);
  return ((double)(t.tms_stime))/sysconf(_SC_CLK_TCK);
#else
  HALT("sysTime() is not implemented for Windows!");
  return 0.;
#endif
}

/** @brief total CPU time of this process in floating-point seconds (same
  as cpuTime + sysTime) -- not implemented for Windows! */
double totalTime() {
#ifndef RAI_TIMEB
  tms t; times(&t);
  return ((double)(t.tms_utime+t.tms_stime))/sysconf(_SC_CLK_TCK);
#else
  HALT("totalTime() is not implemented for Windows!");
  return 0.;
#endif
}

/// the absolute double time and date as string
char *date() {
  return date(clockTime(false));
}

char *date(double sec) {
  time_t nowtime;
  struct tm *nowtm;
  static char tmbuf[64], buf[64];
  
  nowtime = (long)(floor(sec));
  sec -= (double)nowtime;
  nowtm = localtime(&nowtime);
  strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d_%H:%M:%S", nowtm);
  snprintf(buf, sizeof buf, "%s.%06ld", tmbuf, (long)(floor(1e6*sec)));
  return buf;
}

char *date2(bool subsec) {
  return date2(clockTime(false), subsec);
}

char *date2(double sec, bool subsec) {
  time_t nowtime;
  struct tm *nowtm;
  static char tmbuf[64], buf[64];
  
  nowtime = (long)(floor(sec));
  sec -= (double)nowtime;
  nowtm = localtime(&nowtime);
  strftime(tmbuf, sizeof tmbuf, "%y%m%d-%H%M%S", nowtm);
  if(!subsec) return tmbuf;
  snprintf(buf, sizeof buf, "%s.%06ld", tmbuf, (long)(floor(1e6*sec)));
  return buf;
}

/// wait double time
void wait(double sec, bool msg_on_fail) {
#if defined(RAI_Darwin)
  sleep((int)sec);
#elif !defined(RAI_MSVC)
  timespec ts;
  ts.tv_sec = (long)(floor(sec));
  sec -= (double)ts.tv_sec;
  ts.tv_nsec = long(floor(1e9d*sec));
  int rc = clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
  if(rc && msg_on_fail) {
    RAI_MSG("clock_nanosleep() failed " <<rc <<" '" <<strerror(rc) <<"' trying select instead");
    timeval tv;
    tv.tv_sec = ts.tv_sec;
    tv.tv_usec = ts.tv_nsec/1000l;
    rc = select(1, NULL, NULL, NULL, &tv);
    if(rc==-1) RAI_MSG("select() failed " <<rc <<" '" <<strerror(errno) <<"'");
  }
#else
  Sleep((int)(1000.*sec));
  //MsgWaitForMultipleObjects( 0, NULL, FALSE, (int)(1000.*sec), QS_ALLEVENTS);
#endif
  
#if 0
#ifndef RAI_TIMEB
  /* r=0 time is up
     r!=0 data in NULL stream available (nonsense)
     */
#else
  double t=realTime(); while(realTime()-t<sec);
#  endif
#endif
}

/// wait for an ENTER at the console
bool wait(bool useX11) {
  if(!rai::getInteractivity()) {
    rai::wait(.1);
    return true;
  }
  if(!useX11) {
    char c[10];
    std::cout <<" -- hit a key to continue..." <<std::flush;
    //cbreak(); getch();
    std::cin.getline(c, 10);
    std::cout <<"\r" <<std::flush;
    if(c[0]==' ') return true;
    else return false;
    return true;
  } else {
    char c = x11_getKey();
    if(c==' ') return true;
    return false;
  }
}

#ifdef RAI_X11
int x11_getKey() {
  rai::String txt="PRESS KEY";
  int key=0;

  Display *disp = XOpenDisplay(NULL);
  CHECK(disp, "Cannot open display");
  
  Window win = XCreateSimpleWindow(disp, DefaultRootWindow(disp),
                                   10, 10, 80, 50, //24
                                   2, 0x000000, 0x20a0f0);
  XSelectInput(disp, win, KeyPressMask | ExposureMask | ButtonPressMask);
  XMapWindow(disp, win);
  
  GC gc = XCreateGC(disp, win, 0, NULL);
  XSetFont(disp, gc,  XLoadFont(disp,"fixed")); //-adobe-courier-bold-r-*-*-*-220-*-*-*-*-*-*"));
  XSetForeground(disp, gc, 0x000000);
  
  bool quit=false;
  for(; !quit;) {
    XEvent ev;
    XNextEvent(disp, &ev);
    switch(ev.type) {
      case Expose:
        if(ev.xexpose.count == 0) {
          XDrawString(disp, win, gc, 12, 30, txt.p, txt.N);
          XFlush(disp);
        }
        break;
      case KeyPress:
        char string[4];
        XLookupString(&ev.xkey, string, 4, NULL, NULL);
        key = string[0];
        quit=true;
        break;
      case ButtonPress:
        quit=true;
        break;
    }
  }
  
  XCloseDisplay(disp);
  return key;
}
#else
int x11_getKey() {
  LOG(-1) <<"fake implementation (no X11)";
  return 13;
}
#endif

/// the integral shared memory size -- not implemented for Windows!
long mem() {
#ifndef RAI_TIMEB
  static rusage r; getrusage(RUSAGE_SELF, &r);
  return r.ru_idrss;
#else
  HALT("rai::mem() is not implemented for Windows!");
  return 0;
#endif
}

/// start and reset the timer (user CPU time)
double timerStart(bool useRealTime) {
  if(useRealTime) timerUseRealTime=true; else timerUseRealTime=false;
  timerPauseTime=-1.;
  timerStartTime=(timerUseRealTime?realTime():cpuTime());
  return timerStartTime;
}

/// read the timer and optionally also reset it (user CPU time)
double timerRead(bool reset) {
  double c;
  if(timerPauseTime!=-1.) c=timerPauseTime; else c=(timerUseRealTime?realTime():cpuTime())-timerStartTime;
  if(reset) timerStart(timerUseRealTime);
  return c;
}

/// read the timer relative to a given start time (user CPU time)
double timerRead(bool reset, double startTime) {
  double c=(timerUseRealTime?realTime():cpuTime())-startTime;
  if(reset) timerStart(timerUseRealTime);
  return c;
}

/// read and pause the timer
double timerPause() {
  timerPauseTime=(timerUseRealTime?realTime():cpuTime())-timerStartTime;
  return timerPauseTime;
}

/// resume the timer after a pause, neglecting the time inbetween
void timerResume() {
  timerStartTime=(timerUseRealTime?realTime():cpuTime())-timerPauseTime;
  timerPauseTime=-1.;
}

/// memorize the command line arguments and open a log file
void initCmdLine(int _argc, char *_argv[]) {
  argc=_argc; argv=_argv;
  rai::String msg;
  msg <<"** cmd line arguments: '"; for(int i=0; i<argc; i++) msg <<argv[i] <<' ';
  msg <<"\b'";
  LOG(1) <<msg;
}

/// returns true if the tag was found on command line
bool checkCmdLineTag(const char *tag) {
  for(int n=1; n<argc; n++) if(argv[n][0]=='-' && !strcmp(tag, argv[n]+1)) {
      return true;
    }
  return false;
}

/// returns the argument after the cmd-line tag; NULL if the tag is not found
char *getCmdLineArgument(const char *tag) {
  int n;
  for(n=1; n<argc; n++) if(argv[n][0]=='-' && !strcmp(tag, argv[n]+1)) {
      if(n+1==argc) return (char*)"1";
      return argv[n+1];
    }
  return NULL;
}

//String raiPath(const char* rel) {
//  String path(RAI_ROOT_PATH);
//  path <<"/" <<rel;
//  return path;
//}

uint getVerboseLevel() {
  if(verboseLevel==-1) verboseLevel=getParameter<int>("verbose", 0);
  return verboseLevel;
}

bool getInteractivity() {
  if(interactivity==-1) interactivity=(checkParameter<bool>("noInteractivity")?0:1);
  return interactivity==1;
}

}//namespace rai

//===========================================================================
//
// logging

namespace rai {
void handleSIGUSR2(int) {
  int i=5;
  i*=i;    //set a break point here, if you want to catch errors directly
}

struct LogServer {
  LogServer() {
    signal(SIGUSR2, rai::handleSIGUSR2);
    timerStartTime=rai::cpuTime();
    startTime = clockTime(false);
  }
  
  ~LogServer() {
  }
};

Singleton<rai::LogServer> logServer;
}

rai::LogObject::LogObject(const char* key, int defaultLogCoutLevel, int defaultLogFileLevel)
  : key(key), logCoutLevel(defaultLogCoutLevel), logFileLevel(defaultLogFileLevel) {
  if(!strcmp(key,"global")) {
    fil.open("z.log.global");
    fil <<"** compiled at:     " <<__DATE__ <<" " <<__TIME__ <<'\n';
    fil <<"** execution start: " <<rai::date(rai::startTime) <<std::endl;
  } else {
    logCoutLevel = rai::getParameter<int>(STRING("logCoutLevel_"<<key), logCoutLevel);
    logFileLevel = rai::getParameter<int>(STRING("logFileLevel_"<<key), logFileLevel);
  }
}

rai::LogObject::~LogObject() {
  if(!strcmp(key,"global")) {
    fil <<"** execution stop: " <<rai::date()
        <<"\n** real time: " <<rai::realTime()
        <<"sec\n** CPU time: " <<rai::cpuTime()
        <<"sec\n** system (includes I/O) time: " <<rai::sysTime() <<"sec" <<std::endl;
  }
  fil.close();
}

rai::LogToken rai::LogObject::getToken(int log_level, const char* code_file, const char* code_func, uint code_line) {
  return rai::LogToken(*this, log_level, code_file, code_func, code_line);
}

rai::LogToken::~LogToken() {
  auto mut = rai::logServer(); //keep the mutex
  if(log.logFileLevel>=log_level) {
    if(!log.fil.is_open()) rai::open(log.fil, STRING("z.log."<<log.key));
    log.fil <<code_file <<':' <<code_func <<':' <<code_line <<'(' <<log_level <<") " <<msg <<endl;
  }
  if(log.logCoutLevel>=log_level) {
    if(log_level>=0) std::cout <<code_file <<':' <<code_func <<':' <<code_line <<'(' <<log_level <<") " <<msg <<endl;
    if(log_level<0) {

      if(log_level<=-2){
        void *trace_elems[10];
        int trace_elem_count = backtrace( trace_elems, 10 );
        char **stack_syms = backtrace_symbols( trace_elems, trace_elem_count );
        for(int i=trace_elem_count; i--;) std::cout <<"STACK" <<i <<' ' <<stack_syms[i] <<'\n';
        free( stack_syms );
      }

      rai::errString.clear() <<code_file <<':' <<code_func <<':' <<code_line <<'(' <<log_level <<") " <<msg;
// #ifdef RAI_ROS
//       ROS_INFO("RAI-MSG: %s",rai::errString.p);
// #endif
      if(log_level==-1) { cout <<"** WARNING:" <<rai::errString <<endl; }
      if(log_level==-2) { cerr <<"** ERROR:" <<rai::errString <<endl; /*throw does not WORK!!! Because this is a destructor. The THROW macro does it inline*/ }
      if(log_level==-3) { cerr <<"** HARD EXIT! " <<rai::errString <<endl; /*rai::logServer().mutex.unlock();*/ exit(1); }
      if(log_level<=-3) raise(SIGUSR2);
    }
  }
//  rai::logServer().mutex.unlock();
}

void setLogLevels(int fileLogLevel, int consoleLogLevel) {
  _log.logCoutLevel=consoleLogLevel;
  _log.logFileLevel=fileLogLevel;
}

//===========================================================================
//
// parameters

namespace rai {

}

/// a global operator to scan (parse) strings from a stream
std::istream& operator>>(std::istream& is, const PARSE& x) {
  rai::parse(is, x.str); return is;
}

/// the same global operator for non-const string
std::istream& operator>>(std::istream& is, char *str) {
  rai::parse(is, (const char*)str); return is;
}

//===========================================================================
//
// String class
//

int rai::String::StringBuf::overflow(int C) {
  string->append(C);
  return C;
}

int rai::String::StringBuf::sync() {
  if(string->flushCallback) string->flushCallback(*string);
  return 0;
}

void rai::String::StringBuf::setIpos(char *p) { setg(string->p, p, string->p+string->N); }

char *rai::String::StringBuf::getIpos() { return gptr(); }

//-- direct memory operations
void rai::String::append(char x) { resize(N+1, true); operator()(N-1)=x; }

void rai::String::prepend(const rai::String& s){
  uint n=N;
  resize(n+s.N, true);
  memmove(p+s.N, p, n);
  memmove(p, s, s.N);
}

rai::String& rai::String::setRandom() {
  resize(rnd(2,6), false);
  for(uint i=0; i<N; i++) operator()(i)=rnd('a','z');
  return *this;
}

void rai::String::resize(uint n, bool copy) {
  if(N==n && M>N) return;
  char *pold=p;
  uint Mold=M;
  //flexible allocation (more than needed in case of multiple resizes)
  if(M==0) {  //first time
    M=n+1;
  } else if(n+1>M || 10+2*n<M/2) {
    M=11+2*n;
  }
  if(M!=Mold) { //do we actually have to allocate?
    p=new char [M];
    if(!p) HALT("rai::Mem failed memory allocation of " <<M <<"bytes");
    if(copy) memmove(p, pold, N<n?N:n);
    if(Mold) delete[] pold;
  }
  N=n;
  p[N]=0;
  resetIstream();
}

void rai::String::init() { p=0; N=0; M=0; buffer.string=this; flushCallback=NULL; }

/// standard constructor
rai::String::String() : std::iostream(&buffer) { init(); clearStream(); }

/// copy constructor
rai::String::String(const String& s) : std::iostream(&buffer) { init(); this->operator=(s); }

/// copy constructor for an ordinary C-string (needs to be 0-terminated)
rai::String::String(const char *s) : std::iostream(&buffer) { init(); if(s) this->operator=(s); }

rai::String::String(const std::string& s) : std::iostream(&buffer) { init(); this->operator=(s.c_str()); }

rai::String::String(std::istream& is) : std::iostream(&buffer) { init(); read(is, "", "", 0); }

rai::String::~String() { if(M) delete[] p; }

/// returns a reference to this
std::iostream& rai::String::stream() { return (std::iostream&)(*this); }

/// returns a reference to this
rai::String& rai::String::operator()() { return *this; }

/** @brief returns the true memory buffer (C-string) of this class (which is
always kept 0-terminated) */
rai::String::operator char*() { return p; }

/// as above but const
rai::String::operator const char*() const { return p; }

/// returns the i-th char
char& rai::String::operator()(int i) const {
  if(i<0) i+=N;
  CHECK_LE((uint)i, N, "String range error (" <<i <<"<=" <<N <<")");
  return p[i];
}

/// return the substring from 'start' to (exclusive) 'end'.
rai::String rai::String::getSubString(int start, int end) const {
  if(start<0) start+=N;
  if(end<0) end+=N;
  CHECK_GE(start, 0, "start < 0");
  CHECK_LE(end, (int)N, "end out of range");
  CHECK_LE(start ,  end, "end before start");
  String tmp;
  tmp.set(p+start, 1+end-start);
//  for(int i = start; i < end; i++) tmp.append((*this)(i));
  return tmp;
}

/**
 * @brief Return the last 'n' chars of the string.
 * @param n number of chars to return
 */
rai::String rai::String::getLastN(uint n) const {
  return getSubString(-n, -1);
}

/**
 * @brief Return the first 'n' chars of the string.
 * @param n number of chars to return.
 */
rai::String rai::String::getFirstN(uint n) const {
  return getSubString(0, n-1);
}

/// copy operator
rai::String& rai::String::operator=(const String& s) {
  resize(s.N, false);
  memmove(p, s.p, N);
  return *this;
}

rai::String& rai::String::operator=(const std::string& s){
  return this->operator=(s.c_str());
}

/// copies from the C-string
rai::String& rai::String::operator=(const char *s) {
  if(!s) {  clear();  return *this;  }
  uint ls = strlen(s);
  if(!ls) {  clear();  return *this;  }
  if(s>=p && s<=p+N) { //s points to a substring within this string!
    memmove(p, s, ls);
    resize(ls, true);
  } else {
    resize(ls, false);
    memmove(p, s, ls);
  }
  return *this;
}

void rai::String::set(const char *s, uint n) { resize(n, false); memmove(p, s, n); }

rai::String& rai::String::printf(const char *format, ...) {
  resize(100, false);
  va_list valist;
  va_start(valist, format);
  int len = vsnprintf(p, 100, format, valist);
  va_end(valist);
  resize(len, true);
  return *this;
}

/// shorthand for the !strcmp command
bool rai::String::operator==(const char *s) const { return p && !strcmp(p, s); }
/// shorthand for the !strcmp command
bool rai::String::operator==(const String& s) const { return p && s.p && !strcmp(p, s.p); }
bool rai::String::operator!=(const char *s) const { return !operator==(s); }
bool rai::String::operator!=(const String& s) const { return !(operator==(s)); }
bool rai::String::operator<=(const String& s) const { return p && s.p && strcmp(p, s.p)<=0; }

bool rai::String::contains(const String& substring) const {
  if(!p && substring.p) return false;
  if(!substring.p && p) return true;
  char* p = strstr(this->p, substring.p);
  return p != NULL;
}

/// Return true iff the string starts with 'substring'.
bool rai::String::startsWith(const String& substring) const {
  return N>=substring.N && this->getFirstN(substring.N) == substring;
}

/// Return true iff the string starts with 'substring'.
bool rai::String::startsWith(const char* substring) const {
  return this->startsWith(rai::String(substring));
}

/// Return true iff the string ends with 'substring'.
bool rai::String::endsWith(const String& substring) const {
  return this->getLastN(substring.N) == substring;
}
/// Return true iff the string ends with 'substring'.
bool rai::String::endsWith(const char* substring) const {
  return this->endsWith(rai::String(substring));
}

/// deletes all memory and resets all stream flags
rai::String& rai::String::clear() { resize(0, false); return *this; }

/// call IOstream::clear();
rai::String& rai::String::clearStream() { std::iostream::clear(); return *this; }

/** @brief when using this String as an istream (to read other variables
  from it), this method resets the reading-pointer to the beginning
  of the string and also clears all flags of the stream */
rai::String& rai::String::resetIstream() { buffer.setIpos(p); clearStream(); return *this; }

/// writes the string into some ostream
void rai::String::write(std::ostream& os) const { if(N) os <<p; }

/** @brief reads the string from some istream: first skip until one of the stopSymbols
is encountered (default: newline symbols) */
uint rai::String::read(std::istream& is, const char* skipSymbols, const char *stopSymbols, int eatStopSymbol) {
  if(!skipSymbols) skipSymbols=readSkipSymbols;
  if(!stopSymbols) stopSymbols=readStopSymbols;
  if(eatStopSymbol==-1) eatStopSymbol=readEatStopSymbol;
  rai::skip(is, skipSymbols);
  clear();
  char c=is.get();
  while(c!=-1 && is.good() && !rai::contains(stopSymbols, c)) {
    append(c);
    c=is.get();
  }
  if(c==-1) is.clear();
  if(c!=-1 && !eatStopSymbol) is.putback(c);
  return N;
}

//===========================================================================
//
// string-filling routines

/** @brief fills the string with the date and time in the format
 * yy-mm-dd--hh-mm-mm */
rai::String rai::getNowString() {
  time_t t = time(0);
  struct tm *now = localtime(&t);
  
  rai::String str;
  str.resize(19, false); //-- just enough
  sprintf(str.p, "%02d-%02d-%02d-%02d:%02d:%02d",
          now->tm_year-100,
          now->tm_mon+1,
          now->tm_mday,
          now->tm_hour,
          now->tm_min,
          now->tm_sec);
  return str;
}

//===========================================================================
//
// FileToken
//

rai::FileToken::FileToken() {
  cwd = getcwd_string();
}

rai::FileToken::FileToken(const char* filename, bool change_dir) {
  cwd = getcwd_string();
  name = filename;
  if(change_dir) cd_file();
//  if(!exists()) HALT("file '" <<filename <<"' does not exist");
}

rai::FileToken::FileToken(const FileToken& ft) {
  name=ft.name;
  path=ft.path;
  cwd=ft.cwd;
  is = ft.is;
  os = ft.os;
}

rai::FileToken::~FileToken() {}

/// change to the directory of the given filename
void rai::FileToken::decomposeFilename() {
  path = name;
  int i=path.N;
  for(; i--;) if(path(i)=='/' || path(i)=='\\') break;
  if(i==-1) {
    path=".";
  } else {
    path.resize(i, true);
    name = name+i+1;
  }
}

void rai::FileToken::cd_start() {
  LOG(3) <<"entering path '" <<cwd<<"'" <<std::endl;
  if(chdir(cwd)) HALT("couldn't change to directory '" <<cwd <<"'");
}

void rai::FileToken::cd_file() {
  cd_start();
  if(!path.N) decomposeFilename();
  if(path!=".") {
    LOG(3) <<"entering path '" <<path<<"' from '" <<cwd <<"'" <<std::endl;
    if(chdir(path)) HALT("couldn't change to directory '" <<path <<"' from '" <<cwd <<"'");
  }
}

bool rai::FileToken::exists() {
  struct stat sb;
  int r=stat(name, &sb);
  return r==0;
}

std::ofstream& rai::FileToken::getOs(bool change_dir) {
  CHECK(!is,"don't use a FileToken both as input and output");
  if(!os) {
    if(change_dir) cd_file();
    os = std::make_shared<std::ofstream>();
    os->open(name);
    LOG(3) <<"opening output file '" <<name <<"'" <<std::endl;
    if(!os->good()) RAI_MSG("could not open file '" <<name <<"' for output from '" <<cwd <<"./" <<path <<"'");
  }
  return *os;
}

std::ifstream& rai::FileToken::getIs(bool change_dir) {
  CHECK(!os,"don't use a FileToken both as input and output");
  if(!is) {
    if(change_dir) cd_file();
    is = std::make_shared<std::ifstream>();
    is->open(name);
    LOG(3) <<"opening input file '" <<name <<"'" <<std::endl;
    if(!is->good()) THROW("could not open file '" <<name <<"' for input from '" <<cwd <<"./" <<path <<"'");
  }
  return *is;
}

//===========================================================================
//
// random number generator
//

rai::Rnd rnd;

uint32_t rai::Rnd::seed(uint32_t n) {
  uint32_t s, c;
  if(n>12345) { s=n; c=n%113; } else { s=12345; c=n; }
  while(c--) s*=65539;
  s=s>>1;
  seed250(s);
  ready=true;
  return n;
}

uint32_t rai::Rnd::seed() {
  return seed(rai::getParameter<uint32_t>("seed", 0));
}

uint32_t rai::Rnd::clockSeed() {
  uint32_t s;
#ifndef RAI_TIMEB
  timeval t; gettimeofday(&t, 0); s=1000000L*t.tv_sec+t.tv_usec;
#else
  _timeb t; _ftime(&t); s=1000L*t.time+t.millitm;
#endif
  LOG(3) <<"random clock seed: " <<s <<std::endl;
  return seed(s);
}

double rai::Rnd::gauss() {
  double w, v, rsq, fac;
  do {
    v   = 2 * uni() - 1;
    w   = 2 * uni() - 1;
    rsq = v*v + w*w;
  } while(rsq >= 1 || rsq == 0);
  fac  = ::sqrt(-2 * ::log(rsq) / rsq);
  return v*fac;
}

uint32_t rai::Rnd::poisson(double mean) {
  if(mean>100) {
    uint32_t i=(uint32_t)::floor(mean+::sqrt(mean)*gauss()+.5);
    return (i>0)?(uint32_t)i:0;
  }
  uint32_t count = 0;
  double bound, product;
  if(mean>=0) {
    bound=::exp(-mean);
    product=uni();
    while(product>=bound) {
      count++;
      product*=uni();
    }
  }
  return count;
}

void  rai::Rnd::seed250(int32_t seed) {
  int32_t      i;
  int32_t     j, k;
  
  if(seed<=0) seed=1;
  
  for(i=0; i<250; ++i) {  // Schleife ueber Zufallsfeld
    k = seed / 127773;          // Modulozufallszahlengenerator
    seed = 16807 * (seed - k*127773) - 2836 * k;
    if(seed<0) seed += 0x7FFFFFFF;
    rfield[i] = seed;
  }
  
  // Masken ueberlagern
  k = 0x7FFFFFFF;
  j = 0x40000000;
  for(i=1; i<250; i+=8) rfield[i] = (rfield[i] & k) | j;
  
  // rpoint initialisieren
  rpoint = 249;
  
  // Anfangszahlen verwerfen
  for(i=0; i<4711; ++i) rnd250();
}

//===========================================================================
//
// Inotify
//

Inotify::Inotify(const char* filename): fd(0), wd(0) {
  fd = inotify_init();
  if(fd<0) HALT("Couldn't initialize inotify");
  fil = new rai::FileToken(filename, false);
  fil->decomposeFilename();
  wd = inotify_add_watch(fd, fil->path,
                         IN_MODIFY | IN_CREATE | IN_DELETE);
  if(wd == -1) HALT("Couldn't add watch to " <<filename);
  buffer_size = 10*(sizeof(struct inotify_event)+64);
  buffer = new char[buffer_size];
}

Inotify::~Inotify() {
  inotify_rm_watch(fd, wd);
  close(fd);
  delete buffer;
  delete fil;
}

bool Inotify::poll(bool block, bool verbose) {
  if(!block) {
    struct pollfd fd_poll = {fd, POLLIN, 0};
    int r = ::poll(&fd_poll, 1, 0);
    CHECK_GE(r, 0,"poll failed");
    if(!r) return false;
  }
  
  int length = read(fd, buffer, buffer_size);
  CHECK_GE(length, 0, "read failed");
  
  //-- process event list
  for(int i=0; i<length;) {
    struct inotify_event *event = (struct inotify_event *) &buffer[ i ];
    if(verbose) {
      if(event->len) {
        if(event->mask & IN_CREATE)
          cout << "The "
               <<(event->mask&IN_ISDIR?"directory ":"file ")
               <<event->name <<" was created." <<endl;
        if(event->mask & IN_DELETE)
          cout << "The "
               <<(event->mask&IN_ISDIR?"directory ":"file ")
               <<event->name <<" was deleted." <<endl;
        if(event->mask & IN_MODIFY)
          cout << "The "
               <<(event->mask&IN_ISDIR?"directory ":"file ")
               <<event->name <<" was modified." <<endl;
      } else {
        cout <<"event of zero length" <<endl;
      }
    }
    if(event->len
        && (event->mask & (IN_MODIFY|IN_CREATE|IN_DELETE))
        && !strncmp(event->name, fil->name.p, fil->name.N)
      ) return true; //report modification on specific file
    i += sizeof(struct inotify_event) + event->len;
  }
  
  return false;
}

//===========================================================================
//
// Mutex
//

#define MUTEX_DUMP(x) //x

#ifndef RAI_MSVC
Mutex::Mutex() {
  pthread_mutexattr_t atts;
  int rc;
  rc = pthread_mutexattr_init(&atts);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  rc = pthread_mutexattr_settype(&atts, PTHREAD_MUTEX_RECURSIVE_NP);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  rc = pthread_mutex_init(&mutex, &atts);
  //mutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
  state=0;
  recursive=0;
}

Mutex::~Mutex() {
  if(state==-1) { //forced destroy
    int rc = pthread_mutex_destroy(&mutex);
    LOG(-1) <<"pthread forced destroy returned " <<rc <<" '" <<strerror(rc) <<"'";
    return;
  }
  CHECK(!state, "Mutex destroyed without unlocking first");
  int rc = pthread_mutex_destroy(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void Mutex::lock(const char* _lockInfo) {
  int rc = pthread_mutex_lock(&mutex);
  if(rc) {
    //don't use HALT here, because log uses mutexing as well -> can lead to recursive HALT...
    cerr <<STRING("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    exit(1);
  }
  lockInfo = _lockInfo;
  recursive++;
  state=syscall(SYS_gettid);
  MUTEX_DUMP(cout <<"Mutex-lock: " <<state <<" (rec: " <<recursive << ")" <<endl);
}

void Mutex::unlock() {
  MUTEX_DUMP(cout <<"Mutex-unlock: " <<state <<" (rec: " <<recursive << ")" <<endl);
  if(--recursive == 0) state=0;
  int rc = pthread_mutex_unlock(&mutex);
  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

#else//RAI_MSVC
Mutex::Mutex() {}
Mutex::~Mutex() {}
void Mutex::lock() {}
void Mutex::unlock() {}
#endif

//===========================================================================
//
// gnuplot calls
//

struct GnuplotServer {
  FILE *gp;
  GnuplotServer():gp(NULL) {}
  ~GnuplotServer() {
    if(gp) {
      cout <<"Closing Gnuplot" <<endl;
//      send("set terminal wxt nopersist close\nexit", false);
//      fclose(gp);
    }
  }
  
  void send(const char *cmd, bool persist) {
#ifndef RAI_MSVC
    if(!gp) {
      if(!persist) gp=popen("env gnuplot -noraise -geometry 600x600-0-0 2> /dev/null", "w");
      else         gp=popen("env gnuplot -noraise -persist -geometry 600x600-0-0 2> /dev/null", "w");
      CHECK(gp, "could not open gnuplot pipe");
    }
    FILE("z.plotcmd") <<cmd; //for debugging..
    fputs(cmd, gp);
    fflush(gp);
#else
    NIY;
#endif
  }
};

Singleton<GnuplotServer> gnuplotServer;

void gnuplot(const char *command, bool pauseMouse, bool persist, const char *PDFfile) {
  if(!rai::getInteractivity()) {
    pauseMouse=false;
    persist=false;
  }
  
  rai::String cmd;
  cmd <<"set style data lines\n";
  
  // run standard files
  if(!access("~/gnuplot.cfg", R_OK)) cmd <<"load '~/gnuplot.cfg'\n";
  if(!access("gnuplot.cfg", R_OK)) cmd <<"load 'gnuplot.cfg'\n";
  
  cmd <<"set title '(Gui/plot.h -> gnuplot pipe)'\n"
      <<command <<std::endl;
      
  if(PDFfile) {
    cmd <<"set terminal push\n"
        <<"set terminal pdfcairo\n"
        <<"set output '" <<PDFfile <<"'\n"
        <<command <<std::endl
        <<"\nset terminal pop\n";
  }
  
  if(pauseMouse) cmd <<"\n pause mouse" <<std::endl;
  gnuplotServer()->send(cmd.p, persist);
  
  if(!rai::getInteractivity()) {
    rai::wait(.05);
  }
}

//===========================================================================
//
// Cumulative probability for the Standard Normal Distribution
//

namespace rai {
double erf(double x) {
  double t, z, retval;
  z = fabs(x);
  t = 1.0 / (1.0 + 0.5 * z);
  retval = t * exp(-z * z - 1.26551223 + t *
                   (1.00002368 + t *
                    (0.37409196 + t *
                     (0.09678418 + t *
                      (-0.18628806 + t *
                       (0.27886807 + t *
                        (-1.13520398 + t *
                         (1.48851587 + t *
                          (-0.82215223 + t *
                           0.1708727)))))))));
  if(x < 0.0) return retval - 1.0;
  return 1.0 - retval;
}

/// the integral of N(0, 1) from -infty to x
double gaussInt(double x) {
  return .5*(1.+erf(x/RAI_SQRT2));
}

/// expectation \f$\int_x^\infty {\cal N}(x) x dx\f$ when integrated from -infty to x
double gaussIntExpectation(double x) {
  double norm=gaussInt(x) / (::sqrt(RAI_2PI));
  return - norm*rai::approxExp(-.5*x*x);
}
}

//===========================================================================
// MISC

/**
 * @brief Return the current working dir as std::string.
 */
std::string getcwd_string() {
  char buff[PATH_MAX];
  char *succ=getcwd(buff, PATH_MAX);
  if(!succ) HALT("could not call getcwd: errno=" <<errno <<' ' <<strerror(errno));
  return std::string(buff);
}

const char* NAME(const std::type_info& type) {
  const char* name = type.name();
  while(*name>='0' && *name<='9') name++;
  return name;
}

//===========================================================================
//
// explicit instantiations
//

#include "util.tpp"
template void rai::getParameter(int&, const char*);
template void rai::getParameter(int&, const char*, const int&);
template void rai::getParameter(uint&, const char*);
template void rai::getParameter(uint&, const char*, const uint&);
template void rai::getParameter(bool&, const char*, const bool&);
template void rai::getParameter(double&, const char*);
template void rai::getParameter(double&, const char*, const double&);
template void rai::getParameter(rai::String&, const char*, const rai::String&);
template void rai::getParameter(rai::String&, const char*);

template int rai::getParameter<int>(const char*);
template int rai::getParameter<int>(const char*, const int&);
template uint rai::getParameter(const char*);
template uint rai::getParameter<uint>(const char*, const uint&);
template float rai::getParameter<float>(const char*);
template double rai::getParameter<double>(const char*);
template double rai::getParameter<double>(const char*, const double&);
template bool rai::getParameter<bool>(const char*);
template bool rai::getParameter<bool>(const char*, const bool&);
template long rai::getParameter<long>(const char*);
template rai::String rai::getParameter<rai::String>(const char*);
template rai::String rai::getParameter<rai::String>(const char*, const rai::String&);

template bool rai::checkParameter<uint>(const char*);
template bool rai::checkParameter<int>(const char*);
template bool rai::checkParameter<bool>(const char*);
template bool rai::checkParameter<rai::String>(const char*);

