/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/// @file
/// @ingroup group_Core
/// @addtogroup group_Core
/// @{

#ifndef RAI_util_h
#define RAI_util_h

#include <iostream>
#include <fstream>
#include <typeinfo>
#include <stdint.h>
#include <string.h>
#include <memory>
#include <climits>

//----- if no system flag, I assume Linux
#if !defined RAI_MSVC && !defined RAI_Cygwin && !defined RAI_Linux && !defined RAI_MinGW && !defined RAI_Darwin
#  define RAI_Linux
#endif

//----- basic defs:
#define RAI_PI 3.14159265358979323846
#define RAI_LN2 0.69314718055994528622676398299518041312694549560546875
#define RAI_2PI 6.283195307179587
#define RAI_LnSqrt2Pi -0.9189385332046727417803296
#define RAI_SQRT2 1.414213562373095049
#define RAI_SQRTPI 1.772453850905516027

//===========================================================================
//
// types
//

typedef unsigned char byte;            ///< byte
typedef unsigned int uint;             ///< unsigned integer
typedef const char* charp;

//===========================================================================
//
// using
//

using std::cout;
using std::cerr;
using std::endl;
using std::flush;
using std::ostream;
using std::istream;
using std::ofstream;
using std::ifstream;
template<class T> using ptr=std::shared_ptr<T>;
using std::make_shared;


//----- macros to define the standard <<and >>operatos for most my classes:
#define stdInPipe(type)\
  inline std::istream& operator>>(std::istream& is, type& x){ x.read(is); return is; }
#define stdOutPipe(type)\
  inline std::ostream& operator<<(std::ostream& os, const type& x){ x.write(os); return os; }
#define stdPipes(type)\
  inline std::istream& operator>>(std::istream& is, type& x){ x.read(is); return is; }\
  inline std::ostream& operator<<(std::ostream& os, const type& x){ x.write(os); return os; }
#define inPipe(type)\
  inline type& operator<<(type& x, const char* str){ std::istringstream ss(str); ss >>x; return x; }
#define niyPipes(type)\
  inline std::istream& operator>>(std::istream& is, type& x){ NIY; return is; }\
  inline std::ostream& operator<<(std::ostream& os, const type& x){ NIY; return os; }

//----- macros for piping doubles EXACTLY (without rounding errors) in hex coding:
#define OUTHEX(y) "0x" <<std::hex <<*((unsigned long*)&y) <<std::dec
#define INHEX(y)  std::hex >>*((unsigned long*)&y) >>std::dec

//===========================================================================
//
// standard little methods in my namespace (this needs cleanup)
//

namespace rai {
extern int argc;
extern char** argv;
extern bool IOraw;  ///< stream modifier for some classes (Mem in particular)
extern uint lineCount;
extern int verboseLevel;
extern int interactivity;

void system(const char* cmd);

//----- files
void open(std::ofstream& fs, const char *name, const char *errmsg="");
void open(std::ifstream& fs, const char *name, const char *errmsg="");

//----- basic ui
int x11_getKey();

//----- strings and streams
bool contains(const char *s, char c);
char skip(std::istream& is, const char *skipSymbols=" \n\r\t", const char *stopSymbols=NULL, bool skipCommentLines=true);
void skipRestOfLine(std::istream& is);
void skipOne(std::istream& is);
char getNextChar(std::istream& is, const char *skipSymbols=" \n\r\t", bool skipCommentLines=true);
char peerNextChar(std::istream& is, const char *skipSymbols=" \n\r\t", bool skipCommentLines=true);
bool parse(std::istream& is, const char *str, bool silent=false);
bool skipUntil(std::istream& is, const char *tag);

//----- functions
byte bit(byte *str, uint i);
void flip(byte& b, uint i);
void flip(int& b, uint i);
double MIN(double a, double b);
double MAX(double a, double b);
uint MAX(uint a, uint b);
int MAX(int a, int b);
double indicate(bool expr);
double modMetric(double x, double y, double mod);
double sign(double x);
double sign0(double x);
double linsig(double x);
//void   clip(double& x, double a, double b);
double phi(double dx, double dy);
double dphi(double x, double y, double dx, double dy);
double DIV(double x, double y, bool force=false);
double sigmoid11(double x);
double sigmoid(double x);
double dsigmoid(double x);
double approxExp(double x);
double Log(double x);
uint   Log2(uint n);
double sqr(double x);
double sinc(double x);
double cosc(double x);
double erf(double x);
double gaussInt(double x);
double gaussIntExpectation(double x);
double NNsdv(const double& a, const double& b, double sdv);
double NNsdv(double x, double sdv);
double smoothRamp(double x, double eps, double power);
double d_smoothRamp(double x, double eps, double power);

double ineqConstraintCost(double g, double margin, double power);
double d_ineqConstraintCost(double g, double margin, double power);
double eqConstraintCost(double h, double margin, double power);
double d_eqConstraintCost(double h, double margin, double power);

//----- time access
double clockTime(bool today=true); //(really on the clock)
timespec clockTime2();
double realTime(); //(since process start)
double cpuTime();
double sysTime();
double totalTime();
double toTime(const tm& t);
char *date();
char *date(double sec);
char *date2(bool subsec=false);
char *date2(double sec, bool subsec);
void wait(double sec, bool msg_on_fail=true);
bool wait(bool useX11=true);

//----- memory
long mem();

//----- timer functions
double timerStart(bool useRealTime=false);
double timerRead(bool reset=false);
double timerRead(bool reset, double startTime);
double timerPause();
void   timerResume();

//----- command line handling
void initCmdLine(int _argc, char *_argv[]);
bool checkCmdLineTag(const char *tag);
char *getCmdLineArgument(const char *tag);

//----- parameter grabbing from command line, config file, or default value
template<class T> T getParameter(const char *tag);
template<class T> T getParameter(const char *tag, const T& Default);
template<class T> void getParameter(T& x, const char *tag, const T& Default);
template<class T> void getParameter(T& x, const char *tag);
template<class T> bool checkParameter(const char *tag);

template<class T> void putParameter(const char* tag, const T& x);
template<class T> bool getFromMap(T& x, const char* tag);

//----- get verbose level
uint getVerboseLevel();
bool getInteractivity();
}

//----- stream parsing
struct PARSE { const char *str; PARSE(const char* _str):str(_str) {} };
std::istream& operator>>(std::istream& is, const PARSE&);

//===========================================================================
//
// String class
//

#define STRING(x) (((rai::String&)(rai::String().stream() <<x)))
#define STRINGF(format,...) (rai::String().printf(format, __VA_ARGS__))
#define STREAM(x) (((rai::String&)(rai::String().stream() <<x)).stream())

namespace rai {
/** @brief String implements the functionalities of an ostream and an
istream, but also can be send to an ostream or read from an
istream. It is based on a simple streambuf derived from the
rai::Mem class */
struct String:public std::iostream {
private:
  struct StringBuf:std::streambuf {
    String *string;
    virtual int overflow(int C = traits_type::eof());
    virtual int sync();
    void setIpos(char *p);
    char *getIpos();
  } buffer;
  void init();
  
public:
  /// @name data fields
  char *p;    ///< pointer to memory
  uint N;     ///< \# elements (excluding zero)
  uint M;     ///< actual buffer size (in terms of # elements)
  static const char *readSkipSymbols; ///< default argument to read method (also called by operator>>)
  static const char *readStopSymbols; ///< default argument to read method (also called by operator>>)
  static int readEatStopSymbol;       ///< default argument to read method (also called by operator>>)
  void (*flushCallback)(String&);
  
  /// @name constructors
  String();
  String(const String& s);
  String(const char *s);
  explicit String(const std::string& s);
  explicit String(std::istream& is);
  ~String();
  
  /// @name access
  operator char*();
  operator const char*() const;
  char &operator()(int i) const;
  std::iostream& stream();            ///< explicitly returns this as an std::iostream&
  String& operator()();               ///< explicitly return this as a (non-const!) String&
  String getSubString(int start, int end) const;
  String getFirstN(uint n) const;
  String getLastN(uint n) const;
  
  /// @name setting
  String& operator=(const String& s);
  String& operator=(const std::string& s);
  String& operator=(const char *s);
  void set(const char *s, uint n);
  String& printf(const char *format, ...);
  void resize(uint n, bool copy); //low-level resizing the string buffer - with additinal final 0
  void append(char x);
  void prepend(const String& s);
  String& setRandom();
  
  /// @name resetting
  String& clear();       //as with Array: resize(0)
  String& clearStream(); //call IOstream::clear();
  String& resetIstream();
  
  /// @name equality
  bool operator==(const char *s) const;
  bool operator==(const String& s) const;
  bool operator!=(const char *s) const;
  bool operator!=(const String& s) const;
  bool operator<=(const String& s) const;
  
  /// @name misc
  bool contains(const String& substring) const;
  bool startsWith(const String& substring) const;
  bool startsWith(const char* substring) const;
  bool endsWith(const String& substring) const;
  bool endsWith(const char* substring) const;
  
  /// @name I/O
  void write(std::ostream& os) const;
  uint read(std::istream& is, const char* skipSymbols=NULL, const char *stopSymbols=NULL, int eatStopSymbol=-1);
};
stdPipes(String)
}

inline rai::String operator+(const rai::String& a, const char* b) { rai::String s=a; s <<b; return s; }

//===========================================================================
//
// string-filling routines

namespace rai {
rai::String getNowString();
}

//===========================================================================
//
// logging

namespace rai {
/// An object that represents a log file and/or cout logging, together with log levels read from a cfg file
struct LogObject {
  std::ofstream fil;
  const char* key;
  int logCoutLevel, logFileLevel;
  LogObject(const char* key, int defaultLogCoutLevel=0, int defaultLogFileLevel=0);
  ~LogObject();
  LogObject& getNonConst() const { return *((LogObject*)this); } //ugly... but Logs are often members of classes, and they are accessed in const methods of these classes...
  struct LogToken getToken(int log_level, const char *code_file, const char *code_func, uint code_line);
};

/// A Token to such a Log object which, on destruction, writes into the Log
struct LogToken {
  rai::String msg;
  LogObject& log;
  int log_level;
  const char *code_file, *code_func;
  uint code_line;
  LogToken(LogObject& log, int log_level, const char *code_file, const char *code_func, uint code_line)
    : log(log), log_level(log_level), code_file(code_file), code_func(code_func), code_line(code_line) {}
  ~LogToken(); //that's where the magic happens!
  std::ostream& os() { return msg; }
};
}

extern rai::LogObject _log;

#define LOG(log_level) _log.getNonConst().getToken(log_level, __FILE__, __func__, __LINE__).os()

void setLogLevels(int fileLogLevel=3, int consoleLogLevel=2);

//The destructor ~LogToken writes into the log file and
//console. setLogLevel allows to adjust cout verbosity (0 by default),
//and what is written into the log file (1 by default)

//===========================================================================
//
// macros for halting/MSGs etc
//


//----- error handling:
//#define RAI_HERE __FILE__<<':' <<__FUNCTION__ <<':' <<__LINE__ <<' ' //":" <<std::setprecision(5) <<rai::realTime() <<"s "
#define S1(x) #x
#define S2(x) S1(x)
#define RAI_HERE __FILE__ ":" S2(__LINE__)
//#define RAI_HERE __FILE__ ## ":" ## #__FUNCTION__ ## ":" ## #__LINE__


namespace rai {
extern String errString;
}

#ifndef HALT
#  define RAI_MSG(msg){ LOG(-1) <<msg; }
#  define THROW(msg){ LOG(-1) <<msg; throw std::runtime_error(rai::errString.p); }
#  define HALT(msg){ LOG(-2) <<msg; throw std::runtime_error(rai::errString.p); exit(1); }
#  define NIY  { LOG(-2) <<"not implemented yet"; exit(1); }
#  define NICO { LOG(-2) <<"not implemented with this compiler options: usually this means that the implementation needs an external library and a corresponding compiler option - see the source code"; exit(1); }
#endif

//----- check macros:
#ifndef RAI_NOCHECK

#define CHECK(cond, msg) \
  if(!(cond)){ LOG(-2) <<"CHECK failed: '" <<#cond <<"' " <<msg;  throw std::runtime_error(rai::errString.p); }\

#define CHECK_ZERO(expr, tolerance, msg) \
  if(fabs((double)(expr))>tolerance){ LOG(-2) <<"CHECK_ZERO failed: '" <<#expr<<"'=" <<expr <<" > " <<tolerance <<" -- " <<msg; throw std::runtime_error(rai::errString.p); } \

#define CHECK_EQ(A, B, msg) \
  if(!(A==B)){ LOG(-2) <<"CHECK_EQ failed: '" <<#A<<"'=" <<A <<" '" <<#B <<"'=" <<B <<" -- " <<msg; throw std::runtime_error(rai::errString.p); } \

#define CHECK_GE(A, B, msg) \
  if(!(A>=B)){ LOG(-2) <<"CHECK_GE failed: '" <<#A<<"'=" <<A <<" '" <<#B <<"'=" <<B <<" -- " <<msg; throw std::runtime_error(rai::errString.p); } \

#define CHECK_LE(A, B, msg) \
  if(!(A<=B)){ LOG(-2) <<"CHECK_LE failed: '" <<#A<<"'=" <<A <<" '" <<#B <<"'=" <<B <<" -- " <<msg; throw std::runtime_error(rai::errString.p); } \

#else
#define CHECK(cond, msg)
#define CHECK_ZERO(expr, tolerance, msg)
#define CHECK_EQ(A, B, msg)
#define CHECK_GE(A, B, msg)
#define CHECK_LE(A, B, msg)
#endif

//----- TESTING
#ifndef EXAMPLES_AS_TESTS
#  define TEST(name) test##name()
#  define MAIN main
#else
#  define GTEST_DONT_DEFINE_TEST 1
#  include <gtest/gtest.h>
#  define TEST(name) test##name(){} GTEST_TEST(examples, name)
#  define MAIN \
  main(int argc, char** argv){               \
    rai::initCmdLine(argc,argv);             \
    testing::InitGoogleTest(&argc, argv);  \
    return RUN_ALL_TESTS();      \
  }                                          \
  inline int obsolete_main //this starts a method declaration
#endif

//----- verbose:
#define VERBOSE(l, x) if(l<=rai::getVerboseLevel()) x;

//----- other macros:
#define MEM_COPY_OPERATOR(x) memmove(this, &x, sizeof(this));

//===========================================================================
//
// FileToken
//

namespace rai {

//String raiPath(const char* rel=NULL);

/** @brief A ostream/istream wrapper that allows easier initialization of objects, like:
arr X = FILE("inname");
X >>FILE("outfile");
etc
*/
struct FileToken {
  rai::String path, name, cwd;
  std::shared_ptr<std::ofstream> os;
  std::shared_ptr<std::ifstream> is;
  
  FileToken();
  FileToken(const char* _filename, bool change_dir=false);
  FileToken(const FileToken& ft);
  ~FileToken();
  FileToken& operator()() { return *this; }
  
  void decomposeFilename();
  void cd_start();
  void cd_file();
  bool exists();
  std::ofstream& getOs(bool change_dir=false);
  std::ifstream& getIs(bool change_dir=false);
  operator std::istream&() { return getIs(); }
  operator std::ostream&() { return getOs(); }
};
template<class T> FileToken& operator>>(FileToken& fil, T& x) { fil.getIs() >>x;  return fil; }
template<class T> std::ostream& operator<<(FileToken& fil, const T& x) { fil.getOs() <<x;  return fil.getOs(); }
inline std::ostream& operator<<(std::ostream& os, const FileToken& fil) { return os <<fil.name; }
template<class T> FileToken& operator<<(T& x, FileToken& fil) { fil.getIs() >>x; return fil; }
template<class T> void operator>>(const T& x, FileToken& fil) { fil.getOs() <<x; }
}
#define FILE(filename) (rai::FileToken(filename, false)()) //it needs to return a REFERENCE to a local scope object

inline bool operator==(const rai::FileToken&, const rai::FileToken&) { return false; }

//===========================================================================
//
// give names to Enum (for pipe << >> )
//

namespace rai {
template<class enum_T>
struct Enum {
  enum_T x;
  static const char* names [];
  Enum():x((enum_T)-1) {}
  explicit Enum(const enum_T& y):x(y) {}
  explicit Enum(const rai::String& str):Enum() { operator=(str); }
  const enum_T& operator=(const enum_T& y) { x=y; return x; }
  bool operator==(const enum_T& y) const { return x==y; }
  bool operator!=(const enum_T& y) const { return x!=y; }
  operator enum_T() const { return x; }
  void read(std::istream& is) {
    rai::String str(is);
    operator=(str);
  }
  void operator=(const char* str) {
    operator=(STRING(str));
  }
  void operator=(const rai::String& str) {
    bool good=false;
    for(int i=0; names[i]; i++) {
      const char* n = names[i];
      if(!n) LOG(-2) <<"enum_T " <<typeid(enum_T).name() <<' ' <<str <<" out of range";
      if(str==n) { x=(enum_T)(i); good=true; break; }
    }
    if(!good) {
      rai::String all;
      for(int i=0; names[i]; i++) all <<names[i] <<' ';
      LOG(-2) <<"Enum::read could not find the keyword '" <<str <<"'. Possible Enum keywords: " <<all;
    }
    CHECK(!strcmp(names[x], str.p), "");
  }
  static bool contains(const rai::String& str){
    for(int i=0; names[i]; i++) {
      if(str==names[i]) return true;
    }
    return false;
  }
  static const char* name(int i){
    return names[i];
  }
  const char* name() const {
    if(x<0) return "init";
    else return names[x];
  }
  void write(std::ostream& os) const { os <<name(); }
};
template<class T> std::istream& operator>>(std::istream& is, Enum<T>& x) { x.read(is); return is; }
template<class T> std::ostream& operator<<(std::ostream& os, const Enum<T>& x) { x.write(os); return os; }
}

//===========================================================================
//
// random number generator
//

namespace rai {
/** @brief A random number generator. An global instantiation \c
  rai::rnd of a \c Rnd object is created. Use this one object to get
  random numbers.*/
class Rnd {
private:
  bool ready;
  int32_t rpoint;     /* Feldindex    */
  int32_t rfield[256];   /* Schieberegisterfeld  */
  
  
public:
  /// ...
  Rnd() { ready=false; }
  
  
public:/// @name initialization
  /// initialize with a specific seed
  uint32_t seed(uint32_t n);
  
  /// use Parameter<uint>("seed") as seed
  uint32_t seed();
  
  /// uses the internal clock to generate a seed
  uint32_t clockSeed();
  
public:/// @name access
  /// a initeger random number uniformly distributed in [0, ?]
  uint32_t num() { if(!ready) seed(); return (uint32_t)rnd250() >>5; }
  /// same as \c num()
  uint32_t operator()() { return num(); }
  /// a initeger random number uniformly distributed in [0, \c i-1]
  uint32_t num(uint32_t limit) {
    CHECK(limit, "zero limit in rnd.num()"); return num() % limit;
  }
  uint32_t num(int32_t lo, int32_t hi) { return lo+num(hi-lo+1); }
  /// same as \c num(i)
  uint32_t operator()(uint32_t i) { return num(i); }
  uint32_t operator()(int32_t lo, int32_t hi) { return num(lo, hi); }
  /// a random variable uniformly distributed in [0, 1]
  double uni() { return ((double)num(1 <<22))/(1 <<22); }
  /// a random variable uniformly distributed in [\c low, \c high]
  double uni(double low, double high) { return low+uni()*(high-low); }
  /// a gaussian random variable with mean zero
  double gauss();
  /** @brief a positive integer drawn from a poisson distribution with given
    \c mean; is case \c mean>100, a (positive) gauss number
    \c floor(mean+gauss(sqrt(mean))+.5) is returned */
  uint32_t poisson(double mean);
  
private:
  int32_t rnd250() {
    rpoint = (rpoint+1) & 255;          // Index erhoehen
    return rfield[rpoint] =  rfield[(rpoint-250) & 255]
                             ^ rfield[(rpoint-103) & 255];
  }
  
  void seed250(int32_t seed);
};

}
/// The global Rnd object
extern rai::Rnd rnd;

//===========================================================================
//
/// a little inotify wrapper
//

struct Inotify {
  int fd, wd;
  char *buffer;
  uint buffer_size;
  rai::FileToken *fil;
  Inotify(const char *filename);
  ~Inotify();
  bool poll(bool block=false, bool verbose=false);
  
//  void waitAndReport(){ pollForModification(false, true); }
//  void waitForModification(bool verbose=false){ while(!pollForModification(true, verbose)); }
};

//===========================================================================
//
/// a basic mutex lock
//

struct Mutex {
#ifndef RAI_MSVC
  pthread_mutex_t mutex;
#endif
  int state; ///< 0=unlocked, otherwise=syscall(SYS_gettid)
  uint recursive; ///< number of times it's been locked
  const char* lockInfo;
  Mutex();
  ~Mutex();
  void lock(const char *_lockInfo);
  void unlock();
  
  struct Token {
    Mutex &m;
    Token(Mutex& m, const char *_lockInfo):m(m) { m.lock(_lockInfo); }
    ~Token() { m.unlock(); }
  };
  struct Token operator()(const char *_lockInfo) { return Token(*this, _lockInfo); }

  template<class T> struct TypedToken {
    Mutex &m;
    T *data;
    TypedToken(Mutex& m, T *data, const char *_lockInfo):m(m),data(data) { m.lock(_lockInfo); }
    ~TypedToken() { m.unlock(); }
    T* operator->() { return data; }
  };
  template<class T> TypedToken<T> operator()(T *data, const char *_lockInfo) { return TypedToken<T>(*this, data, _lockInfo); }
};

//===========================================================================
//
/// a generic singleton
//

template<class T>
struct Singleton {
  static Mutex mutex;
  static T *singleton;
  
  T *getSingleton() const {
    if(!singleton) {
      mutex.lock(RAI_HERE);
      if(!singleton) singleton = new T;
      mutex.unlock();
    }
    return singleton;
  }
  
  ~Singleton() {
    if(singleton) {
//      static Mutex m; //pthread might already be deinitialized...
//      m.lock();
      T *mine=singleton;
      singleton=NULL;
      if(mine) delete mine;
//      m.unlock();
    }
  }
  
  struct Token {
    const Singleton<T>& base;
    Token(Singleton<T>& _base) : base(_base) { base.getSingleton(); base.mutex.lock(RAI_HERE); }
    ~Token() { base.mutex.unlock(); }
    T* operator->() { return base.getSingleton(); }
    operator T&() { return *base.getSingleton(); }
    T& operator()() { return *base.getSingleton(); }
  };
  
  Token operator()() { return Token(*this); }
};

template<class T> T *Singleton<T>::singleton=NULL;
template<class T> Mutex Singleton<T>::mutex;

//===========================================================================
//
// just a hook to make things gl drawable
//

struct GLDrawer    { virtual void glDraw(struct OpenGL&) = 0; virtual ~GLDrawer() {} };

//===========================================================================

struct NonCopyable {
  NonCopyable & operator=(const NonCopyable&) = delete;
  NonCopyable(const NonCopyable&) = delete;
  NonCopyable() = default;
};

//===========================================================================
//
/// a mutexed cout
//

extern Mutex coutMutex;
struct CoutToken {
  CoutToken() { coutMutex.lock(RAI_HERE); }
  ~CoutToken() { coutMutex.unlock(); }
  std::ostream& getOs() { return std::cout; }
};
#define COUT (CoutToken().getOs())

//===========================================================================
//
// to register a type (instead of general thing/object), use this:
//

struct Type {
  virtual ~Type() {}
  virtual const std::type_info& typeId() const {NIY}
  virtual struct Node* readIntoNewNode(struct Graph& container, std::istream&) const {NIY}
  virtual void* newInstance() const {NIY}
  void write(std::ostream& os) const {  os <<"Type '" <<typeId().name() <<"' ";  }
  void read(std::istream& is) const {NIY}
};
stdPipes(Type)

template<class T>
struct Type_typed : Type {
  virtual const std::type_info& typeId() const { return typeid(T); }
  virtual void* newInstance() const { return new T(); }
};

inline bool operator!=(Type& t1, Type& t2) { return t1.typeId() != t2.typeId(); }
inline bool operator==(Type& t1, Type& t2) { return t1.typeId() == t2.typeId(); }

//===========================================================================
//
/// running code on init (in cpp files)
//

#define RUN_ON_INIT_BEGIN(key) struct key##_RUN_ON_INIT{ key##_RUN_ON_INIT(){
#define RUN_ON_INIT_END(key)   } } key##_RUN_ON_INIT_dummy;

//===========================================================================
//
// gnuplot calls
//

void gnuplot(const char *command, bool pauseMouse=false, bool persist=false, const char* PDFfile=NULL);
void gnuplotClose();

//===========================================================================
//
// Stefan's misc
//

/// Clip the `value` of n between `lower` and `upper`.
template <typename T> T clip(T& x, const T& lower, const T& upper) {
  if(x<lower) x=lower; else if(x>upper) x=upper; return x;
}

std::string getcwd_string();
const char* NAME(const std::type_info& type);

#endif

/// @} //end group
