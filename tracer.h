#pragma once

#include <stdio.h>

//tracer routines (shall be moved from here)

#ifdef _DEBUG_

static void trace(const char* file, int line, const char* capt, const char* mesg) {
    uint64_t tsms = time_us_64() / 1000;
    printf("%s:%d [%llu] [%s %s]\n",file,line,tsms,capt,mesg);
}
static void traceint(const char* file, int line, const char* capt, long intval) {
    uint64_t tsms = time_us_64() / 1000;
    printf("%s:%d [%llu] [%s %d]\n",file,line,tsms,capt,intval);
}

#define TRACE(c) { trace(__FILE__,__LINE__,(c),""); }
#define TRACEstr(c,v) { trace(__FILE__,__LINE__,(c),(v)); }
#define TRACEint(c,i) { traceint(__FILE__,__LINE__,(c),(i)); }

#else 

#define TRACE(c) {}
#define TRACEstr(c,v) {}
#define TRACEint(c,i) {}

#endif
