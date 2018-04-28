#ifndef INCLUDED_UGC_INFO_STRINGS_H_
#define INCLUDED_UGC_INFO_STRINGS_H_

#include <linux/input.h>
int IsBitmapSet(unsigned int flag, unsigned long *bm, unsigned int max);
//IsBitmapSet(code, dev->keybit, KEY_MAX)
// Applies to propbit values too.

void GetBusName(__u16 bustype, const char** bus_name);
void GetKeyName(unsigned int code, const char** code_name);
void GetEventName(unsigned int type, unsigned int code,
      const char** event_name, const char** code_name);

#endif  // INCLUDED_UGC_INFO_STRINGS_H_
