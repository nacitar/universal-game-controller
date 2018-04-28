#ifndef INCLUDED_UGC_CONTROLLER_ID_H_
#define INCLUDED_UGC_CONTROLLER_ID_H_

#include <linux/input.h>  // strcpy
#include <linux/string.h>  // strcpy
#include <linux/slab.h>  // kmalloc, GFP_KERNEL, ...

char *CopyString(const char *source);

struct ControllerId {
  char* name;
  char* uniq;
  char* phys;
  __u16 bustype;
};

void ControllerId_Init(struct ControllerId* id);
struct ControllerId* ControllerId_New(void);
void ControllerId_Clear(struct ControllerId* id);
void ControllerId_Delete(struct ControllerId* id);
void ControllerId_Set(struct ControllerId* id, const char* name,
    const char* uniq, const char* phys, __u16 bustype);
void ControllerId_Populate(struct ControllerId* id,
    struct input_handle *handle);

#endif  // INCLUDED_UGC_CONTROLLER_ID_H_
