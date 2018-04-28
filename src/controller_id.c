#include <ugc/controller_id.h>

char *CopyString(const char *source) {
  char *destination;
  if (source) {
    destination = strcpy(kmalloc(strlen(source)+1, GFP_KERNEL), source);
  } else {
    destination = kmalloc(1, GFP_KERNEL);
    *destination = '\0';
  }
  return destination;
}

void ControllerId_Init(struct ControllerId* id) {
  id->name = NULL;
  id->uniq = NULL;
  id->phys = NULL;
  id->bustype = 0;
}

struct ControllerId* ControllerId_New(void) {
  struct ControllerId* id = kmalloc(sizeof(struct ControllerId), GFP_KERNEL);
  ControllerId_Init(id);
  return id;
}

void ControllerId_Clear(struct ControllerId* id) {
  kfree(id->name);
  kfree(id->uniq);
  kfree(id->phys);
  ControllerId_Init(id);
}

void ControllerId_Delete(struct ControllerId* id) {
  if (id) {
    ControllerId_Clear(id);
    kfree(id);
  }
}

void ControllerId_Set(struct ControllerId* id, const char* name,
    const char* uniq, const char* phys, __u16 bustype) {
  ControllerId_Clear(id);
  id->name = CopyString(name);
  id->uniq = CopyString(uniq);
  id->phys = CopyString(phys);
  id->bustype = bustype;
}

void ControllerId_Populate(struct ControllerId* id,
    struct input_handle *handle) {
  ControllerId_Set(id, handle->dev->name, handle->dev->uniq,
      handle->dev->phys, handle->dev->id.bustype);
}
