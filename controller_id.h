#ifndef INCLUDED_UNIVERSAL_INPUT_CONTROLLER_ID_H
#define INCLUDED_UNIVERSAL_INPUT_CONTROLLER_ID_H

#include <linux/string.h>  // strcpy
#include <linux/slab.h>  // kmalloc, GFP_KERNEL, ...

char *copy_string(const char *source) {
  char *destination;
  if (source) {
    destination = strcpy(kmalloc(strlen(source)+1, GFP_KERNEL), source);
  } else {
    destination = kmalloc(1, GFP_KERNEL);
    *destination = '\0';
  }
  return destination;
}
struct controller_id {
  char* name;
  char* uniq;
  char* phys;
  __u16 bustype;
};

void controller_id_init(struct controller_id* id) {
  id->name = NULL;
  id->uniq = NULL;
  id->phys = NULL;
  id->bustype = 0;
}

struct controller_id* controller_id_new(void) {
  struct controller_id* id = kmalloc(sizeof(struct controller_id), GFP_KERNEL);
  controller_id_init(id);
  return id;
}
void controller_id_clear(struct controller_id* id) {
  kfree(id->name);
  kfree(id->uniq);
  kfree(id->phys);
  controller_id_init(id);
}

void controller_id_delete(struct controller_id* id) {
  if (id) {
    controller_id_clear(id);
    kfree(id);
  }
}

void controller_id_set(struct controller_id* id, const char* name,
    const char* uniq, const char* phys, __u16 bustype) {
  controller_id_clear(id);
  id->name = copy_string(name);
  id->uniq = copy_string(uniq);
  id->phys = copy_string(phys);
  id->bustype = bustype;
}

void controller_id_populate(struct controller_id* id, struct input_handle *handle) {
  controller_id_set(id, handle->dev->name, handle->dev->uniq,
      handle->dev->phys, handle->dev->id.bustype);
}

#endif  // INCLUDED_UNIVERSAL_INPUT_CONTROLLER_ID_H
