/*
 *  Copyright (c) 1999-2001 Vojtech Pavlik
 */

/*
 *  Input driver event debug module - dumps all events into syslog
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Should you need to contact me, the author, you can do so either by
 * e-mail - mail your message to <vojtech@ucw.cz>, or by paper mail:
 * Vojtech Pavlik, Simunkova 1594, Prague 8, 182 00 Czech Republic
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "info_strings.h"
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/device.h>

MODULE_AUTHOR("Jacob McIntosh <nacitar@ubercpp.com>");
MODULE_DESCRIPTION("Module to hook into the kernel's input subsystem.");
MODULE_LICENSE("GPL");


// Track all deviced by name, id, and physical location (not dev node)


// https://www.kernel.org/doc/Documentation/input/event-codes.txt
// https://github.com/torvalds/linux/blob/master/include/uapi/linux/input-event-codes.h

// figure out how to handle bluetooth!
// upon connecting, identify the controller, see if it has settings stored.
// use the pointer to lookup the value in a sorted list(set)
// figure out a way to enter programming mode, and setup bindings.
// figure out a way to save those bindings
// ignore axis movement that isn't 50% or more (how do we know extents?)
// need some sort of external button for programming inputs and pairing.
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

char* copy_string(const char* source) {
  char* destination;
  if (source) {
    destination = strcpy(kmalloc(strlen(source)+1, GFP_KERNEL), source);
  } else {
    destination = kmalloc(1, GFP_KERNEL);
    *destination = '\0';
  }
  return destination;
}

void controller_id_set(struct controller_id* id, const char* name,
    const char* uniq, const char* phys, __u16 bustype) {
  controller_id_clear(id);
  id->name = copy_string(name);
  id->uniq = copy_string(uniq);
  id->phys = copy_string(phys);
  id->bustype = bustype;
}

static void evbug_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
  const char *event_name, *code_name, *bus_name;
  get_event_name(type, code, &event_name, &code_name);
  get_bus_name(handle->dev->id.bustype, &bus_name);
  if (!event_name) {
    event_name = "UNKNOWN";
  }
  if (!code_name) {
    code_name = "UNKNOWN";
  }
  printk(KERN_DEBUG pr_fmt("Event. Dev: %s, Type: %s[%d], Code: %s[%d], Value: %d\n"),
      dev_name(&handle->dev->dev), event_name, type, code_name, code, value);
  if (type == EV_KEY) {
    printk(KERN_DEBUG pr_fmt("Event. Dev: %s, Dev PTR: %p, Name: %s, Phys: %s, Uniq: %s,"
          " Bus: %s, Vendor: %d, Product: %d, Version: %d\n"),
        dev_name(&handle->dev->dev),
        handle->dev,
        handle->dev->name,
        handle->dev->phys,
        handle->dev->uniq,
        bus_name,
        handle->dev->id.vendor,
        handle->dev->id.product,
        handle->dev->id.version
        );
  }
}

static int evbug_connect(struct input_handler *handler, struct input_dev *dev,
    const struct input_device_id *id)
{
  struct input_handle *handle;
  int error;

  handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
  if (!handle)
    return -ENOMEM;

  handle->dev = dev;
  handle->handler = handler;
  handle->name = "evbug";

  error = input_register_handle(handle);
  if (error)
    goto err_free_handle;

  error = input_open_device(handle);
  if (error)
    goto err_unregister_handle;

  printk(KERN_DEBUG pr_fmt("Connected device: %s (ptr %p) (%s at %s)\n"),
      dev_name(&dev->dev),
      handle->dev,
      dev->name ?: "unknown",
      dev->phys ?: "unknown");

  return 0;

err_unregister_handle:
  input_unregister_handle(handle);
err_free_handle:
  kfree(handle);
  return error;
}

static void evbug_disconnect(struct input_handle *handle)
{
  printk(KERN_DEBUG pr_fmt("Disconnected device: %s\n"),
      dev_name(&handle->dev->dev));

  input_close_device(handle);
  input_unregister_handle(handle);
  kfree(handle);
}

static const struct input_device_id evbug_ids[] = {
  { .driver_info = 1 },	/* Matches all devices */
  { },			/* Terminating zero entry */
};

MODULE_DEVICE_TABLE(input, evbug_ids);

static struct input_handler evbug_handler = {
  .event =	evbug_event,
  .connect =	evbug_connect,
  .disconnect =	evbug_disconnect,
  .name =		"evbug",
  .id_table =	evbug_ids,
};

static int __init evbug_init(void)
{
  return input_register_handler(&evbug_handler);
}

static void __exit evbug_exit(void)
{
  input_unregister_handler(&evbug_handler);
}

module_init(evbug_init);
module_exit(evbug_exit);
