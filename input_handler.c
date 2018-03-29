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

#include "controller_id.h"

MODULE_AUTHOR("Jacob McIntosh <nacitar@ubercpp.com>");
MODULE_DESCRIPTION("Module to hook into the kernel's input subsystem.");
MODULE_LICENSE("GPL");


#define HANDLER_NAME "universal_input_device"

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


// input_value is type code and value
// input_dev.absinfo


// break axes into positive and negative directions
#define UGC_MAX_DEVICES 256u
#define UGC_CONFIGURE_REPEAT_COUNT 10u
#define UGC_MAX_INPUTS 50

struct ugc_input {
  unsigned int type;
  unsigned int code;
};

enum ugc_config_state {
  CONNECTED, CONFIGURING, READY
};
// start with final button to configure; store that as terminal
struct ugc_device {
  enum ugc_config_state config_state;
  unsigned int repeat_count;
  struct ugc_input last_input;
  struct rb_root input_code_to_index;  // = RB_ROOT;
  __u32 input_state[UGC_MAX_INPUTS];
};

// 2 chars, [0] = a char id, [1] = null terminator
// Index 0 is "\0", empty string... but valid still.
const char ugc_handle_name[UGC_MAX_DEVICES][2] = {
#define UGC_ID_GEN(offset) \
  {(char)offset+0}, {(char)offset+1}, {(char)offset+2}, {(char)offset+3}, \
  {(char)offset+4}, {(char)offset+5}, {(char)offset+6}, {(char)offset+7}, \
  {(char)offset+8}, {(char)offset+9}, {(char)offset+10}, {(char)offset+11}, \
  {(char)offset+12}, {(char)offset+13}, {(char)offset+14}, {(char)offset+15}
  UGC_ID_GEN(0x00), UGC_ID_GEN(0x10), UGC_ID_GEN(0x20), UGC_ID_GEN(0x30),
  UGC_ID_GEN(0x40), UGC_ID_GEN(0x50), UGC_ID_GEN(0x60), UGC_ID_GEN(0x70),
  UGC_ID_GEN(0x80), UGC_ID_GEN(0x90), UGC_ID_GEN(0xA0), UGC_ID_GEN(0xB0),
  UGC_ID_GEN(0xC0), UGC_ID_GEN(0xD0), UGC_ID_GEN(0xE0), UGC_ID_GEN(0xF0)
#undef UGC_ID_GEN
};

struct ugc_handle_group {
  unsigned long acquiredbit[BITS_TO_LONGS(UGC_MAX_DEVICES)];
  unsigned int num_acquired;
};

const char* ugc_handle_name_acquire(struct ugc_handle_group* group) {
  if (group->num_acquired < UGC_MAX_DEVICES) {
    const int index = find_first_zero_bit(group->acquiredbit, UGC_MAX_DEVICES);
    set_bit(index, group->acquiredbit);
    ++group->num_acquired;
    return ugc_handle_name[index];
  }
  printk(KERN_DEBUG pr_fmt("Cannot acquire handle; max devices reached.\n"));
  return NULL;
}

void ugc_handle_name_release(struct ugc_handle_group* group,
    const char* name) {
  if (test_and_clear_bit(*name, group->acquiredbit)) {
    --group->num_acquired;
  } else {
    printk(KERN_DEBUG pr_fmt("Cannot release %d; it is not acquired.\n"),
        (int)*name);
  }
}

struct ugc_handle_group g_handle_group = {0};
struct ugc_device g_devices[UGC_MAX_DEVICES];  // TODO: cleared/initialized along with handle id acquisition
unsigned int g_device_count = 0;

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
  handle->name = ugc_handle_name_acquire(&g_handle_group);

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
  ugc_handle_name_release(&g_handle_group, handle->name);
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
  .name =		HANDLER_NAME,
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
