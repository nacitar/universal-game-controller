#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/device.h>

#include <ugc/controller_id.h>
#include <ugc/info_strings.h>
#include <ugc/input_state.h>
#include <ugc/pin_config.h>

#include <linux/interrupt.h>
#include <linux/gpio.h>

MODULE_AUTHOR("Jacob McIntosh <nacitar@ubercpp.com>");
MODULE_DESCRIPTION("Allows usage of arbitrary input devices on "
    "retro game consoles.");
MODULE_LICENSE("GPL");


#define HANDLER_NAME "universal_game_controller"


// https://lwn.net/Articles/443043/


static irqreturn_t SnesClockRisingInterrupt(int irq, void *dev_id) {
   unsigned long flags;
   // disable hard interrupts (remember them in flag 'flags')
   local_irq_save(flags);

   //printk(KERN_NOTICE "Interrupt [%d] for device %s was triggered !.\n",
   //       irq, (char *) dev_id);

   // restore hard interrupts
   local_irq_restore(flags);
   return IRQ_HANDLED;
}
static irqreturn_t SnesLatchFallingInterrupt(int irq, void *dev_id) {
   unsigned long flags;
   // disable hard interrupts (remember them in flag 'flags')
   local_irq_save(flags);

   //printk(KERN_NOTICE "Interrupt [%d] for device %s was triggered !.\n",
   //       irq, (char *) dev_id);

   // restore hard interrupts
   local_irq_restore(flags);
   return IRQ_HANDLED;
}

static struct PinConfig g_snes_data = {
  .label = "snes_data",
  .pin_number = 11,  // BCM 17
  .direction = kOutput,
  .output_value = kLow,
};
static struct PinConfig g_snes_clock = {
  .label = "snes_clock",
  .pin_number = 13,  // BCM 27
  .direction = kInput,
  .input_irq_flags = IRQF_TRIGGER_RISING,
  .input_irq_handler = SnesClockRisingInterrupt,
};
static struct PinConfig g_snes_latch = {
  .label = "snes_latch",
  .pin_number = 15,  // BCM 22
  .direction = kInput,
  .input_irq_flags = IRQF_TRIGGER_FALLING,  // rising too? to store inputs
  .input_irq_handler = SnesLatchFallingInterrupt,
};

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
#define UGC_NAME_TO_INDEX(name) (+(unsigned char)*(name))

// axis < 0 and > 0 distinct inputs
const __u32 g_UGC_MAX_VALUE = U32_MAX;
const __u32 g_UGC_MIN_PRESSED_VALUE = U32_MAX / 2;

enum ConfigState {
  kConnected=0, kConfiguring, kReady
};

// start with final button to configure; store that as terminal
struct Device {
  struct input_dev *dev;  // name, uniq, phys, id.bustype
  enum ConfigState config_state;
  unsigned int count;  // repeat count in kConfiguring state, button count in kReady state
  struct InputState last_input;
  struct InputState input_nodes[UGC_MAX_INPUTS];  // storage for nodes of the tree
  struct rb_root input_code_to_index;  // = RB_ROOT; but that just zeroes...
  __u32 input_state[UGC_MAX_INPUTS];
};

struct DeviceGroup {
  unsigned long acquiredbit[BITS_TO_LONGS(UGC_MAX_DEVICES)];
  unsigned int num_acquired;
};

// 2 chars, [0] = a char id, [1] = null terminator
// Index 0 is "\0", empty string... but valid still.
const char kDeviceName[UGC_MAX_DEVICES][2] = {
#define UGC_NAME_PT(offset) \
  {(char)offset+0}, {(char)offset+1}, {(char)offset+2}, {(char)offset+3}, \
  {(char)offset+4}, {(char)offset+5}, {(char)offset+6}, {(char)offset+7}, \
  {(char)offset+8}, {(char)offset+9}, {(char)offset+10}, {(char)offset+11}, \
  {(char)offset+12}, {(char)offset+13}, {(char)offset+14}, {(char)offset+15}
  UGC_NAME_PT(0x00), UGC_NAME_PT(0x10), UGC_NAME_PT(0x20), UGC_NAME_PT(0x30),
  UGC_NAME_PT(0x40), UGC_NAME_PT(0x50), UGC_NAME_PT(0x60), UGC_NAME_PT(0x70),
  UGC_NAME_PT(0x80), UGC_NAME_PT(0x90), UGC_NAME_PT(0xA0), UGC_NAME_PT(0xB0),
  UGC_NAME_PT(0xC0), UGC_NAME_PT(0xD0), UGC_NAME_PT(0xE0), UGC_NAME_PT(0xF0)
#undef UGC_NAME_PT
};

const char* DeviceNameAcquire(struct DeviceGroup* group) {
  if (group->num_acquired < UGC_MAX_DEVICES) {
    const int index = find_first_zero_bit(group->acquiredbit, UGC_MAX_DEVICES);
    set_bit(index, group->acquiredbit);
    ++group->num_acquired;
    return kDeviceName[index];
  }
  printk(KERN_DEBUG pr_fmt("Cannot acquire name; max devices reached.\n"));
  return NULL;
}

void DeviceNameRelease(struct DeviceGroup* group,
    const char* name) {
  if (test_and_clear_bit(UGC_NAME_TO_INDEX(name), group->acquiredbit)) {
    --group->num_acquired;
  } else {
    printk(KERN_DEBUG pr_fmt("Cannot release name %d; it is not acquired.\n"),
        UGC_NAME_TO_INDEX(name));
  }
}

// scales the value into the range of a __u32
__u32 NormalizeValue(__u32 value, __s32 minimum, __s32 maximum) {
  return (__u32)((__u64)((__s64)value - minimum) * (((__u64)1 << 32) - 1) /
      (__u64)((__s64)maximum - minimum));
}

static struct DeviceGroup g_device_group = {0};
static struct Device g_devices[UGC_MAX_DEVICES];


static int ConnectDevice(struct input_handler *handler, struct input_dev *dev,
    const struct input_device_id *id)
{
  struct input_handle *handle;
  int error;
  const char* bus_name;
  const char* device_name;
  handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
  if (!handle) {
    return -ENOMEM;
  }

  device_name = DeviceNameAcquire(&g_device_group);
  if (!device_name) {
    printk(KERN_DEBUG pr_fmt("Device connected, but no names available.\n"));
    return 0;
  }
  handle->dev = dev;
  handle->handler = handler;
  handle->name = device_name;

  error = input_register_handle(handle);
  if (error)
    goto err_free_handle;

  error = input_open_device(handle);
  if (error)
    goto err_unregister_handle;

  g_devices[UGC_NAME_TO_INDEX(device_name)] = (struct Device) {
    .dev = dev,
    .input_code_to_index = RB_ROOT
  };

  GetBusName(dev->id.bustype, &bus_name);

  printk(KERN_DEBUG pr_fmt("Connected device: [%s] %s (%s) at %s\n"),
      bus_name,
      dev->name ?: "unknown",
      dev->uniq ?: "unknown",
      dev->phys ?: "unknown");

  return 0;

err_unregister_handle:
  input_unregister_handle(handle);
err_free_handle:
  kfree(handle);
  return error;
}

static void DisconnectDevice(struct input_handle *handle)
{
  const char* bus_name;
  GetBusName(handle->dev->id.bustype, &bus_name);

  // no need to cleanup the device itself; all storage is static and
  // it is cleared when reused
  DeviceNameRelease(&g_device_group, handle->name);

  printk(KERN_DEBUG pr_fmt("Disconnected device: [%s] %s (%s) at %s\n"),
      bus_name,
      handle->dev->name ?: "unknown",
      handle->dev->uniq ?: "unknown",
      handle->dev->phys ?: "unknown");

  input_close_device(handle);
  input_unregister_handle(handle);
  kfree(handle);
}

static void EventHandler(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
  const char *event_name, *code_name, *bus_name;
  struct Device *device;
  GetEventName(type, code, &event_name, &code_name);
  GetBusName(handle->dev->id.bustype, &bus_name);
  if (!event_name) {
    event_name = "UNKNOWN";
  }
  if (!code_name) {
    code_name = "UNKNOWN";
  }
  //printk(KERN_DEBUG pr_fmt("Event. Dev: %s, Type: %s[%d], Code: %s[%d], Value: %d\n"),
  //    dev_name(&handle->dev->dev), event_name, type, code_name, code, value);

  device = g_devices + UGC_NAME_TO_INDEX(handle->name);


  if (type == EV_REL || type == EV_KEY) {
    struct InputState this_input = {
      .type = type,
      .code = code,
      .positive = true  // for EV_KEY
    };
    if (type == EV_REL) {
        struct input_absinfo* absinfo = device->dev->absinfo + code;
        if (value < 0) {
          // TODO check absolute negative to ensure output is positive
          this_input.positive = false;
          this_input.value = NormalizeValue(0 - value, 0, -absinfo->minimum);
        } else {
          this_input.value = NormalizeValue(value, 0, absinfo->maximum);
        }
    } else {
      this_input.value = NormalizeValue(value, 0, 1);
    }

    if (device->config_state != kReady &&
        this_input.value >= g_UGC_MIN_PRESSED_VALUE) {
      if (device->config_state == kConnected) {
        if (InputState_Compare(&device->last_input, &this_input) == 0) {
          if (++device->count == 10) {
            device->config_state = kConfiguring;
            device->count = 0;
          }
        } else {
          device->last_input = this_input;
          device->count = 1;
        }
      } else if (device->config_state == kConfiguring) {
        const bool is_terminal = (
            InputState_Compare(&device->last_input, &this_input) == 0);
        struct InputState *node = InputState_Search(
            &device->input_code_to_index, &this_input);
        if (node || (device->count == 0 && is_terminal)) {
          // no double bindings, and first input can't be terminal
          return;
        }
        node = device->input_nodes + device->count;
        *node = this_input;
        node->value = device->count;
        // TODO output more
        printk(KERN_DEBUG pr_fmt("Adding button: %u, Code %u\n"),
            node->value, node->code);
        if (!InputState_Insert(&device->input_code_to_index, node)) {
          printk(KERN_DEBUG pr_fmt("FAIL\n"));
        }
        ++device->count;
        if (is_terminal) {
          device->config_state = kReady;
        }
      }
    } else if (device->config_state == kReady) {
      struct InputState *node;
      node = InputState_Search(&device->input_code_to_index, &this_input);
      if (node) {
        device->input_state[node->value] = this_input.value;
        printk(KERN_DEBUG pr_fmt("Button: %u, Value: %u\n"),
            node->value, this_input.value);
      }
    }
  }
}
static const struct input_device_id g_id_match_table[] = {
  { .driver_info = 1 },	/* Matches all devices */
  { },			/* Terminating zero entry */
};

MODULE_DEVICE_TABLE(input, g_id_match_table);

static struct input_handler g_InputHandler = {
  .event =	EventHandler,
  .connect =	ConnectDevice,
  .disconnect =	DisconnectDevice,
  .name =		HANDLER_NAME,
  .id_table =	g_id_match_table,
};

static int __init Init(void)
{
  if (
      PinConfig_Setup(&g_snes_data) &&
      PinConfig_Setup(&g_snes_clock) &&
      PinConfig_Setup(&g_snes_latch)) {
    return input_register_handler(&g_InputHandler);
  }
  return -1;
}

static void __exit Exit(void)
{
  input_unregister_handler(&g_InputHandler);
  PinConfig_Release(&g_snes_data);
  PinConfig_Release(&g_snes_clock);
  PinConfig_Release(&g_snes_latch);
}

module_init(Init);
module_exit(Exit);
