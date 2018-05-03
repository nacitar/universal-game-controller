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

// https://www.kernel.org/doc/Documentation/input/event-codes.txt
// https://github.com/torvalds/linux/blob/master/include/uapi/linux/input-event-codes.h
// https://lwn.net/Articles/443043/

MODULE_AUTHOR("Jacob McIntosh <nacitar@ubercpp.com>");
MODULE_DESCRIPTION("Allows usage of arbitrary input devices on "
    "retro game consoles.");
MODULE_LICENSE("GPL");


#define HANDLER_NAME "universal_game_controller"

#define UGC_MAX_DEVICES 256u
#define UGC_CONFIGURE_REPEAT_COUNT 10u
#define UGC_MAX_INPUTS 50
#define UGC_NAME_TO_INDEX(name) (+(unsigned char)*(name))

const __u32 kPressedThreshold = U32_MAX / 2;

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

// dev of null reuses the existing value
void Device_ResetConfig(struct Device *device, struct input_dev *dev) {
  if (likely(!dev)) {
    dev = device->dev;
  }
  *device = (struct Device) {
    .dev = dev,
    .input_code_to_index = RB_ROOT
  };
}

struct DeviceGroup {
  unsigned long acquiredbit[BITS_TO_LONGS(UGC_MAX_DEVICES)];
  unsigned int num_acquired;
};

static struct DeviceGroup g_device_group = {0};
static struct Device g_devices[UGC_MAX_DEVICES];
// TODO: support multiple later?
static struct Device *g_active_device = NULL;

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

static irqreturn_t SnesLatchChangedInterrupt(int irq, void *dev_id);
static irqreturn_t SnesClockRisingInterrupt(int irq, void *dev_id);

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
  .input_irq_handler = SnesLatchChangedInterrupt,
};

#define SNES_CYCLE_COUNT 16
static const unsigned int kSnesCycleCount = SNES_CYCLE_COUNT;
static bool g_latched_state[SNES_CYCLE_COUNT] = {0};
static unsigned int g_cycle_index = SNES_CYCLE_COUNT;
static bool g_latch_state_known = false;
static enum PinState g_latch_state;


static inline void SnesSendNextButton(void) {
  gpio_set_value(g_snes_data.pin_number,
      (int)g_latched_state[g_cycle_index]);
  ++g_cycle_index;
}

static irqreturn_t SnesLatchChangedInterrupt(int irq, void *dev_id) {
  unsigned long flags;
  // disable hard interrupts (remember them in flag 'flags')
  local_irq_save(flags);

  if (unlikely(!g_latch_state_known)) {
    g_latch_state_known = true;
    g_latch_state = (enum PinState)(
        gpio_get_value(g_snes_latch.pin_number) != 0);
  } else {
    g_latch_state = (enum PinState)(!g_latch_state);
  }
  // timing is less strict for the rise than the fall
  if (unlikely(g_latch_state)) {
    // TODO: optimize this?
    const __u32 * const current_state = g_active_device->input_state;
    // rising edge: save state, initialize offset
    int i;
    for (i=0; i < kSnesCycleCount; ++i) {
      // For SNES, pressed == low, so inverting the check.
      g_latched_state[i] = (current_state[i] <= kPressedThreshold);
    }
    g_cycle_index = 0;
  } else {
    // send first button state
    SnesSendNextButton();
  }
  // restore hard interrupts
  local_irq_restore(flags);
  return IRQ_HANDLED;
}

static irqreturn_t SnesClockRisingInterrupt(int irq, void *dev_id) {
  unsigned long flags;
  // disable hard interrupts (remember them in flag 'flags')
  local_irq_save(flags);
  if (likely(g_cycle_index < kSnesCycleCount)) {
    // send next button state
    SnesSendNextButton();
  } else {
    gpio_set_value(g_snes_data.pin_number, (int)kLow);
  }
  // restore hard interrupts
  local_irq_restore(flags);
  return IRQ_HANDLED;
}


// scales the value into the range of a __u32
__u32 NormalizeValue(__s32 value, __s32 low, __s32 high) {
  if (high < low) {
    return U32_MAX - NormalizeValue(value, high, low);
  }
  if (value < low) {
    value = low;
  } else if (value > high) {
    value = high;
  }
  return (__u64)((__s64)value - low) * (__u64)U32_MAX /
      (__u64)((__s64)high - low);
}


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

  Device_ResetConfig(g_devices + UGC_NAME_TO_INDEX(device_name), dev);

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
  struct Device *device;
  //const char *event_name, *code_name, *bus_name;
  //GetEventName(type, code, &event_name, &code_name);
  //GetBusName(handle->dev->id.bustype, &bus_name);
  //if (!event_name) {
  //  event_name = "UNKNOWN";
  //}
  //if (!code_name) {
  //  code_name = "UNKNOWN";
  //}
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
          this_input.positive = false;
          // TODO: check absinfo->value vs value; prefer abs if match
          this_input.value = NormalizeValue(value, 0, absinfo->minimum);
        } else {
          this_input.value = NormalizeValue(value, 0, absinfo->maximum);
        }
    } else {
      this_input.value = NormalizeValue(value, 0, 1);
    }

    if (unlikely(device->config_state != kReady &&
        this_input.value >= kPressedThreshold)) {
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
          struct Device *old_device;
          device->config_state = kReady;
          old_device = g_active_device;
          g_active_device = device;
          Device_ResetConfig(old_device, NULL);
        }
      }
    } else if (likely(device->config_state == kReady)) {
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

static bool g_is_snes_gpio = false;

static bool setup_snes_gpio(void) {
  if (g_is_snes_gpio) {
    return true;
  }
  if (!PinConfig_Setup(&g_snes_data)) {
    return false;
  }
  if (!PinConfig_Setup(&g_snes_clock)) {
    goto err_release_data;
  }
  if (!PinConfig_Setup(&g_snes_latch)) {
    goto err_release_clock;
  }
  g_is_snes_gpio = true;
  return true;
err_release_clock:
  PinConfig_Release(&g_snes_clock);
err_release_data:
  PinConfig_Release(&g_snes_data);
  return false;
}
static void release_snes_gpio(void) {
  if (g_is_snes_gpio) {
    PinConfig_Release(&g_snes_latch);
    PinConfig_Release(&g_snes_clock);
    PinConfig_Release(&g_snes_data);
    g_is_snes_gpio = false;
  }
}

static bool g_is_handler_registered = false;
static int __init Init(void)
{
  if (setup_snes_gpio()) {
    g_is_handler_registered = true;
    return input_register_handler(&g_InputHandler);
  }
  return -1;
}

static void __exit Exit(void)
{
  release_snes_gpio();
  if (g_is_handler_registered) {
    input_unregister_handler(&g_InputHandler);
  }
}

module_init(Init);
module_exit(Exit);
