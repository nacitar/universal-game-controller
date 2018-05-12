#include <ugc/pin_config.h>

bool PinConfig_HasInterrupt(struct PinConfig *config) {
  return ((config->input_irq_flags & IRQF_TRIGGER_MASK)
          && config->input_irq_handler);
}

int PinConfig_Setup(struct PinConfig *config) {
  int result;
  if (!gpio_is_valid(config->pin_number)) {
    printk(KERN_DEBUG pr_fmt("%s GPIO not valid: %d\n"),
        config->label, config->pin_number);
    return -ENXIO;
  }
  result = gpio_request(config->pin_number, config->label);
  if (result != 0) {
    printk(KERN_DEBUG pr_fmt("%s GPIO request failed with code: %d\n"),
        config->label, result);
    return result;
  }
  switch (config->direction) {
    case kInput: {
      result = gpio_direction_input(config->pin_number);
      if (result != 0) {
        printk(KERN_DEBUG pr_fmt("%s GPIO input direction setting failed"
            " with code: %d\n"), config->label, result);
        goto cleanup_gpio_request;
      }
      if (PinConfig_HasInterrupt(config)) {
        result = gpio_to_irq(config->pin_number);
        if (result < 0) {
          printk(KERN_DEBUG pr_fmt("%s GPIO to IRQ failed with code: %d\n"),
              config->label, result);
          goto cleanup_gpio_request;
        }
        config->input_irq_number = result;
        result = request_irq(
            config->input_irq_number,
            config->input_irq_handler,
            config->input_irq_flags,
            config->label,
            NULL);
        if (result != 0) {
          printk(KERN_DEBUG pr_fmt("%s GPIO to IRQ failed with code: %d\n"),
              config->label, result);
          goto cleanup_gpio_request;
        }
      }
      break;
    }
    case kOutput: {
      result = gpio_direction_output(config->pin_number, config->output_value);
      if (result != 0) {
        printk(KERN_DEBUG pr_fmt("%s GPIO output direction setting failed"
            " with code: %d\n"), config->label, result);
        goto cleanup_gpio_request;
      }
      break;
    }
    default: {
      result = -EINVAL;
      printk(KERN_DEBUG pr_fmt("%s GPIO invalid direction: %d\n"),
          config->label, config->direction);
      goto cleanup_gpio_request;
    }
  }
  return 0;
cleanup_gpio_request:
  gpio_free(config->pin_number);
  return result;
}
void PinConfig_Release(struct PinConfig *config) {
  switch (config->direction) {
    case kInput: {
      if (PinConfig_HasInterrupt(config)) {
        free_irq(config->input_irq_number, NULL);
      }
      gpio_free(config->pin_number);
      break;
    }
    case kOutput: {
      gpio_free(config->pin_number);
      break;
    }
    default: {
      printk(KERN_DEBUG pr_fmt("%s GPIO invalid direction: %d\n"),
          config->label, config->direction);
    }
  }
}

