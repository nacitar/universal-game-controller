#ifndef INCLUDED_UGC_PIN_CONFIG_H_
#define INCLUDED_UGC_PIN_CONFIG_H_

#include <linux/interrupt.h>
#include <linux/gpio.h>
enum PinState {
  kLow = 0,
  kHigh = 1,
};
enum PinDirection {
  kInput,
  kOutput,
};
struct PinConfig {
  const char* label;
  int pin_number;
  enum PinDirection direction;

  // for output
  int output_value;

  // for input
  unsigned long input_irq_flags;
  int input_irq_number;
  irq_handler_t input_irq_handler;
};

bool PinConfig_HasInterrupt(struct PinConfig *config);
bool PinConfig_Setup(struct PinConfig *config);
void PinConfig_Release(struct PinConfig *config);

#endif  // INCLUDED_UGC_PIN_CONFIG_H_
