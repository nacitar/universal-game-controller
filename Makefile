
obj-m += universal_game_controller.o
universal_game_controller-objs := \
  ./src/universal_game_controller.o \
  ./src/controller_id.o \
  ./src/info_strings.o \
  ./src/input_state.o \
  ./src/pin_config.o

ccflags-y := -I$(src)/include

all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules


clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
