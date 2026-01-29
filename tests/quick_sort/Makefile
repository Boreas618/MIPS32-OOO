include ../common.mk

NAME = quick_sort

hello.bin: $(NAME).bin

$(NAME).bin: $(NAME).c linker_script.ld
	$(CC) $(CFLAGS) -fno-builtin -ffreestanding $(NAME).c -o $(NAME).o
	$(LD) $(LDFLAGS) $(NAME).o -T linker_script.ld -o $(NAME).elf
	$(OBJCOPY) $(OBJCOPY_FLAGS) $(NAME).elf $(NAME).bin

clean:
	rm -f $(NAME).o $(NAME).elf $(NAME).bin
