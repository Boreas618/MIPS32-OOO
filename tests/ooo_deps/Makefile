include ../common.mk

NAME = ooo_deps

$(NAME).bin: $(NAME).S
	$(CC) $(CFLAGS) $(NAME).S -o $(NAME).o
	$(LD) $(LDFLAGS) $(NAME).o -Ttext=0x1000 -o $(NAME).elf
	$(OBJCOPY) $(OBJCOPY_FLAGS) $(NAME).elf $(NAME).bin

clean:
	rm -f $(NAME).o $(NAME).elf $(NAME).bin
