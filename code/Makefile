OUT_FNAME=change_me

ARCH=mips-elf

CC=$(ARCH)-gcc
AS=$(ARCH)-as
LD=$(ARCH)-ld
OBJCOPY=$(ARCH)-objcopy

CFLAGS  += -ggdb -O1
AFLAGS  += -ggdb
LDFLAGS += -ggdb
LDFLAGS += -nostdlib -nodefaultlibs -nostartfiles
#LDFLAGS += -Wl,-Ttext,0x80020000
LDFLAGS += -Wl,-Ttext,0x00000000

all:default

.PHONY:clean

%.srec:%
	$(OBJCOPY) -O srec $< $@

%.dat:%.bin
	 hexdump -v -e '"%08x\n" ' $< >$@

%.bin:%
	$(OBJCOPY) --only-section=.text --reverse-bytes=4 -O binary $< $@

%.out:%
	$(OBJCOPY) -O ecoff-bigmips $< $@

%.o:%.S
	$(CC) -D__ASSEMBLY__ $(AFLAGS) -c $< -o $@

%.s:%.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -S $< -o $@

%.o:%.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

# default output
default:$(OUT_FNAME).srec $(OUT_FNAME).dat

# executable file:object file.o
$(OUT_FNAME):$(OUT_FNAME).o
	$(CC) $(LDFLAGS) $^ -o $@

# all generated that would be cleaned
clean:
	rm -f $(OUT_FNAME) $(OUT_FNAME).o $(OUT_FNAME).out \
	$(OUT_FNAME).srec $(OUT_FNAME).dat
