SUBDIRS := addiu addu bitwise branch jr load_store lui quick_sort \
           ooo_deps ooo_mem ooo_branch \
           matrix_mult prime_sieve fib_memo binary_search linked_list

.PHONY: all $(SUBDIRS)

all: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

clean:
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean; \
	done