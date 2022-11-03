
.global flashio_worker_begin
.global flashio_worker_end

.balign 4

flashio_worker_begin:
# a0 ... address of SPI ctrl reg
# a1 ... data pointer
# a2 ... data length
# a3 ... optional WREN cmd (0 = disable)

# Set CS high, IO0 is output
li   t1, 0x120
sh   t1, 0(a0)

# Enable Manual SPI Ctrl
sb   zero, 3(a0)

# Send optional WREN cmd
beqz a3, flashio_worker_L1
li   t5, 8
andi t2, a3, 0xff
flashio_worker_L4:
srli t4, t2, 7
sb   t4, 0(a0)
ori  t4, t4, 0x10
sb   t4, 0(a0)
slli t2, t2, 1
andi t2, t2, 0xff
addi t5, t5, -1
bnez t5, flashio_worker_L4
sb   t1, 0(a0)

# SPI transfer
flashio_worker_L1:
beqz a2, flashio_worker_L3
li   t5, 8
lbu  t2, 0(a1)
flashio_worker_L2:
srli t4, t2, 7
sb   t4, 0(a0)
ori  t4, t4, 0x10
sb   t4, 0(a0)
lbu  t4, 0(a0)
andi t4, t4, 2
srli t4, t4, 1
slli t2, t2, 1
or   t2, t2, t4
andi t2, t2, 0xff
addi t5, t5, -1
bnez t5, flashio_worker_L2
sb   t2, 0(a1)
addi a1, a1, 1
addi a2, a2, -1
j    flashio_worker_L1
flashio_worker_L3:

# Back to MEMIO mode
li   t1, 0x80
sb   t1, 3(a0)

fence.i

ret

.balign 4
flashio_worker_end:
