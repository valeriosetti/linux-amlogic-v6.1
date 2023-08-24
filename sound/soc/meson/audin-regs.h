
/* Helper to properly translate between register's index reported on the
 * datasheet and the real offet in bytes. */
#define INDEX_TO_OFFSET(idx)	(idx * 4)

#define AUDIN_I2SIN_CTRL        INDEX_TO_OFFSET(0x10)
#define AUDIN_FIFO0_START       INDEX_TO_OFFSET(0x20)
#define AUDIN_FIFO0_END         INDEX_TO_OFFSET(0x21)
#define AUDIN_FIFO0_PTR         INDEX_TO_OFFSET(0x22)
#define AUDIN_FIFO0_INTR        INDEX_TO_OFFSET(0x23)
#define AUDIN_FIFO0_RDPTR       INDEX_TO_OFFSET(0x24)
#define AUDIN_FIFO0_CTRL        INDEX_TO_OFFSET(0x25)
#define AUDIN_FIFO0_CTRL1       INDEX_TO_OFFSET(0x26)
#define AUDIN_FIFO0_LVL0        INDEX_TO_OFFSET(0x27)
#define AUDIN_FIFO0_LVL1        INDEX_TO_OFFSET(0x28)
#define AUDIN_FIFO0_LVL2        INDEX_TO_OFFSET(0x29)
#define AUDIN_FIFO0_REQID       INDEX_TO_OFFSET(0x30)
#define AUDIN_FIFO0_WRAP        INDEX_TO_OFFSET(0x31)
#define AUDIN_FIFO1_START       INDEX_TO_OFFSET(0x33)
#define AUDIN_FIFO1_END         INDEX_TO_OFFSET(0x34)
#define AUDIN_FIFO1_PTR         INDEX_TO_OFFSET(0x35)
#define AUDIN_FIFO1_INTR        INDEX_TO_OFFSET(0x36)
#define AUDIN_FIFO1_RDPTR       INDEX_TO_OFFSET(0x37)
#define AUDIN_FIFO1_CTRL        INDEX_TO_OFFSET(0x38)
#define AUDIN_FIFO1_CTRL1       INDEX_TO_OFFSET(0x39)
#define AUDIN_FIFO1_LVL0        INDEX_TO_OFFSET(0x40)
#define AUDIN_FIFO1_LVL1        INDEX_TO_OFFSET(0x41)
#define AUDIN_FIFO1_LVL2        INDEX_TO_OFFSET(0x42)
#define AUDIN_FIFO1_REQID       INDEX_TO_OFFSET(0x43)
#define AUDIN_FIFO1_WRAP        INDEX_TO_OFFSET(0x44)
#define AUDIN_FIFO2_START       INDEX_TO_OFFSET(0x45)
#define AUDIN_FIFO2_END         INDEX_TO_OFFSET(0x46)
#define AUDIN_FIFO2_PTR         INDEX_TO_OFFSET(0x47)
#define AUDIN_FIFO2_INTR        INDEX_TO_OFFSET(0x48)
#define AUDIN_FIFO2_RDPTR       INDEX_TO_OFFSET(0x49)
#define AUDIN_FIFO2_CTRL        INDEX_TO_OFFSET(0x4a)
#define AUDIN_FIFO2_CTRL1       INDEX_TO_OFFSET(0x4b)
#define AUDIN_FIFO2_LVL0        INDEX_TO_OFFSET(0x4c)
#define AUDIN_FIFO2_LVL1        INDEX_TO_OFFSET(0x4d)
#define AUDIN_FIFO2_LVL2        INDEX_TO_OFFSET(0x4e)
#define AUDIN_FIFO2_REQID       INDEX_TO_OFFSET(0x4f)
#define AUDIN_FIFO2_WRAP        INDEX_TO_OFFSET(0x50)
#define AUDIN_INT_CTRL          INDEX_TO_OFFSET(0x51)
#define AUDIN_FIFO_INT          INDEX_TO_OFFSET(0x52)
