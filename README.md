![img](IMG/iic.png)


```
#define SCL BIT_ADDR((GPIOB_BASE+12), 15)
#define SDA BIT_ADDR((GPIOB_BASE+12), 13)
#define AD0 BIT_ADDR((GPIOC_BASE+12), 6)

#define SDA_IN() {GPIOB->CRH&=0xFF0FFFFF;GPIOB->CRH|=0x00700000;}

#define GPIOB_IDR_Addr (GPIOB_BASE + 8)
#define READ_SDA BIT_ADDR(GPIOB_IDR_Addr, 13)
```



```
$ git clone https://github.com/ARMinARM/stm32flash
$ cd stm32flash
$ make
```

## coreutils
```
$ brew install coreutils
```
## gcc-arm-none-eabi
```
$ brew install gcc-arm-none-eabi
```

