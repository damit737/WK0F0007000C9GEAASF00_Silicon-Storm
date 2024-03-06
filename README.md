CUBE IDE version:1.13.0
CUBE MX version:6.6.1
TOUCHGFX version:4.18.1
Please note the following:

After each compilation, the programming file should be placed in the "debug" folder with the name "STM32F769BGT6.hex". The programming process should be done using Cube Programmer, and it is important not to use Cube IDE for programming. This is because the graphics data is stored in the external flash, and using the IDE for programming may cause issues.

When using Cube Programmer for programming, please make sure to select "W25Q128JV_STM32F769.stldr" as the external loader.
