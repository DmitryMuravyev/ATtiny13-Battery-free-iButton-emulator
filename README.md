<h1>Features</h1>

- Up to 8 DS1990 keys stored in EEPROM.
- A button for quick access to keys from the 4th to the 8th.
- Any battery with a voltage of 8-30V is suitable for programming (for example, CR-9V).
- A bridge rectifier to prevent wrong battery polarity.
- Works with most popular intercoms.
- No special programmer or computer is needed to store keys.

 
Battery-free emulator of the well-known iButton key based on ATtiny13. The body of the key is the same size as the original one, and the microcontroller's EEPROM memory allows to store up to 8 key codes.
The key itself runs on parasitic power, but programming requires an external 5-30 volt power source, for example, a CR-9V (PP3) battery.
The device is designed just for fun and as a souvenir.

See the detailed review of the project on my YT-channel (English subtitles): https://youtu.be/5b73LqXjfCg

[![Click to see](https://img.youtube.com/vi/5b73LqXjfCg/maxresdefault.jpg)](https://www.youtube.com/watch?v=5b73LqXjfCg)

**Attention!**

**This project dedicated for research purposes only. All information carried within is exploratory in nature. The creator (me) take no responsibility for any incorrect use of the technologies mentioned.**

<h1>Details</h1>

In many respects, the project is based on my previous projects of fan controllers on [ATtiny10](https://github.com/DmitryMuravyev/ATtiny10-DS18B20-Fan-Controller) and [ATtiny13](https://github.com/DmitryMuravyev/ATtiny13-DS18B20-Fan-Controller).

![Schematic_iButton Emulator](https://github.com/DmitryMuravyev/ATtiny13-Battery-free-iButton-emulator/assets/152902525/0fbbe6d8-caf4-475f-a761-9a1ce4ac8420)

You can find the project schematic, PCB drawing and other materials here: https://oshwlab.com/sneer2sneer/battery-free-ibutton-emulator-up-to-8-keys

For MCU Flash ROM programming you will need a USBasp programmer (or any other that supports the TPI - Tiny Programming Interface), which is connected according to this schematic:

![USBasp connection](https://github.com/DmitryMuravyev/ATtiny13-Battery-free-iButton-emulator/assets/152902525/d1147d06-eb23-4dae-adb8-1d47c9776b5c)

The programming itself is performed from the Arduino IDE, with the ATtiny13 core installed in it (see the links below).

${\color{red}Please \space note \space (!)}$ that the code is written for the MCU clock frequency of 8 MHz. This means that in the Arduino IDE, in the Tools->Clock menu, you should set the value to "9.6 MHz internal osc.", and then preferably to slow down the clock by calibrating the value of the OSCCAL register in the code (this value will be unique for every specific exemplar of the chip). This can be done, for example, by uncommenting "Pulse generation" section of the code and measuring the duration of the pulses using an oscilloscope.

<h1>Commands</h1>

Uploading/fuses:

    avrdude -v -pt13 -cusbasp -Pusb -B32 -Uhfuse:w:0xff:m -Ulfuse:w:0b00111010:m -Uflash:w:iButton_Emulator.ino.hex:i

Decompiling binary file:

    avr-objdump -Dzmavr:25 Binary_file.hex >> Disassebled_file.asm

Decompiling ELF file (Executable and Linking Format):

    avr-objdump -S Binary_file.elf >> Disassebled_file.elf.asm

<h1>Links</h1>

Project - https://oshwlab.com/sneer2sneer/battery-free-ibutton-emulator-up-to-8-keys

Additional project files - https://drive.google.com/drive/folders/1QQMyIkBVng6_JfQlNiIVPIaYJ0QIDTjX

ATtiny13 Datasheet - https://ww1.microchip.com/downloads/en/DeviceDoc/ATtiny13A-Data-Sheet-DS40002307A.pdf

Tips and Tricks to Optimize Code for 8-bit AVR - https://ww1.microchip.com/downloads/en/AppNotes/doc8453.pdf

1-Wire protocol (Book of iButton standards) - https://pdfserv.maximintegrated.com/en/an/AN937.pdf

USBasp firmware - https://www.fischl.de/usbasp/

USBasp firmware update guide - https://www.electronics-lab.com/project/usbasp-firmware-update-guide/

ATTinyCore Universal for Arduino IDE - https://github.com/SpenceKonde/ATTinyCore

Battery-free 4in1 DS1990 emulator based on tiny13 - https://anyram.net/blog_ru/?p=1050



