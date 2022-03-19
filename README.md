### Inkster example of JPEG decoding

This is an EPDiy /Inkster example for the Arduino ESP32 framework (>= 2.0.2)

![cat](assets/demo_readme.jpg)


```
Performance test using LilygoEPD47 as display:

60747 bytes read from https://loremflickr.com/960/540
I (12425) decode: 291 ms - 960x540 image MCUs:34
I (12425) www-dw: 322 ms - download
I (12425) render: 170 ms - render
I (14006) total: 783 ms - total time spent

```

The original example was a [contribution for the EPDiy component](https://github.com/vroland/epdiy/graphs/contributors) and this is my intent to port it to arduino-esp32 framework.
The JPG version is still faulty and the image render is not as good as the IDF version, still have to found why, but you've been warned that is not the same result.

This contains two examples, and can be used for different epaper displays and boards. Currently it supports only 2 environments but you can extend this and add more:

```
[platformio]
; lilygo47 | 9inch
default_envs = 9inch
; Select between jpg or bmp example build src directory
; jpg | bmp
src_dir = bmp
```

The BMP example seems to work correctly and the results are 100% identical to the IDF original take.

The JPG version has a gamma correction that you can edit in main.cpp:

```
// Affects the gamma to calculate gray (lower is darker/higher contrast)
// Nice test values: 0.9 1.2 1.4 higher and is too bright
double gamma_value = 0.7;
```

Enjoy and just create an issue if you find a bug. I will try to find time to fix it and make it better.
If you like the repository and works for you, please bookmark it with a ⭐