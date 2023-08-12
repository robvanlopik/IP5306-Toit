import ..src.driver show *
import i2c
import gpio
import gpio.pwm



main:
  bl := gpio.Pin 32
  speaker := gpio.Pin 25
  generator := pwm.Pwm --frequency=400
  back_light := generator.start bl

  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  device := bus.device IP5306_ADDR
  power := IP5306 device
  print "power object created"
  print "battery level is $power.battery_level %"
  4.repeat:
    back_light.set_duty_factor 0.8
    print "backlight 50%"
    sleep --ms=1000
    back_light.set_duty_factor 0.1
    sleep --ms=1000

  print "end"
  audio := generator.start speaker
  audio.set_duty_factor 0.01
  sleep --ms=1000
