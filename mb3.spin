{{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// METABOT3 Motor Controller
//
// Serial interface, see help messages below
//
// I2C interface : registers
//      0 :     Motor 0 position (4 bytes)
//      4 :     Motor 1 position (4 bytes)
//      8 :     Motor 2 position (4 bytes)
//      12 :    Motor 3 position (4 bytes)
//      16 :    Left Ping distance in mm (2 bytes)
//      18 :    Right Ping distance in mm (2 bytes)
//      20 :    Front Ping distance in mm (2 bytes)
//      22 :    Target Motor 0 Speed (1 byte) signed -127 - 127
//      23 :    Target Motor 1 Speed (1 byte)
//      24 :    Target Motor 2 Speed (1 byte)
//      25 :    Target Motor 3 Speed (1 byte)
//      26 :    Options (low bit = autoping on / off)
//      27 :    Read Ready (master sets to 1 when ready to read, slave sets to zero when multi-byte values updated
// 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}}

CON
  _CLKMODE = xtal1 + pll16x
  _XINFREQ = 6_000_000
  debuglim = 8
  posbase = 0
  pingbase = 16
  speedbase = 22
  options = 26
  readready = 27

OBJ
  ps : "propshell"
  quad :  "encoder"
  pwm : "pibal_pwm"
  i2c : "I2C_slave"
  ping : "Ping"

DAT
' Encoder wire colours : grey, yellow
  encoderPins byte 18, 17, 15, 16, 13, 14, 12, 11
  
' Motor wire colours : 
'                Prop    Controller
'        grey    22, 3   7   INA1
'        yellow  21, 2   6   PWM2
'        purple  20, 1   5   PWM1
'        green   19, 0   4   INA2

' Spares
'         grey   25
'         blue   26
'         green  27

  motorPWM    byte 1, 2, 20, 21
  motorD1     byte 3, 0, 22, 19

' Pinger pins  
  trigPins      byte 6, 8, 24   ' yellow
  echoPins      byte 7, 9, 23   ' purple

VAR
  long  pidstack[30]
  long  pingstack[30]
  word  pingval[3]
  long  lastpos[4]
  long  debug[debuglim]
  long  actual_speed[4]
  long  error_integral[4]
  long  error_derivative[4]
  long  millidiv
  long  millioffset
  long  Kp
  long  Ki
  long  Kd
  byte  b
  
PUB main
  millidiv := clkfreq / 1000
  Kp := 20
  Ki := 2
  Kd := 10
  millioffset := negx / millidiv * -1
  ps.init(string(">"), string("?"), 115200, 31, 30)   ' start the command interpreter shell (COG 1)
  i2c.start(28,29,$42)                                ' (COG 2)
  quad.Start(@encoderPins)                            ' start the quadrature encoder reader (COG 3)
  resetMotors                                         ' reset the motors
  pwm.start_pwm(motorPWM[0], motorPWM[1], motorPWM[2], motorPWM[3], 20000)    ' start the pwm driver (COGS 4 & 5)
  cognew(pid, @pidstack)                              ' COG 6
  cognew(autoping, @pingstack)                        ' COG 7 (last one)
  
  ' Very weird if I add one more line to main it causes the ps prompt to get corrupted
  ' Line below is least necessary so commenting out
  ' ps.puts(string("Propeller starting...", ps#CR))

  repeat
    update
    waitcnt(millidiv + cnt)
  
  '  result := ps.prompt
  '  \cmdHandler(result)

PRI cmdHandler(cmdLine)
  cmdSetSpeed(ps.commandDef(string("+ss"), string("Set speed <motor[0..3], speed[-100...100]>") , cmdLine))
  cmdSetSpeedAll(ps.commandDef(string("+sa"), string("Set all speeds <speed[-100...100] x 4>") , cmdLine))
  cmdSetStop(ps.commandDef(string("+st"), string("Stop") , cmdLine))
  cmdSetPID(ps.commandDef(string("+sp"), string("Set PID Parameters <Kp Ki Kd>") , cmdLine))
  cmdGetPos(ps.commandDef(string("+gp"), string("Get position") , cmdLine))
  cmdGetSpeed(ps.commandDef(string("+gs"), string("Get speed") , cmdLine))
  cmdGetTime(ps.commandDef(string("+gt"), string("Get time") , cmdLine))
  cmdPing(ps.commandDef(string("+p"), string("Ping") , cmdLine))
  cmdGetDebug(ps.commandDef(string("+gd"), string("Get debug") , cmdLine))
  ps.puts(string("? for help", ps#CR))  ' no command recognised
  return true

PRI cmdSetSpeedAll(forMe) | motor, newspeed
  if not forMe
    return
  repeat motor from 0 to 3
    ps.parseAndCheck(motor+1, string("!ERR 1"), true)
    newspeed := ps.currentParDec
    ps.putd(newspeed)
    ps.puts(string(" "))
    i2c.put(speedbase + motor, newspeed)
  ps.puts(string(ps#CR))
  ps.commandHandled
  
PRI cmdSetSpeed(forMe) | motor, newspeed
  if not forMe
    return
  ps.parseAndCheck(1, string("!ERR 1"), true)
  motor := ps.currentParDec
  if motor < 0 or motor > 3
    ps.puts(string("!ERR 2", ps#CR))
    abort
  ps.parseAndCheck(2, string("!ERR 3"), true)
  newspeed := ps.currentParDec
  ps.puts(string("Set Motor Speed "))
  ps.putd(motor)
  ps.puts(string(", "))
  ps.putd(newspeed)
  ps.puts(string(ps#CR))
  i2c.put(speedbase + motor, newspeed)
  ps.commandHandled

PRI cmdSetStop(forMe)
  if not forMe
    return
  resetMotors
  ps.puts(string("Stopped"))
  ps.puts(string(ps#CR))
  ps.commandHandled

PRI cmdSetPID(forMe) | motor, newKp, newKi, newKd
  if not forMe
    return
  ps.parseAndCheck(1, string("!ERR 1"), true)
  newKp := ps.currentParDec
  ps.parseAndCheck(2, string("!ERR 2"), true)
  newKi := ps.currentParDec
  ps.parseAndCheck(3, string("!ERR 3"), true)
  newKd := ps.currentParDec

  ps.puts(string("Set PID Parameters "))
  ps.putd(newKp)
  ps.puts(string(", "))
  ps.putd(newKi)
  ps.puts(string(", "))
  ps.putd(newKd)
  ps.puts(string(ps#CR))
  Kp := newKp
  Ki := newKi
  Kd := newKd
  ps.commandHandled
  
PRI cmdGetPos(forMe) | i
  if not forMe
    return
  repeat i from 0 to 3
    ps.putd(lastpos[i])
    ps.puts(string(" "))
  ps.putd(millioffset + cnt/millidiv)  
  ps.puts(string(ps#CR))
  ps.commandHandled
  
PRI cmdGetSpeed(forMe) | i
  if not forMe
    return
  repeat i from 0 to 3
    ps.putd(actual_speed[i])
    ps.puts(string(" "))
  ps.putd(millioffset + cnt/millidiv)
  ps.puts(string(ps#CR))
  ps.commandHandled

PRI cmdGetTime(forMe)
  if not forMe
    return
  ps.putd(millioffset + cnt/millidiv)
  ps.puts(string(ps#CR))
  ps.commandHandled
  
PRI cmdGetDebug(forMe) | i
  if not forMe
    return
  ps.puts(string(ps#CR))
  repeat i from 0 to debuglim-1
    ps.putd(i)
    ps.puts(string(": "))
    ps.putd(debug[i])
    ps.puts(string(ps#CR))
  ps.commandHandled

PRI cmdPing(forMe) | i
  if not forMe
    return
  if i2c.get(options) == 0         ' if autopinger not running do one manually
    doPing(0)
    doPing(1)
  ps.puts(string("Ping: "))
  ps.putd(pingval[0])
  ps.puts(string(" mm, "))
  ps.putd(pingval[1])
  ps.puts(string(" mm"))
  ps.puts(string(ps#CR))
  ps.commandHandled
 
PRI doPing(side) | m
  side := 0 #> side <# 2
  pingval[side] := ping.Millimetres(trigPins[side], echoPins[side])
 
PRI autoPing | i
  i2c.put(options, 0)                                 ' auto-pinger turned off
  i2c.putw(pingbase, 0)                               ' zero ping results
  i2c.put(pingbase+2, 0)                                
  i2c.put(pingbase+4, 0)                                      
  repeat
    ' Do autoping
    if i2c.get(options) > 0                   ' autopinger is enabled
      repeat i from 0 to 2
        doPing(i)
 
PRI update | i
  ' If host is ready to read then write ping and position values
  if i2c.get(readready) == 1
    repeat i from 0 to 2
      i2c.putw(pingbase+i*2, pingval[i])
    repeat i from 0 to 3  
      i2c.putl(posbase+i*4, lastpos[i])
    i2c.put(readready, 0)                   ' reset ready for the next read
 
PRI resetMotors | i
  repeat i from 0 to 3
    i2c.put(speedbase+i,0)
    error_integral[i] := 0
    outa[motorPWM[i]] := %0
    outa[motorD1[i]] := %0
    dira[motorPWM[i]] := %1
    dira[motorD1[i]] := %1

PRI pid | i, nextpos, error, last_error, nexttime, newspeed, desired_speed
  nextpos := 0
  resetMotors    ' enables the direction ports control from this cog
  nexttime := millidiv + cnt
  repeat
    waitcnt(nexttime)
    nexttime += millidiv * 15
    'Here once every 15 milliseconds
   
    repeat i from 0 to 3          ' loop takes just under 1ms to complete
      b := i2c.get(speedbase+i)   
      desired_speed := ~b         ' note sneaky '~' that sign extends the byte value
      nextpos := quad.count(i)
      last_error := desired_speed - actual_speed[i] 
      actual_speed[i] := nextpos - lastpos[i]
      lastpos[i] := nextpos
      error := desired_speed - actual_speed[i] 
      error_derivative[i] := error - last_error
      if desired_speed == 0
        error_integral[i] := 0    ' stop integral wind-up when stopped
      else
        error_integral[i] += error
      newspeed := Kp * error + Ki * error_integral[i] + Kd * error_derivative[i]
      setMotorSpeed(i, newspeed)
      
PRI setMotorSpeed(motor, speed)
  pwm.set_duty(motor, speed)
  
  if speed == 0
    outa[motorD1[motor]] := %0
  elseif speed > 0
    outa[motorD1[motor]] := %0
  else
    outa[motorD1[motor]] := %1
  