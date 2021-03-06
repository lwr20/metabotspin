CON
  TO_CM = 29_034                                                                ' Centimetres
                                                                                 
PUB Ticks(TrigPin, EchoPin) : Microseconds | cnt1, cnt2
''Return Ping)))'s one-way ultrasonic travel time in microseconds
                                                                                 
  outa[TrigPin]~                                                                    ' Clear I/O Pin
  dira[TrigPin]~~                                                                   ' Make Pin Output
  outa[TrigPin]~~                                                                   ' Set I/O Pin
  outa[TrigPin]~                                                                    ' Clear I/O Pin (> 2 µs pulse)

  dira[EchoPin]~                                                                    ' Make I/O Pin Input
  'cnt2 := cnt
  waitpne(0, |< EchoPin, 0)                                                         ' Wait For Pin To Go HIGH
  ' repeat until (INA [EchoPin] == 1) or ((cnt - cnt2) > (clkfreq / 25))
  cnt1 := cnt                                                                   ' Store Current Counter Value
  waitpeq(0, |< EchoPin, 0)                                                         ' Wait For Pin To Go LOW 
  ' repeat until (INA [EchoPin] == 0) or ((cnt - cnt1) > (clkfreq / 25))
  cnt2 := cnt                                                                   ' Store New Counter Value
  Microseconds := (||(cnt1 - cnt2) / (clkfreq / 1_000_000)) >> 1                ' Return Time in µs
                                                                               
PUB Centimetres(TrigPin, EchoPin) : Distance                                                  
''Measure object distance in centimetres
                                              
  Distance := Millimetres(TrigPin, EchoPin) / 10                                             ' Distance In Centimetres
                                                                                 
PUB Millimetres(TrigPin, EchoPin) : Distance                                                  
''Measure object distance in millimetres
                                              
  Distance := Ticks(TrigPin, EchoPin) * 10_000 / TO_CM                                       ' Distance In Millimetres

