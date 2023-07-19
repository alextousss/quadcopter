
plot 'data6.csv' using 0:1 with lines title 'X',\
'data6.csv' using 0:2 with lines title 'Y',\
'data6.csv' using 0:3 with lines title 'X point',\
'data6.csv' using 0:4 with lines title 'Y point',\
'data6.csv' using 0:5 with lines title 'M0',\
'data6.csv' using 0:7 with lines title 'M2',\
'data6.csv' using 0:(-3*$2) with lines title 'csigne X point'


while (1) {  # make a new 'gpio.dat' every cycle with fresh data
    replot
    pause 3
}
