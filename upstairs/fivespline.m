s=[tqs^5 5*tqs^4 20*tqs^3
    tqs^4 4*tqs^3 12*tqs^2
    tqs^3 3*tqs^2 6*tqs];
sni=s^-1;
l=[ts/(td+ts)*sw  sw/(td+ts)  0]*sni;
display(l);